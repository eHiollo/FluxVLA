#!/usr/bin/env python3
"""A10 terminal commands. Robot protocol and control logic live in A10Operator."""

from __future__ import annotations

import argparse
import json
import math
import queue
from pathlib import Path
import shlex
import sys
import threading
import time

# Prefer this checkout, including when another branch is installed editable.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fluxvla.engines.operators.kaanh_commands import LOCAL_COMMANDS, add_local_command_parsers


HELP = """可用指令：
  reconnect                     重新连接并查询状态，不重发控制指令（仅 shell）
  启动控制台                    python scripts/a10_control.py（默认 shell，允许控制，不自动使能）
  选择机器人配置                启动时加 --config configs/a10/a10_dual_arm_operator.py
  覆盖机器人地址                启动时加 --host IP；连接参数放在 shell 之前
  get [--raw]                   查看状态；--raw 显示完整回复
  manual_en                     手动使能，回读 Enabled
  manual_ds                     手动掉使能，回读 Disabled（不是急停）
  set_jog_coordinate --tool     切换工具坐标系并回读
  set_jog_coordinate --axis     切换关节坐标系并回读
  follower_cart                 启动本连接的笛卡尔跟随（仅 shell）
  follower_aj                   启动本连接的单/双臂关节跟随（仅 shell）
  joints                        查询六轴实际角度（度），只读取不发送目标
  joints --deg q1 ... qN        更新持续发送的绝对角度目标；单臂6个、双臂12个（仅 shell）
  stop_follower                 停止跟随（共享模式也可停止推理；仅 shell）
  pq x y z qx qy qz qw          发送一个 UDP 目标（仅 shell）
  jog left x 5                  左臂沿本次开始时工具 X 轴移动 5 mm（双臂 shell）
  jog right rz 1                右臂绕本次开始时工具 Z 轴旋转 1 度
  jog left x 5 right x 5        两臂各沿自身工具 X 轴移动 5 mm
  g_move --pos=毫米数            发送夹爪目标，范围 0..100 mm
  m_init                        回到初始位置，成功后恢复原控制模式与跟随状态
  set_acc 400 [--id 0|--all]     设置电机±加速度（°/s²），默认全部12轴；卸载程序并掉使能
  stop                          取消当前及排队命令，停止动作下发并请求停止 follower
  其他未注册指令                原文发送到控制器，显示完整回复
  raw 控制器指令                显式透传，可发送与内置名称相同的原生命令
  help                          显示帮助
  exit                          停止本次跟随后关闭连接，不自动掉使能

pq 的 xyz 单位是米，四元数顺序是 xyzw，须为单位四元数。
目标相对于跟随启动基准，不是绝对位置，也不是逐次累加的移动量。
关节目标单位度；各臂关节1到6，双臂按 joint_target_order 排列；不包含夹爪。
关节跟随与末端跟随不能混发；切换前先 stop_follower。
UDP 发出不等于控制器收到或机器人到位；坐标语义需要现场核对。
双臂配置：get/joints 显示各臂；follower_cart/stop_follower 作用于整个控制组。
双臂 jog：轴 x/y/z 单位毫米，rx/ry/rz 单位度；单次最多 10 mm / 5 度。
先确认 ee_id 的物理左右对应，再 manual_en、set_jog_coordinate --tool、follower_cart、jog。
未选择的臂保持；连续 jog 保留累计量；到位检查失败会请求停止整个控制组。
双臂 pq、g_move 仍拒绝；joints 查询会打印完整关节目标模板。
m_init 使用 operator 的 init_position；需先 manual_en；配置的双夹爪与机械臂同时回位。
夹爪由 VR 占用时请使用 VR 的 m_init；控制台不会抢占串口或自动使能夹爪。
Ctrl+C / Ctrl+D 与 exit 一样尝试退出；断网或强制结束无法保证停止。
断线或启动时设备离线仍保留 a10>；恢复后输入 reconnect，只重连并查询。
内置指令保留参数检查和反馈确认；其他指令原文透传，不自动重试。
透传回复 ret_code=0 只表示控制器接受，不表示动作完成；--read-only 禁止透传。
原生命令回复默认等待 60 秒，可用启动参数 --command-timeout 修改。
共享模式下查询不打断推理；控制指令会先停止推理跟随，推理不会自动恢复。
回复超时表示执行结果未知；本地连接会关闭以隔离迟到回复，输入 reconnect 恢复查询。
透传会撤销本地 UDP 目标发送资格；需 stop_follower 后重新启动跟随。
"""
SESSION_COMMANDS = {'follower_cart', 'follower_aj', 'stop_follower', 'pq', 'reconnect', 'jog'}
BUILTIN_COMMANDS = SESSION_COMMANDS | {
    'get', 'manual_en', 'manual_ds', 'exit', 'help',
    'set_jog_coordinate', 'joints', 'g_move',
}


class CommandError(ValueError):
    pass


class CommandParser(argparse.ArgumentParser):
    def error(self, message):
        raise CommandError(message)


def parse_command(tokens, *, raw_text=None):
    if tokens[:1] == ['raw']:
        if len(tokens) < 2:
            raise CommandError('raw 后请输入控制器指令。')
        text = (raw_text.lstrip()[3:].lstrip() if raw_text is not None
                else shlex.join(tokens[1:]))
        return argparse.Namespace(command='controller_command', text=text)
    if tokens and tokens[0] not in BUILTIN_COMMANDS and tokens[0] not in LOCAL_COMMANDS:
        return argparse.Namespace(command='controller_command',
                                  text=raw_text if raw_text is not None else shlex.join(tokens))
    parser = CommandParser(prog='a10', add_help=False, allow_abbrev=False)
    subs = parser.add_subparsers(dest='command', required=True,
                                 parser_class=CommandParser)
    add_local_command_parsers(subs)
    get = subs.add_parser('get', add_help=False, allow_abbrev=False)
    get.add_argument('--raw', action='store_true')
    for name in ('manual_en', 'manual_ds', 'follower_cart', 'follower_aj',
                 'stop_follower', 'exit', 'help', 'reconnect'):
        subs.add_parser(name, add_help=False, allow_abbrev=False)
    coord = subs.add_parser('set_jog_coordinate', add_help=False, allow_abbrev=False)
    coordinates = coord.add_mutually_exclusive_group(required=True)
    coordinates.add_argument('--tool', action='store_true')
    coordinates.add_argument('--axis', action='store_true')
    joints = subs.add_parser('joints', add_help=False, allow_abbrev=False)
    joints.add_argument('--deg', action='store_true')
    joints.add_argument('targets', nargs='*', type=float)
    pose = subs.add_parser('pq', add_help=False, allow_abbrev=False)
    pose.add_argument('pose', nargs=7, type=float)
    jog = subs.add_parser('jog', add_help=False, allow_abbrev=False)
    jog.add_argument('moves', nargs='+')
    gripper = subs.add_parser('g_move', add_help=False, allow_abbrev=False)
    gripper.add_argument('--pos', type=float, required=True)
    # pq has only positional numbers. argparse otherwise mistakes -1e-3 for
    # an option, despite it being a valid small negative Cartesian target.
    if tokens and tokens[0] == 'pq' and tokens[1:2] != ['--']:
        tokens = ['pq', '--', *tokens[1:]]
    if tokens[:2] == ['joints', '--deg'] and tokens[2:3] != ['--']:
        tokens = ['joints', '--deg', '--', *tokens[2:]]
    if tokens and tokens[0] == 'jog' and tokens[1:2] != ['--']:
        tokens = ['jog', '--', *tokens[1:]]
    args = parser.parse_args(tokens)
    if args.command == 'jog':
        if len(args.moves) not in (3, 6):
            raise CommandError('用法：jog left x 5 或 jog left x 5 right x 5')
        args.deltas = {}
        for i in range(0, len(args.moves), 3):
            name, axis, value = args.moves[i:i+3]
            if name in args.deltas or axis not in ('x', 'y', 'z', 'rx', 'ry', 'rz'):
                raise CommandError('每臂只能出现一次，轴为 x/y/z/rx/ry/rz')
            try:
                value = float(value)
            except ValueError as exc:
                raise CommandError('移动量必须是数字') from exc
            rotation = axis.startswith('r')
            if not math.isfinite(value) or abs(value) > (5 if rotation else 10):
                raise CommandError('单次移动量须有限，且不超过 10 mm / 5 度')
            delta = [0.] * 6
            delta[('x', 'y', 'z', 'rx', 'ry', 'rz').index(axis)] = (
                math.radians(value) if rotation else value / 1000)
            args.deltas[name] = delta
    if args.command == 'joints' and (args.targets or args.deg):
        if not args.deg or len(args.targets) not in (6, 12):
            raise CommandError('发送关节目标请使用 joints --deg 加单臂6个或双臂12个绝对角度；查询只输入 joints。')
    return args


def show_state(state, output=print):
    context = state.raw['ret_context']
    info = context.get('robot_msg', {})
    labels = {'Manual': '手动', 'Auto': '自动', 'Enabled': '已使能',
              'Disabled': '未使能', 'Stopped': '已停止'}
    def label(value):
        return '未知' if value is None else f'{labels.get(value, value)} ({value})'
    output(f"操作模式：{label(info.get('op_mode'))}")
    output(f"电机状态：{label(info.get('activate'))}")
    output(f"运动状态：{label(info.get('motion'))}")
    speed = (context.get('motion_msg', {}).get('jog_vel') if info.get('op_mode') == 'Manual'
             else context.get('program_msg', {}).get('vel') if info.get('op_mode') == 'Auto' else None)
    output(f'控制器速度倍率：{speed if speed is not None else "未知"}%')
    output(f"坐标系：{context['motion_msg'].get('jog_coordinate', '未知')}")
    active = {True: '运行中', False: '未运行', None: '未知'}[state.follower_active]
    output(f'跟随状态：{active}；模式：{state.follower_mode}')
    output(f"故障：{'有' if state.has_error else '无'}")
    if state.has_error:
        output('故障详情：' + json.dumps({k: context.get(k) for k in
               ('robot_msg', 'log_msg', 'driver_msg')}, ensure_ascii=False))
    if state.actual_joints_rad is not None:
        output('实际关节（弧度）：' + str(state.actual_joints_rad.tolist()))
    if len(getattr(state, 'arms', {})) > 1:
        output('控制器级状态以上显示；双臂共用一个 follower。')
        for name, arm in state.arms.items():
            output(f'{name}（ee_id={arm.ee_id}）实际关节（弧度）：' +
                   ('暂无反馈' if arm.actual_joints_rad is None else str(arm.actual_joints_rad.tolist())))
            output(f'{name} TCP（米/xyzw）：' +
                   ('暂无已确认单位的反馈' if arm.tcp_pq_m is None else str(arm.tcp_pq_m.tolist())))
    if len(getattr(state, 'arms', {})) > 1:
        output('双臂夹爪：本轮未适配。')
    else:
        output(f'实际夹爪（毫米）：{state.gripper_mm if state.gripper_mm is not None else "暂无反馈"}')


class A10Console:
    def __init__(self, robot, output=print, prepare_connection=None, joint_hz=30.):
        if not math.isfinite(joint_hz) or joint_hz <= 0:
            raise ValueError('joint_hz must be positive and finite')
        self.robot, self.output = robot, output
        self.joint_hz = joint_hz
        self._joint_target = None
        self._joint_lock = threading.Lock()
        self._joint_stop = threading.Event()
        self._joint_thread = None
        self.prepare_connection = prepare_connection
        self.unresolved_follower = False
        self._commands = queue.Queue()
        self._dispatch_lock = threading.Lock()
        self._worker_idle = threading.Event()
        self._worker_idle.set()
        self._worker_interrupt = threading.Event()
        self._generation = 0

    def _stop_joint_stream(self):
        self._joint_stop.set()
        thread = self._joint_thread
        if thread is not None:
            thread.join(timeout=max(5., 2 * self.robot.timeout_s + 1.))
            if thread.is_alive():
                raise TimeoutError('关节发送线程尚未退出，禁止开始新动作。')
        with self._joint_lock:
            self._joint_target = None
            self._joint_thread = None

    def _set_joint_target(self, targets):
        if self._joint_stop.is_set():
            self._stop_joint_stream()
        with self._joint_lock:
            # Validate and send before replacing the previous accepted target.
            self.robot.send_joints(targets)
            self._joint_target = {name: list(q) for name, q in targets.items()}
            if self._joint_thread is None:
                self._joint_stop.clear()
                self._joint_thread = threading.Thread(
                    target=self._send_joint_stream, name='a10-console-joints', daemon=True)
                self._joint_thread.start()

    def _send_joint_stream(self):
        interval = 1. / self.joint_hz
        next_send = time.monotonic() + interval
        last_feedback = time.monotonic()
        try:
            while not self._joint_stop.wait(max(0., next_send - time.monotonic())):
                with self._joint_lock:
                    if self._joint_stop.is_set():
                        break
                    if time.monotonic() - last_feedback >= .1:
                        self.robot.get_state()
                        last_feedback = time.monotonic()
                    if self._joint_stop.is_set():
                        break
                    self.robot.send_joints(self._joint_target)
                # Slow I/O never causes a burst of overdue packets.
                next_send += interval
                if next_send <= time.monotonic():
                    next_send = time.monotonic() + interval
        except Exception as exc:
            if not self._joint_stop.is_set():
                self._joint_stop.set()
                self.output(f'关节持续发送已中止：{exc}；不会自动恢复。')
                try:
                    self.robot.stop_all_actions()
                except Exception as stop_error:
                    self.output(f'机器人停止未确认：{stop_error}')

    def observe(self, state):
        if state.follower_active is False:
            self.unresolved_follower = False
        return state

    def report_error(self, exc):
        self.output(f'操作未完成：{exc}')
        if not self.robot.is_connected:
            self.unresolved_follower |= self.robot.follower_stop_pending
            self.output('控制连接当前不可用，控制台保留。输入 reconnect 重新连接并查询；不会自动重发控制指令。')
            if self.unresolved_follower:
                self.output('之前的跟随尚未确认停止，请通过现场控制器确认；重连不会自动接管。')

    def reconnect(self):
        self._stop_joint_stream()
        # An explicit reconnect mustn't discard a live session's ownership.
        if self.robot.is_connected and self.robot.follower_stop_pending:
            state = self.observe(self.robot.get_state())
            self.output('当前连接有待停止的跟随，连接已保留；请先输入 stop_follower。')
        else:
            self.unresolved_follower |= self.robot.follower_stop_pending
            self.robot.close()
            if self.prepare_connection is not None:
                self.prepare_connection()
            self.robot.connect()
            state = self.observe(self.robot.get_state())
            self.output('连接成功（状态读取已确认），未发送使能或运动指令。')
        show_state(state, self.output)
        if self.unresolved_follower:
            self.output('旧会话的跟随尚未确认停止，请在现场控制器处理；新连接不继承控制权限。')

    def execute(self, args):
        # Keep a multi-step console operation (notably m_init) exclusive until
        # its speed/mode restoration is complete. Queries never claim control.
        shared = getattr(self.robot, 'shared_control', None)
        controls = (args.command not in ('get', 'help', 'exit', 'reconnect', 'm_init', 'set_acc')
                    and not (args.command == 'joints' and not args.targets))
        reserved = (controls and isinstance(shared, dict) and shared.get('role') == 'console'
                    and self.robot.allow_motion and self.robot.is_connected)
        if reserved:
            self.robot.begin_shared_console_operation()
        try:
            return self._execute(args)
        finally:
            if reserved:
                self.robot.end_shared_console_operation()

    def _execute(self, args):
        """Execute parsed commands via public operator methods only."""
        robot, out, cmd = self.robot, self.output, args.command
        if cmd not in ('joints', 'get', 'help'):
            self._stop_joint_stream()
        if getattr(robot, 'is_multi_arm', False) and (
                cmd in ('pq', 'g_move')):
            raise CommandError('双臂模式拒绝单臂目标和夹爪指令；请使用 m_init 或完整控制组的 operator 接口。')
        if cmd in LOCAL_COMMANDS:
            result = LOCAL_COMMANDS[cmd].execute(robot, args, out)
            if cmd == 'stop':
                self.observe(result)
        elif cmd == 'help':
            out(HELP)
        elif cmd == 'exit':
            return False
        elif cmd == 'reconnect':
            self.reconnect()
        elif cmd == 'get':
            state = self.observe(robot.get_state())
            if args.raw:
                out(json.dumps(state.raw, ensure_ascii=False, indent=2))
            else:
                show_state(state, out)
        elif cmd in ('manual_en', 'manual_ds'):
            state = self.observe(robot.enable() if cmd == 'manual_en' else robot.disable())
            out('手动使能成功（反馈已确认）。' if cmd == 'manual_en'
                else '手动掉使能成功（反馈已确认）。')
            show_state(state, out)
        elif cmd == 'set_jog_coordinate':
            coordinate = 'Axis' if args.axis else 'Tool'
            state = self.observe(robot.set_jog_coordinate(coordinate))
            out(f'坐标系设置成功（{coordinate} 反馈已确认）。')
            show_state(state, out)
        elif cmd in ('follower_cart', 'follower_aj'):
            mode = 'joint' if cmd == 'follower_aj' else 'cartesian'
            state = self.observe(robot.start_follower(mode=mode))
            if getattr(robot, 'is_multi_arm', False):
                out('本次启用整个双臂控制组，不是独立启动某一臂。')
            out((('双臂关节' if getattr(robot, 'is_multi_arm', False) else '六轴关节')
                 if mode == 'joint' else '笛卡尔') +
                '跟随已启动（反馈已确认）；尚未发送 UDP 目标。')
            show_state(state, out)
        elif cmd == 'joints':
            state = self.observe(robot.get_state())
            if args.targets:
                order = robot.joint_target_order
                expected = sum(robot.joint_dofs[name] for name in order)
                if len(args.targets) != expected:
                    raise CommandError(f'当前配置需要 {expected} 个绝对角度，顺序：' + '、'.join(order))
                targets, offset = {}, 0
                for name in order:
                    count = robot.joint_dofs[name]
                    targets[name] = [math.radians(v) for v in args.targets[offset:offset + count]]
                    offset += count
                self._set_joint_target(targets)
                out(f'{expected}轴目标已更新，按 {self.joint_hz:g} Hz 持续发送（目标不变也重复发送）；'
                    '输入 stop 停止。尚未确认运动或到位。')
                return True
            if getattr(robot, 'is_multi_arm', False):
                for name, arm in state.arms.items():
                    values = ('暂无实际反馈' if arm.actual_joints_rad is None else
                              ' '.join(format(math.degrees(float(v)), '.17g') for v in arm.actual_joints_rad))
                    out(f'{name}（ee_id={arm.ee_id}）六轴实际角度（度）：{values}')
                order = robot.joint_target_order
                if all(state.arms[name].actual_joints_rad is not None for name in order):
                    values = ' '.join(format(math.degrees(float(v)), '.17g')
                                      for name in order for v in state.arms[name].actual_joints_rad)
                    out('可复制为目标（这里只显示，未发送）；顺序：' + '、'.join(order))
                    out('joints --deg ' + values)
                return True
            if state.actual_joints_rad is None:
                raise RuntimeError('没有六轴实际关节反馈，不能生成参考目标。')
            values = ' '.join(format(math.degrees(float(v)), '.17g')
                              for v in state.actual_joints_rad)
            out('六轴实际角度（度，关节1到6）：' + values)
            out('可复制为目标（这里只显示，未发送）：')
            out('joints --deg ' + values)
        elif cmd == 'stop_follower':
            state = self.observe(robot.stop_follower())
            out('跟随已停止（反馈已确认）。')
            show_state(state, out)
        elif cmd == 'pq':
            # Refresh permission before each manually issued target. This isn't
            # a background watchdog or a guarantee about the next instant.
            self.observe(robot.get_state())
            robot.send_eepose({robot.arm_name: args.pose})
            out('UDP 目标已发送；没有接收确认，尚未确认运动或到位。')
        elif cmd == 'jog':
            out('执行小幅移动：' + ' '.join(args.moves) + '；方向取各臂本次开始时的工具坐标系。')
            state = self.observe(robot.jog_tool(args.deltas))
            out('本次目标的双臂反馈已连续 3 次落入容差（1 mm / 0.02 rad）；follower 仍运行。')
            show_state(state, out)
        elif cmd == 'g_move':
            robot.send_gripper({robot.arm_name: args.pos})
            out('夹爪命令已收到成功回复；尚未确认夹爪到位。')
        elif cmd == 'controller_command':
            out(f'准备发送原生命令，最多等待 {robot.command_timeout_s:g} 秒回复；不会自动重发。')
            reply = robot.send_controller_command(args.text)
            out('控制器原始回复：' + json.dumps(reply, ensure_ascii=False, indent=2))
            if reply['ret_code'] != 0:
                raise RuntimeError(f"控制器拒绝指令，ret_code={reply['ret_code']}：{reply.get('ret_msg', '')}")
            out('控制器已回复；未确认动作完成。')
        return True

    def _run_commands(self):
        while True:
            job = self._commands.get()
            try:
                if job is None:
                    return
                generation, args = job
                with self._dispatch_lock:
                    if generation != self._generation:
                        continue  # A stop cancels queued work as well as active work.
                    self._worker_idle.clear()
                    self.robot.begin_console_command()
                try:
                    self.execute(args)
                except KeyboardInterrupt:
                    self._worker_interrupt.set()
                    self.robot.cancel_current_command()
                except InterruptedError:
                    self.output('当前命令已取消。')
                except BaseException as exc:
                    self.report_error(exc)
            finally:
                self._worker_idle.set()
                self._commands.task_done()

    def _stop_current_command(self):
        self.output('收到 stop，正在取消当前及排队命令……')
        with self._dispatch_lock:
            self._generation += 1
            self._joint_stop.set()
            self.robot.cancel_current_command()
        budget = max(5., 2 * self.robot.timeout_s + 1.)
        if not self._worker_idle.wait(budget):
            raise TimeoutError('已禁止后续动作下发，但当前命令尚未退出；请在现场确认机器人停止。')
        self.execute(parse_command(['stop']))

    def shell(self, input_fn=input):
        self.output('输入 help 查看命令；动作执行中可随时输入 stop 取消，控制台保持打开。')
        worker = threading.Thread(target=self._run_commands, name='a10-console-command', daemon=True)
        worker.start()
        result = 0
        try:
            while True:
                try:
                    if self._worker_interrupt.is_set():
                        raise KeyboardInterrupt
                    line = input_fn('a10> ')
                    if self._worker_interrupt.is_set():
                        raise KeyboardInterrupt
                    if not line.strip():
                        continue
                    tokens = shlex.split(line)
                    if tokens in (['--help'], ['-h']):
                        tokens = ['help']
                    args = parse_command(tokens, raw_text=line)
                    if args.command == 'exit':
                        break
                    if args.command == 'stop':
                        self._stop_current_command()
                    else:
                        with self._dispatch_lock:
                            self._commands.put((self._generation, args))
                except EOFError:
                    break
                except Exception as exc:
                    self.report_error(exc)
            self._commands.join()
        except KeyboardInterrupt:
            self.output('\n收到 Ctrl+C，正在取消命令并退出。')
            result = 130
            try:
                self._stop_current_command()
            except Exception as exc:
                self.report_error(exc)
        finally:
            self._commands.put(None)
            worker.join(timeout=max(5., 2 * self.robot.timeout_s + 1.))
            if worker.is_alive():
                self.output('命令线程未退出；本地取消已锁定，请在现场确认停止。')
                result = 1
            self._stop_joint_stream()
        return result

    def cleanup(self):
        """Only stop this session's attempted start. Never take over another one."""
        ok = True
        try:
            self._stop_joint_stream()
            if self.robot.follower_stop_pending:
                self.output('正在请求停止本次跟随……')
                if not self.robot.is_connected:
                    raise ConnectionError('连接已断开')
                self.observe(self.robot.stop_follower())
                self.output('本次跟随已停止（反馈已确认）。')
            if self.unresolved_follower:
                raise RuntimeError('旧会话的跟随仍未确认停止，新连接不能替旧会话自动停止')
        except (Exception, KeyboardInterrupt) as exc:
            ok = False
            self.output(f'未能确认跟随停止：{exc}。请在现场控制器确认并停止；异常运动时使用急停。')
        finally:
            try:
                self.robot.close()
                self.output('连接已关闭；关闭连接不代表机械臂停止，不会自动掉使能。')
            except (Exception, KeyboardInterrupt) as exc:
                ok = False
                self.output(f'连接清理未完成：{exc}')
        return ok


def main(argv=None, *, operator_factory=None, input_fn=input, output=print):
    parser = argparse.ArgumentParser(description='A10 真机通信控制台',
                                     epilog='例如：--config configs/a10/a10_dual_arm_operator.py（默认进入 shell 并允许控制）',
                                     allow_abbrev=False)
    parser.add_argument('--config', type=Path,
                        help='机器人配置；默认 configs/a10/a10_single_operator.py，双臂使用 a10_dual_arm_operator.py')
    parser.add_argument('--host', help='工控机 IP 或主机名；也可由 --config 提供')
    parser.add_argument('--ws-port', type=int)
    parser.add_argument('--udp-port', type=int)
    parser.add_argument('--joint-udp-port', type=int)
    parser.add_argument('--joint-hz', type=float, default=30.,
                        help='控制台最新关节目标持续发送频率，默认30 Hz')
    parser.add_argument('--timeout', type=float, help='命令确认超时秒数；默认 5')
    access = parser.add_mutually_exclusive_group()
    access.add_argument('--read-only', action='store_true', help='只读；禁止设置和控制命令')
    access.add_argument('--allow-motion', action='store_true', help='允许控制（交互控制台默认开启）；不自动使能')
    parser.add_argument('--command-timeout', type=float, default=None,
                        help='原生/透传控制器指令的回复等待秒数（默认：60；与 --timeout 独立）')
    connection_mode = parser.add_mutually_exclusive_group()
    connection_mode.add_argument('--shared', dest='shared', action='store_true',
                                 help='使用共享连接（不带配置时默认；控制台自动启动本机服务）')
    connection_mode.add_argument('--direct', dest='shared', action='store_false',
                                 help='独占直连机器人，不启动共享服务')
    parser.set_defaults(shared=None)
    parser.add_argument('--status-ws-port', type=int)
    parser.add_argument('--shared-host', help='共享服务地址；默认127.0.0.1或配置值')
    parser.add_argument('--shared-port', type=int, help='共享服务端口；默认5870或配置值')
    parser.add_argument('command', nargs=argparse.REMAINDER, help='shell 或单次指令；默认 shell')
    args = parser.parse_args(argv)
    tokens = args.command or ['shell']
    interactive = tokens == ['shell']
    try:
        command = None if interactive else parse_command(tokens)
        if command and (command.command in SESSION_COMMANDS or
                        (command.command == 'joints' and command.targets)):
            raise CommandError('跟随启停、目标发送和 reconnect 请在 shell 中执行。')
    except CommandError as exc:
        output(f'指令格式错误：{exc}；使用 --host IP help 查看帮助。')
        return 2
    if command and command.command in ('help', 'exit'):
        if command.command == 'help':
            output(HELP)
        return 0

    console = None
    shared_service = None
    result = 0
    try:
        if operator_factory is None:
            from fluxvla.engines.operators.a10_operator import A10Operator
            operator_factory = A10Operator
        from mmengine import Config
        config_path = args.config or Path(__file__).resolve().parents[1] / 'configs/a10/a10_single_operator.py'
        settings = dict(Config.fromfile(str(config_path))['operator'])
        if settings.pop('type', 'A10Operator') != 'A10Operator':
            raise CommandError('控制台配置必须使用 A10Operator')
        overrides = dict(host=args.host, ws_port=args.ws_port, udp_port=args.udp_port,
                         joint_udp_port=args.joint_udp_port, timeout_s=args.timeout,
                         command_timeout_s=args.command_timeout, status_ws_port=args.status_ws_port)
        settings.update({key: value for key, value in overrides.items() if value is not None})
        if interactive or args.allow_motion:
            settings['allow_motion'] = True
        shared = args.shared if args.shared is not None else (
            args.config is None or bool(settings.get('shared_control')))
        if args.read_only:
            settings['allow_motion'] = False
            # Monitor-only, with no accessory opens or extra background socket.
            if not shared and args.config is not None and args.ws_port is None:
                settings['ws_port'] = settings.get('status_ws_port', settings.get('ws_port', 5999))
            settings.pop('status_poll_interval_s', None)
            for key in ('cameras', 'gripper', 'gripper_feedback_path'):
                settings.pop(key, None)
            settings['gripper_transport'] = 'controller'
        if shared:
            from fluxvla.engines.communication.a10_shared_server import ConsoleSharedControl
            shared_options = dict(settings.get('shared_control') or {})
            shared_options.update(role='console')
            if args.shared_host is not None:
                shared_options['host'] = args.shared_host
            if args.shared_port is not None:
                shared_options['port'] = args.shared_port
            shared_options.setdefault('host', '127.0.0.1')
            shared_options.setdefault('port', 5870)
            if shared_options.get('mode') != 'managed':
                shared_service = ConsoleSharedControl(
                    (shared_options['host'], shared_options['port']), output=output,
                    host=settings['host'], ws_port=settings.get('ws_port', 5999),
                    status_ws_port=settings.get('status_ws_port', 5888),
                    udp_port=settings.get('udp_port', 9998),
                    joint_udp_port=settings.get('joint_udp_port', 9999),
                    timeout_s=settings.get('timeout_s', 5.))
            shared_options.setdefault('status_ws_port', settings.get('status_ws_port', 5888))
            settings['shared_control'] = shared_options
        else:
            settings.pop('shared_control', None)
        output(f'使用配置：{config_path.resolve()}')
        arm_count = len(settings.get('arms') or {settings.get('arm_name', 'right'): {}})
        output(f"目标机器人：{settings['host']}:{settings.get('ws_port', 5999)}"
               f"（{'双臂' if arm_count > 1 else '单臂'}；{'共享连接' if shared else '直连'}；"
               f"{'允许控制' if settings.get('allow_motion', False) else '只读'}）")
        robot = operator_factory(**settings)
        console = A10Console(robot, output,
                             prepare_connection=shared_service.start if shared_service else None,
                             joint_hz=args.joint_hz)
        if interactive:
            try:
                console.reconnect()
            except Exception as exc:
                console.report_error(exc)
            result = console.shell(input_fn)
        else:
            if shared_service is not None:
                shared_service.start()
            robot.connect()
            console.execute(command)
    except KeyboardInterrupt:
        output('操作被中断，正在清理连接。')
        result = 130
    except Exception as exc:
        output(f'操作未完成：{exc}')
        result = 1
    finally:
        if console is not None and not console.cleanup():
            result = 1
        if shared_service is not None:
            try:
                shared_service.close()
            except (Exception, KeyboardInterrupt) as exc:
                output(f'共享连接清理未完成：{exc}')
                result = 1
    return result


if __name__ == '__main__':
    sys.exit(main())
