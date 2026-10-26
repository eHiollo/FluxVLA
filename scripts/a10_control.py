#!/usr/bin/env python3
"""A10 terminal commands. Robot protocol and control logic live in A10Operator."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import shlex
import sys

# Prefer this checkout, including when another branch is installed editable.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


HELP = """可用指令：
  reconnect                     重新连接并查询状态，不重发控制指令（仅 shell）
  启动控制台                    python scripts/a10_control.py shell（自动启动共享连接）
  get [--raw]                   查看状态；--raw 显示完整回复
  manual_en                     手动使能，回读 Enabled
  manual_ds                     手动掉使能，回读 Disabled（不是急停）
  set_jog_coordinate --tool     切换工具坐标系并回读
  set_jog_coordinate --axis     切换关节坐标系并回读
  follower_cart                 启动本连接的笛卡尔跟随（仅 shell）
  follower_aj                   启动本连接的六轴关节跟随（仅 shell）
  joints                        查询六轴实际角度（度），只读取不发送目标
  joints --deg q1 q2 q3 q4 q5 q6 发送六轴绝对角度目标（仅 shell）
  stop_follower                 停止跟随（共享模式也可停止推理；仅 shell）
  pq x y z qx qy qz qw          发送一个 UDP 目标（仅 shell）
  jog left x 5                  左臂沿本次开始时工具 X 轴移动 5 mm（双臂 shell）
  jog right rz 1                右臂绕本次开始时工具 Z 轴旋转 1 度
  jog left x 5 right x 5        两臂各沿自身工具 X 轴移动 5 mm
  g_move --pos=毫米数            发送夹爪目标，范围 0..100 mm
  其他指令（如 m_init）          原文发送到控制器，显示完整回复
  raw 控制器指令                显式透传，可发送与内置名称相同的原生命令
  help                          显示帮助
  exit                          停止本次跟随后关闭连接，不自动掉使能

pq 的 xyz 单位是米，四元数顺序是 xyzw，须为单位四元数。
目标相对于跟随启动基准，不是绝对位置，也不是逐次累加的移动量。
关节目标是六个绝对角度，单位度，顺序为关节1到6；不包含夹爪。
关节跟随与末端跟随不能混发；切换前先 stop_follower。
UDP 发出不等于控制器收到或机器人到位；坐标语义需要现场核对。
双臂配置：get/joints 显示各臂；follower_cart/stop_follower 作用于整个控制组。
双臂 jog：轴 x/y/z 单位毫米，rx/ry/rz 单位度；单次最多 10 mm / 5 度。
先确认 ee_id 的物理左右对应，再 manual_en、set_jog_coordinate --tool、follower_cart、jog。
未选择的臂保持；连续 jog 保留累计量；到位检查失败会请求停止整个控制组。
双臂 pq、joints --deg、g_move、follower_aj 仍拒绝；jog 与原始 pq 不能混用。
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
    if tokens and tokens[0] not in BUILTIN_COMMANDS:
        return argparse.Namespace(command='controller_command',
                                  text=raw_text if raw_text is not None else shlex.join(tokens))
    parser = CommandParser(prog='a10', add_help=False, allow_abbrev=False)
    subs = parser.add_subparsers(dest='command', required=True,
                                 parser_class=CommandParser)
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
        if not args.deg or len(args.targets) != 6:
            raise CommandError('发送关节目标请使用 joints --deg 加六个绝对角度；查询只输入 joints。')
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
    def __init__(self, robot, output=print, prepare_connection=None):
        self.robot, self.output = robot, output
        self.prepare_connection = prepare_connection
        self.unresolved_follower = False

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
        """Execute parsed commands via public operator methods only."""
        robot, out, cmd = self.robot, self.output, args.command
        if getattr(robot, 'is_multi_arm', False) and (
                cmd in ('pq', 'g_move', 'follower_aj') or (cmd == 'joints' and args.targets)):
            raise CommandError('双臂模式拒绝单臂目标、夹爪和关节跟随指令；请使用完整控制组的 operator 笛卡尔接口。')
        if cmd == 'help':
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
            out(('六轴关节' if mode == 'joint' else '笛卡尔') +
                '跟随已启动（反馈已确认）；尚未发送 UDP 目标。')
            show_state(state, out)
        elif cmd == 'joints':
            state = self.observe(robot.get_state())
            if getattr(robot, 'is_multi_arm', False):
                for name, arm in state.arms.items():
                    values = ('暂无实际反馈' if arm.actual_joints_rad is None else
                              ' '.join(format(math.degrees(float(v)), '.17g') for v in arm.actual_joints_rad))
                    out(f'{name}（ee_id={arm.ee_id}）六轴实际角度（度）：{values}')
                return True
            if args.targets:
                robot.send_joints({robot.arm_name: [math.radians(v) for v in args.targets]})
                out('六轴绝对角度目标已通过 UDP 发送；没有接收确认，尚未确认运动或到位。')
            else:
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

    def shell(self, input_fn=input):
        self.output('输入 help 查看命令。此控制台可以发送控制指令；连接本身不会使能或启动跟随。')
        while True:
            try:
                line = input_fn('a10> ')
                if not line.strip():
                    continue
                tokens = shlex.split(line)
                if tokens in (['--help'], ['-h']):
                    tokens = ['help']
                if not self.execute(parse_command(tokens, raw_text=line)):
                    return 0
            except EOFError:
                return 0
            except KeyboardInterrupt:
                self.output('\n收到 Ctrl+C，正在退出。')
                return 130
            except Exception as exc:
                self.report_error(exc)

    def cleanup(self):
        """Only stop this session's attempted start. Never take over another one."""
        ok = True
        try:
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
                                     epilog='例如：shell（默认机器人 192.168.0.125）',
                                     allow_abbrev=False)
    parser.add_argument('--config', type=Path, help='包含 operator 字典的 Python 配置文件')
    parser.add_argument('--host', help='工控机 IP 或主机名；也可由 --config 提供')
    parser.add_argument('--ws-port', type=int)
    parser.add_argument('--udp-port', type=int)
    parser.add_argument('--joint-udp-port', type=int)
    parser.add_argument('--timeout', type=float, help='命令确认超时秒数；默认 5')
    access = parser.add_mutually_exclusive_group()
    access.add_argument('--read-only', action='store_true', help='只读；禁止设置和控制命令')
    access.add_argument('--allow-motion', action='store_true', help='显式开启配置中的控制权限；不自动使能')
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
        if args.config is not None:
            from mmengine import Config
            settings = dict(Config.fromfile(str(args.config))['operator'])
            if settings.pop('type', 'A10Operator') != 'A10Operator':
                raise CommandError('控制台配置必须使用 A10Operator')
        else:
            settings = dict(host='192.168.0.125', ws_port=5999, udp_port=9998,
                            joint_udp_port=9999, status_ws_port=5888, timeout_s=5., allow_motion=True)
        overrides = dict(host=args.host, ws_port=args.ws_port, udp_port=args.udp_port,
                         joint_udp_port=args.joint_udp_port, timeout_s=args.timeout,
                         command_timeout_s=args.command_timeout, status_ws_port=args.status_ws_port)
        settings.update({key: value for key, value in overrides.items() if value is not None})
        if args.allow_motion:
            settings['allow_motion'] = True
        shared = args.shared if args.shared is not None else (
            args.config is None or bool(settings.get('shared_control')))
        if shared and len(settings.get('arms') or {}) > 1:
            raise CommandError('双臂控制台目前使用 --direct；共享服务尚未验证双臂反馈。')
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
            shared_service = ConsoleSharedControl(
                (shared_options['host'], shared_options['port']), output=output,
                host=settings['host'], ws_port=settings.get('ws_port', 5999),
                status_ws_port=settings.get('status_ws_port', 5888),
                udp_port=settings.get('udp_port', 9998),
                joint_udp_port=settings.get('joint_udp_port', 9999),
                timeout_s=settings.get('timeout_s', 5.))
            settings['shared_control'] = shared_options
        else:
            settings.pop('shared_control', None)
        robot = operator_factory(**settings)
        console = A10Console(robot, output,
                             prepare_connection=shared_service.start if shared_service else None)
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
