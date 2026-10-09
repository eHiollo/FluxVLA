#!/usr/bin/env python3
"""A10 terminal commands. Robot protocol and control logic live in A10Operator."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shlex
import sys

# Prefer this checkout, including when another branch is installed editable.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


HELP = """可用指令：
  get [--raw]                   查看状态；--raw 显示完整回复
  manual_en                     手动使能，回读 Enabled
  manual_ds                     手动掉使能，回读 Disabled（不是急停）
  set_jog_coordinate --tool     切换工具坐标系并回读
  follower_cart                 启动本连接的笛卡尔跟随（仅 shell）
  stop_follower                 停止本连接启动的跟随（仅 shell）
  pq x y z qx qy qz qw          发送一个 UDP 目标（仅 shell）
  g_move --pos=毫米数            发送夹爪目标，范围 0..100 mm
  help                          显示帮助
  exit                          停止本次跟随后关闭连接，不自动掉使能

pq 的 xyz 单位是米，四元数顺序是 xyzw，须为单位四元数。
目标相对于跟随启动基准，不是绝对位置，也不是逐次累加的移动量。
UDP 发出不等于控制器收到或机器人到位；坐标语义需要现场核对。
Ctrl+C / Ctrl+D 与 exit 一样尝试退出；断网或强制结束无法保证停止。
清错仍使用现场控制器界面，本控制台不透传任意控制器命令。
"""
SESSION_COMMANDS = {'follower_cart', 'stop_follower', 'pq'}


class CommandError(ValueError):
    pass


class CommandParser(argparse.ArgumentParser):
    def error(self, message):
        raise CommandError(message)


def parse_command(tokens):
    parser = CommandParser(prog='a10', add_help=False, allow_abbrev=False)
    subs = parser.add_subparsers(dest='command', required=True,
                                 parser_class=CommandParser)
    get = subs.add_parser('get', add_help=False, allow_abbrev=False)
    get.add_argument('--raw', action='store_true')
    for name in ('manual_en', 'manual_ds', 'follower_cart', 'stop_follower', 'exit', 'help'):
        subs.add_parser(name, add_help=False, allow_abbrev=False)
    coord = subs.add_parser('set_jog_coordinate', add_help=False, allow_abbrev=False)
    coord.add_argument('--tool', action='store_true', required=True)
    pose = subs.add_parser('pq', add_help=False, allow_abbrev=False)
    pose.add_argument('pose', nargs=7, type=float)
    gripper = subs.add_parser('g_move', add_help=False, allow_abbrev=False)
    gripper.add_argument('--pos', type=float, required=True)
    return parser.parse_args(tokens)


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
    output(f'实际夹爪（毫米）：{state.gripper_mm if state.gripper_mm is not None else "暂无反馈"}')


class A10Console:
    def __init__(self, robot, output=print):
        self.robot, self.output = robot, output

    def execute(self, args):
        """Execute parsed commands via public operator methods only."""
        robot, out, cmd = self.robot, self.output, args.command
        if cmd == 'help':
            out(HELP)
        elif cmd == 'exit':
            return False
        elif cmd == 'get':
            state = robot.get_state()
            if args.raw:
                out(json.dumps(state.raw, ensure_ascii=False, indent=2))
            else:
                show_state(state, out)
        elif cmd in ('manual_en', 'manual_ds'):
            state = robot.enable() if cmd == 'manual_en' else robot.disable()
            out('手动使能成功（反馈已确认）。' if cmd == 'manual_en'
                else '手动掉使能成功（反馈已确认）。')
            show_state(state, out)
        elif cmd == 'set_jog_coordinate':
            state = robot.set_tool_coordinate()
            out('工具坐标系设置成功（Tool 反馈已确认）。')
            show_state(state, out)
        elif cmd == 'follower_cart':
            state = robot.start_follower()
            out('笛卡尔跟随已启动（反馈已确认）；尚未发送 UDP 目标。')
            show_state(state, out)
        elif cmd == 'stop_follower':
            state = robot.stop_follower()
            out('跟随已停止（反馈已确认）。')
            show_state(state, out)
        elif cmd == 'pq':
            # Refresh permission before each manually issued target. This isn't
            # a background watchdog or a guarantee about the next instant.
            robot.get_state()
            robot.send_eepose({robot.arm_name: args.pose})
            out('UDP 目标已发送；没有接收确认，尚未确认运动或到位。')
        elif cmd == 'g_move':
            robot.send_gripper({robot.arm_name: args.pos})
            out('夹爪命令已收到成功回复；尚未确认夹爪到位。')
        return True

    def shell(self, input_fn=input):
        self.output('输入 help 查看命令。此控制台可以发送控制指令；连接本身不会使能或启动跟随。')
        while True:
            try:
                line = input_fn('a10> ').strip()
                if not line:
                    continue
                tokens = shlex.split(line)
                if tokens in (['--help'], ['-h']):
                    tokens = ['help']
                if not self.execute(parse_command(tokens)):
                    return 0
            except EOFError:
                return 0
            except KeyboardInterrupt:
                self.output('\n收到 Ctrl+C，正在退出。')
                return 130
            except Exception as exc:
                self.output(f'操作未完成：{exc}')
                if not self.robot.is_connected:
                    self.output('连接已断开，结束本次会话；不会自动重发控制指令。')
                    return 1

    def cleanup(self):
        """Only stop this session's attempted start. Never take over another one."""
        ok = True
        try:
            if self.robot.follower_stop_pending:
                self.output('正在请求停止本次跟随……')
                if not self.robot.is_connected:
                    raise ConnectionError('连接已断开')
                self.robot.stop_follower()
                self.output('本次跟随已停止（反馈已确认）。')
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
                                     epilog='例如：--host 192.168.110.124 shell',
                                     allow_abbrev=False)
    parser.add_argument('--host', required=True, help='工控机 IP 或主机名')
    parser.add_argument('--ws-port', type=int, default=5999)
    parser.add_argument('--udp-port', type=int, default=9998)
    parser.add_argument('--timeout', type=float, default=5.0, help='命令确认超时秒数')
    parser.add_argument('--read-only', action='store_true', help='只读；禁止设置和控制命令')
    parser.add_argument('command', nargs=argparse.REMAINDER, help='shell 或单次指令；默认 shell')
    args = parser.parse_args(argv)
    tokens = args.command or ['shell']
    interactive = tokens == ['shell']
    try:
        command = None if interactive else parse_command(tokens)
        if command and command.command in SESSION_COMMANDS:
            raise CommandError('跟随启停与 pq 请在 shell 中执行，以保持同一连接。')
    except CommandError as exc:
        output(f'指令格式错误：{exc}；使用 --host IP help 查看帮助。')
        return 2
    if command and command.command in ('help', 'exit'):
        if command.command == 'help':
            output(HELP)
        return 0

    console = None
    result = 0
    try:
        if operator_factory is None:
            from fluxvla.engines.operators.a10_operator import A10Operator
            operator_factory = A10Operator
        robot = operator_factory(host=args.host, ws_port=args.ws_port, udp_port=args.udp_port,
                                 timeout_s=args.timeout, allow_motion=not args.read_only)
        console = A10Console(robot, output)
        robot.connect()
        if interactive:
            state = robot.get_state()
            output(f'已连接 A10：{args.host}（状态读取成功）')
            show_state(state, output)
            result = console.shell(input_fn)
        else:
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
    return result


if __name__ == '__main__':
    sys.exit(main())
