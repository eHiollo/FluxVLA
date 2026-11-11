"""CLI regressions against the independent loopback follower server."""

import contextlib
import io
import json
from pathlib import Path
import socket
import subprocess
import tempfile
import sys
import time
import unittest
from unittest.mock import patch

from scripts.a10_control import main as control_main, parse_command, CommandError, A10Console
from test_a10_operator import FakeController, DUAL_ARMS, dual_motion


def main(argv=None, **kwargs):
    # These protocol scenarios intentionally submit the next input after the
    # previous reply. Production input is asynchronous; concurrent takeover and
    # cancellation are exercised separately, without this deterministic driver.
    class OrderedInputConsole(A10Console):
        def shell(self, input_fn=input):
            def read(prompt):
                self._commands.join()
                return input_fn(prompt)
            return super().shell(read)
    with patch('scripts.a10_control.A10Console', OrderedInputConsole):
        return control_main(['--direct', *(argv or [])], **kwargs)


class ConsoleTest(unittest.TestCase):
    def test_script_entrypoint_from_another_directory(self):
        script = Path(__file__).resolve().parents[1] / 'scripts/a10_control.py'
        with FakeController() as c:
            c.activation = 'Disabled'
            result = subprocess.run([sys.executable, str(script), '--direct', '--host', '127.0.0.1',
                '--ws-port', str(c.server.socket.getsockname()[1]),
                '--udp-port', str(c.udp.getsockname()[1]),
                '--joint-udp-port', str(c.joint_udp.getsockname()[1]), '--timeout', '1', 'shell'],
                input='manual_en\nfollower_aj\njoints --deg 0 90 -90 180 0 45\nstop_follower\nfollower_cart\nstop_follower\nmanual_ds\nexit\n',
                text=True, capture_output=True, cwd='/tmp', timeout=60)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertNotIn('操作未完成', result.stdout)
            self.assertIn('手动使能成功', result.stdout)
            self.assertEqual([cmd for cmd in c.commands if cmd != 'get'],
                ['manual_en', 'start_follower', 'follower_aj', 'stop_follower', 'follower_cart', 'stop_follower', 'manual_ds'])
            packet,_ = c.joint_udp.recvfrom(65535)
            self.assertEqual(packet[40:], b'jp --motor_pos=[0,90,-90,180,0,45]')

    def run_console(self, controller, commands, extra=()):
        output = []
        lines = iter(commands)
        def read(_prompt):
            try:
                value = next(lines)
            except StopIteration:
                raise EOFError
            if isinstance(value, BaseException):
                raise value
            return value
        def factory(**options):
            # main supplies loopback ports; this factory only tracks cleanup.
            return controller.operator(**options)
        result = main(['--host', '127.0.0.1',
                       '--ws-port', str(controller.server.socket.getsockname()[1]),
                       '--udp-port', str(controller.udp.getsockname()[1]),
                       '--joint-udp-port', str(controller.joint_udp.getsockname()[1]),
                       '--timeout', '.15', *extra, 'shell'],
                      operator_factory=factory, input_fn=read, output=output.append)
        return result, '\n'.join(output)

    def test_joint_session_query_and_absolute_target(self):
        with FakeController() as c:
            code, output = self.run_console(c, ['joints','set_jog_coordinate --axis',
                'follower_aj','joints --deg 0 90 -90 180 0 45','stop_follower','exit'])
            self.assertEqual(code, 0)
            self.assertNotIn('操作未完成', output)
            self.assertIn('六轴实际角度（度', output)
            self.assertIn('六轴关节跟随已启动', output)
            self.assertIn('尚未确认运动或到位', output)
            packet,_ = c.joint_udp.recvfrom(65535)
            self.assertEqual(packet[4:40],bytes(36))
            self.assertEqual(int.from_bytes(packet[:4],'little'),len(packet)-40)
            self.assertEqual(packet[40:],b'jp --motor_pos=[0,90,-90,180,0,45]')
            self.assertEqual([cmd for cmd in c.commands if cmd!='get'],
                             ['set_jog_coordinate --axis','start_follower','follower_aj','stop_follower'])
            self.assertIsNone(c.operators[0]._connection.joint_udp)

    def test_joint_send_before_start_or_wrong_mode_refused(self):
        with FakeController() as c:
            code, output = self.run_console(c, ['joints --deg 0 0 0 0 0 0',
                'follower_cart','joints --deg 0 0 0 0 0 0','stop_follower',
                'follower_aj','pq 0 0 0 0 0 0 1','exit'])
            self.assertEqual(code, 0)
            self.assertIn('Axis follower start has not been confirmed', output)
            self.assertIn('Cart follower start has not been confirmed', output)
            with self.assertRaises(socket.timeout):
                c.joint_udp.recvfrom(65535)
            self.assertEqual(c.commands.count('stop_follower'),2)

    def test_joint_bad_command_shapes_do_not_send_targets(self):
        with FakeController() as c:
            commands = ['joints --deg','joints --deg 1 2 3 4 5',
                'joints --deg 1 2 3 4 5 6 7','joints 1 2 3 4 5 6',
                'joints --deg nan 0 0 0 0 0','joints --deg inf 0 0 0 0 0',
                'set_jog_coordinate --axis --tool', 'help', 'exit']
            code,output = self.run_console(c, commands)
            self.assertEqual(code,0)
            self.assertIn('操作未完成',output)
            self.assertTrue(all(cmd=='get' for cmd in c.commands))
            with self.assertRaises(socket.timeout):
                c.joint_udp.recvfrom(65535)

    def test_joint_exit_eof_and_interrupt_stop_this_session(self):
        for end,expected in ((['exit'],0),([],0),([KeyboardInterrupt()],130)):
            with self.subTest(end=end),FakeController() as c:
                code,output = self.run_console(c,['follower_aj',*end])
                self.assertEqual(code,expected)
                self.assertFalse(c.active)
                self.assertIn('Stopped 反馈已确认' if expected == 130 else '本次跟随已停止',output)

    def test_joint_query_does_not_substitute_target_feedback(self):
        with FakeController() as c:
            c.actual=False
            code,output=self.run_console(c,['joints','exit'])
            self.assertEqual(code,0)
            self.assertIn('没有六轴实际关节反馈',output)
            self.assertNotIn('joints --deg 1',output)
            self.assertTrue(all(cmd=='get' for cmd in c.commands))

    def test_joint_reconnect_does_not_inherit_old_start_or_send(self):
        with FakeController() as c:
            steps=iter(['follower_aj','get','reconnect','joints --deg 0 0 0 0 0 0','exit'])
            def read(_):
                line=next(steps)
                if line=='get':c.fault='disconnect'
                if line=='reconnect':c.fault=None
                return line
            out=[]
            code=main(['--host','127.0.0.1','shell'],operator_factory=lambda **_:c.operator(),
                      input_fn=read,output=out.append)
            self.assertEqual(code,1)
            self.assertIn('旧会话的跟随', '\n'.join(out))
            self.assertEqual(c.commands.count('follower_aj'),1)
            self.assertNotIn('stop_follower',c.commands)
            with self.assertRaises(socket.timeout):c.joint_udp.recvfrom(65535)

    def test_one_shot_joint_query_is_readonly_and_target_requires_shell(self):
        with FakeController() as c:
            out=[]
            self.assertEqual(main(['--host','127.0.0.1','joints'],
                operator_factory=lambda **_:c.operator(), output=out.append),0)
            self.assertEqual(c.commands,['get'])
            self.assertIn('可复制为目标','\n'.join(out))
        with patch('socket.socket',side_effect=AssertionError('unexpected network')):
            for cmd in (['follower_aj'],['joints','--deg','0','0','0','0','0','0']):
                self.assertEqual(main(['--host','example',*cmd],output=lambda _:None),2)

    def test_full_session_ws_udp_and_explicit_stop(self):
        with FakeController() as c:
            c.activation, c.coordinate = 'Disabled', 'Axis'
            code, output = self.run_console(c, ['get', 'set_jog_coordinate --tool',
                'manual_en', 'follower_cart', 'pq 0.001 0 0 0 0 0 1',
                'g_move --pos=25', 'stop_follower', 'manual_ds', 'exit'])
            self.assertEqual(code, 0)
            self.assertIn('没有接收确认', output)
            self.assertIn('尚未确认夹爪到位', output)
            packet, _ = c.udp.recvfrom(65535)
            self.assertEqual(packet[4:40], bytes(36))
            self.assertEqual(len(packet)-40, int.from_bytes(packet[:4], 'little'))
            self.assertEqual(json.loads(packet[40:]), {'type':'pq','pq':[[.001,0,0,0,0,0,1]]})
            controls = [cmd for cmd in c.commands if cmd != 'get']
            self.assertEqual(controls, ['set_jog_coordinate --tool', 'manual_en',
                'follower_cart', 'g_move --pos=25', 'stop_follower', 'manual_ds'])
            self.assertTrue(all(not op.is_connected for op in c.operators))

    def test_exit_eof_and_interrupt_stop_only_own_follower(self):
        for end, expected in ((['exit'], 0), ([], 0), ([KeyboardInterrupt()], 130)):
            with self.subTest(end=end), FakeController() as c:
                code, output = self.run_console(c, ['follower_cart', *end])
                self.assertEqual(code, expected)
                self.assertEqual(c.commands.count('stop_follower'), 1)
                self.assertIn('Stopped 反馈已确认' if expected == 130 else '本次跟随已停止', output)
                self.assertNotIn('manual_ds', c.commands)

    def test_exit_does_not_stop_existing_follower(self):
        with FakeController(initial_active=True) as c:
            code, output = self.run_console(c, ['follower_cart', 'exit'])
            self.assertEqual(code, 0)
            self.assertIn('refusing takeover', output)
            self.assertTrue(all(cmd == 'get' for cmd in c.commands))
            self.assertTrue(c.active)

    def test_interrupt_while_start_is_sent_still_attempts_cleanup(self):
        with FakeController() as c:
            operator = c.operator()
            send = operator._connection.send_command
            def interrupted_send(command):
                send(command)
                if command == 'follower_cart':
                    raise KeyboardInterrupt
            with patch.object(operator._connection, 'send_command', interrupted_send):
                code = main(['--host','127.0.0.1','shell'],
                            operator_factory=lambda **_: operator,
                            input_fn=lambda _: 'follower_cart', output=lambda _:None)
            self.assertEqual(code, 130)
            self.assertEqual(c.commands.count('stop_follower'), 1)
            self.assertFalse(c.active)

    def test_read_only_rejects_all_control(self):
        with FakeController() as c:
            code, output = self.run_console(c, ['manual_en','manual_ds',
                'set_jog_coordinate --tool','set_jog_coordinate --axis','follower_cart','follower_aj',
                'joints','joints --deg 0 0 0 0 0 0','stop_follower',
                'pq 0 0 0 0 0 0 1','g_move --pos=20','exit'], ['--read-only'])
            self.assertEqual(code, 0)
            self.assertTrue(all(cmd == 'get' for cmd in c.commands))
            self.assertIn('allow_motion is false', output)
            with self.assertRaises(socket.timeout):
                c.udp.recvfrom(65535)

    def test_malformed_commands_keep_shell_usable_and_send_no_controls(self):
        with FakeController() as c:
            code, output = self.run_console(c, ['"unterminated',
                'manual_en --typo', 'set_jog_coordinate --base',
                'pq 0 0', 'pq nan 0 0 0 0 0 1', 'g_move --pos=101',
                'get --raw', 'help', 'exit'])
            self.assertEqual(code, 0)
            self.assertIn('ret_context', output)
            self.assertIn('可用指令', output)
            self.assertTrue(all(cmd == 'get' for cmd in c.commands))

    def test_cleanup_failure_is_reported_and_nonzero(self):
        with FakeController() as c:
            steps = iter(['follower_cart', 'exit'])
            output = []
            def read(_):
                line = next(steps)
                if line == 'exit':
                    c.change_mode = False
                return line
            code = main(['--host','127.0.0.1','shell'],
                        operator_factory=lambda **_: c.operator(timeout_s=.05),
                        input_fn=read, output=output.append)
            self.assertEqual(code, 1)
            self.assertIn('未能确认跟随停止', '\n'.join(output))
            self.assertNotIn('本次跟随已停止', '\n'.join(output))
            self.assertFalse(c.operators[0].is_connected)

    def test_disconnect_with_pending_start_does_not_retry_or_claim_stop(self):
        with FakeController() as c:
            steps = iter(['follower_cart', 'get'])
            output = []
            def read(_):
                line = next(steps, 'exit')
                if line == 'get':
                    c.fault = 'disconnect'
                return line
            code = main(['--host','127.0.0.1','shell'],
                        operator_factory=lambda **_: c.operator(timeout_s=.1),
                        input_fn=read, output=output.append)
            self.assertEqual(code, 1)
            self.assertEqual(c.commands.count('follower_cart'), 1)
            self.assertNotIn('stop_follower', c.commands)
            self.assertIn('未能确认跟随停止', '\n'.join(output))

    def test_disconnect_keeps_prompt_and_explicit_reconnect_does_not_replay(self):
        with FakeController() as c:
            lines = iter(['get', 'manual_ds', 'help', 'reconnect', 'get', 'exit'])
            output = []
            reads = 0
            def read(_):
                nonlocal reads
                reads += 1
                line = next(lines)
                if reads == 1:
                    c.fault = 'disconnect'
                if line == 'reconnect':
                    c.fault = None
                return line
            code = main(['--host','127.0.0.1','shell'],
                        operator_factory=lambda **_: c.operator(), input_fn=read, output=output.append)
            self.assertEqual(code, 0)
            self.assertEqual(reads, 6)
            self.assertIn('控制台保留', '\n'.join(output))
            self.assertTrue(all(cmd == 'get' for cmd in c.commands))

    def test_initial_connection_failure_stays_in_shell_and_can_recover(self):
        with FakeController() as c:
            robot = c.operator()
            robot.close()
            connect = robot.connect
            calls = 0
            def unavailable_then_ready():
                nonlocal calls
                calls += 1
                if calls <= 2:
                    raise ConnectionRefusedError('controller offline')
                connect()
            lines = iter(['help', 'reconnect', 'reconnect', 'get', 'exit'])
            out = []
            with patch.object(robot, 'connect', unavailable_then_ready):
                code = main(['--host','127.0.0.1','shell'], operator_factory=lambda **_: robot,
                            input_fn=lambda _: next(lines), output=out.append)
            self.assertEqual(code, 0)
            self.assertEqual(calls, 3)
            self.assertIn('可用指令', '\n'.join(out))
            self.assertTrue(all(cmd == 'get' for cmd in c.commands))

    def test_reconnect_never_inherits_active_follower_and_remembers_uncertainty(self):
        for still_active in (True, False):
            with self.subTest(still_active=still_active), FakeController() as c:
                lines = iter(['follower_cart','get','reconnect','pq 0 0 0 0 0 0 1','exit'])
                reads = 0
                output = []
                def read(_):
                    nonlocal reads
                    reads += 1
                    line = next(lines)
                    if reads == 2:
                        c.fault = 'disconnect'
                    if line == 'reconnect':
                        c.fault = None
                        c.active = still_active
                    return line
                code = main(['--host','127.0.0.1','shell'],
                            operator_factory=lambda **_: c.operator(), input_fn=read, output=output.append)
                self.assertEqual(code, 1 if still_active else 0)
                self.assertEqual(c.commands.count('follower_cart'), 1)
                self.assertNotIn('stop_follower', c.commands)
                with self.assertRaises(socket.timeout):
                    c.udp.recvfrom(65535)
                if still_active:
                    self.assertIn('旧会话的跟随仍未确认停止', '\n'.join(output))

    def test_reconnect_preserves_live_session_stop_ownership(self):
        with FakeController() as c:
            code, output = self.run_console(c, ['follower_cart','reconnect','stop_follower','exit'])
            self.assertEqual(code, 0)
            self.assertIn('连接已保留', output)
            self.assertEqual(c.commands.count('stop_follower'), 1)

    def test_one_shot_and_help_have_expected_connection_scope(self):
        with FakeController() as c:
            out=[]
            code=main(['--host','127.0.0.1','get','--raw'],
                      operator_factory=lambda **_: c.operator(), output=out.append)
            self.assertEqual(code, 0)
            self.assertIn('ret_context', '\n'.join(out))
            self.assertFalse(c.operators[0].is_connected)
        with patch('socket.socket', side_effect=AssertionError('unexpected network')):
            for cmd in (['follower_cart'], ['stop_follower'], ['pq','0','0','0','0','0','0','1']):
                self.assertEqual(main(['--host','example',*cmd], output=lambda _:None), 2)
            self.assertEqual(main(['--host','example','help'], output=lambda _:None), 0)
            with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(SystemExit) as exc:
                main(['--help'])
            self.assertEqual(exc.exception.code, 0)

    def test_parse_exact_controller_spelling(self):
        self.assertEqual(parse_command(['g_move','--pos=12.5']).pos, 12.5)
        self.assertEqual(parse_command(['pq','-1e-3','0','0','0','0','0','1']).pose[0], -.001)
        self.assertEqual(parse_command(['joints','--deg','-1e-3','0','0','0','0','0']).targets[0],-.001)
        with self.assertRaises(CommandError):
            parse_command(['set_jog_coordinate','--to'])

    def test_unknown_commands_preserve_text_and_controller_reply(self):
        commands = ['native_init', '  custom --value="两个 words"  --pos=[1,2]  ']
        with FakeController() as c:
            for command in commands:
                c.command_replies[command] = {'ret_code': 0, 'ret_msg': 'accepted',
                                               'ret_context': 'plain controller text'}
            code, output = self.run_console(c, [*commands, 'get', 'exit'])
            self.assertEqual(code, 0)
            self.assertEqual([v for v in c.commands if v != 'get'], commands)
            self.assertIn('plain controller text', output)
            self.assertNotIn('操作未完成', output)
            self.assertFalse(c.operators[0].follower_stop_pending)

    def test_unknown_command_failure_displays_actual_response_and_keeps_shell(self):
        with FakeController() as c:
            c.command_replies['native_init'] = {'ret_code': 72, 'ret_msg': 'init rejected',
                                          'ret_context': {'reason': 'not ready'}}
            code, output = self.run_console(c, ['native_init', 'get', 'exit'])
            self.assertEqual(code, 0)
            self.assertIn('"ret_code": 72', output)
            self.assertIn('not ready', output)
            self.assertIn('控制器拒绝指令', output)
            self.assertNotIn('控制器已回复；未确认动作完成', output)

    def test_unknown_command_readonly_and_disconnected_do_not_send(self):
        with FakeController() as c:
            code, output = self.run_console(c, ['native_init', 'exit'], ['--read-only'])
            self.assertEqual(code, 0)
            self.assertNotIn('native_init', c.commands)
            self.assertIn('allow_motion is false', output)
        with FakeController() as c:
            c.command_replies['disconnect_me'] = None
            code, output = self.run_console(c, ['disconnect_me', 'native_init', 'help', 'exit'])
            self.assertEqual(code, 0)  # Arbitrary commands are not follower starts.
            self.assertIn('控制台保留', output)
            self.assertIn('可用指令', output)
            self.assertNotIn('native_init', c.commands)

    def test_unknown_command_invalidates_targets_but_preserves_cleanup(self):
        with FakeController() as c:
            c.command_replies['native_init'] = {'ret_code': 0, 'ret_context': ''}
            code, output = self.run_console(c, ['follower_aj', 'native_init',
                'joints --deg 0 0 0 0 0 0', 'reconnect', 'exit'])
            self.assertEqual(code, 0)
            self.assertIn('Axis follower start has not been confirmed', output)
            self.assertIn('连接已保留', output)
            self.assertEqual(c.commands.count('stop_follower'), 1)
            with self.assertRaises(socket.timeout):
                c.joint_udp.recvfrom(65535)

    def test_unknown_command_can_start_follower_without_granting_udp_permission(self):
        with FakeController() as c:
            def start():
                c.active, c.mode = True, 'Axis'
                return {'ret_code': 0, 'ret_context': ''}
            c.command_replies['custom_start'] = start
            code, output = self.run_console(c, ['custom_start',
                'joints --deg 0 0 0 0 0 0', 'exit'])
            self.assertEqual(code, 0)
            self.assertIn('Axis follower start has not been confirmed', output)
            self.assertFalse(c.active)
            self.assertEqual(c.commands.count('stop_follower'), 1)

    def test_one_shot_unknown_command(self):
        with FakeController() as c:
            c.command_replies['native_init --mode=1'] = {'ret_code': 0, 'ret_context': ''}
            out = []
            self.assertEqual(main(['--host', '127.0.0.1', 'native_init', '--mode=1'],
                operator_factory=lambda **_: c.operator(), output=out.append), 0)
            self.assertIn('native_init --mode=1', c.commands)

    def test_explicit_raw_preserves_native_arguments_for_builtin_name(self):
        with FakeController() as c:
            native = 'manual_en --native="two words"  --value=3'
            c.command_replies[native] = {'ret_code': 0, 'ret_context': 'native response'}
            code, output = self.run_console(c, ['raw ' + native, 'exit'])
            self.assertEqual(code, 0)
            self.assertIn(native, c.commands)
            self.assertIn('native response', output)
        with self.assertRaises(CommandError):
            parse_command(['raw'])

    def test_raw_start_stop_and_readonly_keep_ownership_rules(self):
        with FakeController() as c:
            code, output = self.run_console(c, ['raw follower_aj',
                'joints --deg 0 0 0 0 0 0', 'raw stop_follower', 'exit'])
            self.assertEqual(code, 0)
            self.assertIn('Axis follower start has not been confirmed', output)
            self.assertEqual(c.commands.count('stop_follower'), 1)
            self.assertFalse(c.active)
        with FakeController() as c:
            code, output = self.run_console(c, ['raw manual_en', 'exit'], ['--read-only'])
            self.assertEqual(code, 0)
            self.assertNotIn('manual_en', c.commands)

    def test_native_reply_budget_is_independent_from_get_timeout(self):
        with FakeController() as c:
            def slow_reply():
                time.sleep(.12)
                return {'ret_code': 0, 'ret_context': 'init reply'}
            c.command_replies['native_init'] = slow_reply
            code, output = self.run_console(c, ['native_init', 'get', 'exit'],
                ['--timeout', '.05', '--command-timeout', '.5'])
            self.assertEqual(code, 0)
            self.assertIn('init reply', output)
            self.assertEqual(c.commands.count('native_init'), 1)
            self.assertNotIn('stop_follower', c.commands)
            self.assertEqual(c.operators[0].timeout_s, .05)
            self.assertEqual(c.operators[0].command_timeout_s, .5)

    def test_native_timeout_has_no_fictitious_follower_and_reconnect_does_not_replay(self):
        with FakeController() as c:
            def late_reply():
                time.sleep(.12)
                return {'ret_code': 0, 'ret_context': 'late init reply'}
            c.command_replies['native_init'] = late_reply
            code, output = self.run_console(c,
                ['native_init', 'get', 'reconnect', 'get', 'exit'], ['--command-timeout', '.03'])
            self.assertEqual(code, 0)
            self.assertIn('执行结果未知', output)
            self.assertIn('为避免迟到回复串线', output)
            self.assertNotIn('之前的跟随尚未确认停止', output)
            self.assertNotIn('late init reply', output)
            self.assertNotIn('stop_follower', c.commands)
            self.assertEqual(c.commands.count('native_init'), 1)
            self.assertGreaterEqual(c.commands.count('get'), 4)
            self.assertFalse(c.operators[0].follower_stop_pending)

    def test_known_raw_follower_start_timeout_retains_stop_obligation(self):
        with FakeController() as c:
            def late_start():
                c.active, c.mode = True, 'Axis'
                time.sleep(.12)
                return {'ret_code': 0, 'ret_context': ''}
            c.command_replies['follower_aj'] = late_start
            code, output = self.run_console(c, ['raw follower_aj', 'exit'],
                ['--command-timeout', '.03'])
            self.assertEqual(code, 1)
            self.assertIn('之前的跟随尚未确认停止', output)
            self.assertEqual(c.commands.count('follower_aj'), 1)
            self.assertNotIn('stop_follower', c.commands)

    def test_native_default_timeout_is_longer_than_connection_budget(self):
        with FakeController() as c:
            code, output = self.run_console(c, ['exit'])
            self.assertEqual(code, 0)
            self.assertEqual(c.operators[0].command_timeout_s, 60.)
            self.assertEqual(c.operators[0].timeout_s, .15)


class DualConsoleTest(unittest.TestCase):
    def run_config(self, controller, commands, *, read_only=False, extra=()):
        settings = dict(type='A10Operator', host='127.0.0.1',
                        ws_port=controller.server.socket.getsockname()[1],
                        status_ws_port=controller.server.socket.getsockname()[1],
                        udp_port=controller.udp.getsockname()[1],
                        joint_udp_port=controller.joint_udp.getsockname()[1],
                        timeout_s=.3, allow_motion=not read_only,
                        feedback_position_unit='mm', arms=DUAL_ARMS,
                        cartesian_wire_format='pe321',
                        cartesian_target_order=['left','right'])
        output, lines = [], iter(commands)
        def read(_):
            try:
                return next(lines)
            except StopIteration:
                raise EOFError
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'dual.py'
            path.write_text('operator = ' + repr(settings))
            args = ['--config', str(path)]
            if read_only:
                args.append('--read-only')
            args.extend(extra)
            result = main([*args, 'shell'], operator_factory=lambda **kw: controller.operator(**kw),
                          input_fn=read, output=output.append)
        return result, '\n'.join(output)

    def test_jog_parser_units_bounds_and_session_only(self):
        import math
        args = parse_command(['jog','left','x','-1e-3','right','rz','1'])
        self.assertEqual(args.deltas['left'], [-.000001,0,0,0,0,0])
        self.assertEqual(args.deltas['right'][-1], math.radians(1))
        for moves in (['left','x','nan'], ['left','x','11'], ['left','rx','6'],
                      ['left','xx','1'], ['left','x','1','left','y','1'], ['left','x']):
            with self.assertRaises(CommandError): parse_command(['jog',*moves])
        with patch('socket.socket', side_effect=AssertionError('unexpected network')):
            self.assertEqual(main(['--host','example','jog','left','x','1'], output=lambda _:None), 2)

    def test_dual_jog_full_console_zero_hold_and_stop(self):
        with FakeController() as c:
            c.motion_override = dual_motion()
            result, output = self.run_config(c, ['follower_cart', 'jog left x 0 right x 0',
                                                'stop_follower', 'exit'])
            self.assertEqual(result, 0)
            self.assertNotIn('操作未完成', output)
            self.assertIn('连续 3 次落入容差', output)
            packet = c.udp.recv(4096)
            self.assertEqual(json.loads(packet[40:]), {'type':'321', 'pe':[[0]*6,[0]*6]})
            self.assertEqual([x for x in c.commands if x!='get'], ['follower_cart','stop_follower'])

    def test_dual_jog_readonly_cannot_start_or_send(self):
        with FakeController() as c:
            c.motion_override = dual_motion()
            result, output = self.run_config(c, ['jog left x 1','exit'], read_only=True)
            self.assertEqual(result, 0)
            self.assertIn('allow_motion is false', output)
            self.assertEqual(set(c.commands), {'get'})
            with self.assertRaises(socket.timeout): c.udp.recv(4096)

    def test_allow_motion_explicitly_overrides_safe_config(self):
        with FakeController() as c, tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'safe.py'
            path.write_text("operator = dict(type='A10Operator', host='127.0.0.1', allow_motion=False)")
            captured = []
            def factory(**settings):
                captured.append(settings)
                return c.operator(**settings)
            result = main(['--config',str(path),'--allow-motion','manual_en'],
                          operator_factory=factory, output=lambda _:None)
            self.assertEqual(result,0)
            self.assertTrue(captured[0]['allow_motion'])

    def test_readonly_config_without_status_port_uses_its_command_port(self):
        with FakeController() as c, tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'single_ws.py'
            port = c.server.socket.getsockname()[1]
            path.write_text(f"operator = dict(type='A10Operator', host='127.0.0.1', ws_port={port})")
            captured=[]
            def factory(**settings):
                captured.append(settings)
                return c.operator(**settings)
            result=main(['--config',str(path),'--read-only','get'],operator_factory=factory,output=lambda _:None)
            self.assertEqual(result,0)
            self.assertEqual(captured[0]['ws_port'],port)
            self.assertEqual(set(c.commands),{'get'})

    def test_dual_get_joints_global_lifecycle_and_incomplete_commands(self):
        with FakeController() as c:
            c.motion_override = dual_motion()
            result, output = self.run_config(c, [
                'get','joints','follower_cart','pq 0 0 0 0 0 0 1',
                'joints --deg 0 0 0 0 0 0','g_move --pos=10','follower_aj','exit'])
            self.assertEqual(result, 0)
            self.assertIn('left（ee_id=0）', output)
            self.assertIn('right（ee_id=1）', output)
            self.assertIn('整个双臂控制组', output)
            self.assertIn('双臂模式拒绝', output)
            self.assertIn('可复制为目标', output)
            self.assertEqual([x for x in c.commands if x!='get'], ['follower_cart','stop_follower'])
            for sock in (c.udp,c.joint_udp):
                with self.assertRaises(TimeoutError):
                    sock.recv(65535)

    def test_readonly_config_queries_only_even_with_start_requested(self):
        with FakeController() as c:
            c.motion_override = dual_motion()
            result, output = self.run_config(c, ['get','follower_cart','manual_en','exit'], read_only=True)
            self.assertEqual(result, 0)
            self.assertIn('allow_motion is false', output)
            self.assertEqual(set(c.commands), {'get'})

    def test_single_controller_cannot_bootstrap_dual_config_or_send_start(self):
        with FakeController() as c:
            result, output = self.run_config(c, ['follower_cart','exit'])
            self.assertEqual(result, 0)
            self.assertIn('configured feedback group', output)
            self.assertTrue(all(x=='get' for x in c.commands))

    def test_bad_config_type_does_not_construct_or_connect(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'wrong.py'
            path.write_text("operator = dict(type='OtherOperator', host='127.0.0.1')")
            def never(**_):
                self.fail('invalid operator type constructed')
            output = []
            result = main(['--config',str(path),'get'],operator_factory=never,output=output.append)
            self.assertEqual(result,1)
            self.assertIn('必须使用 A10Operator','\n'.join(output))


class MergedConsoleCompatibilityTest(unittest.TestCase):
    def capture(self, args, settings=None):
        from unittest.mock import MagicMock
        captured = []
        with FakeController() as controller, tempfile.TemporaryDirectory() as directory:
            if settings is not None:
                path = Path(directory) / 'config.py'
                path.write_text('operator = ' + repr(dict(type='A10Operator', **settings)))
                args = ['--config', str(path), *args]
            def factory(**options):
                captured.append(options)
                return controller.operator(allow_motion=options.get('allow_motion', False))
            service = MagicMock()
            with patch('fluxvla.engines.communication.a10_shared_server.ConsoleSharedControl',
                       return_value=service) as constructor:
                code = control_main([*args,'get'], operator_factory=factory, output=lambda _:None)
            return code, captured, constructor, service

    def test_legacy_no_config_keeps_shared_defaults(self):
        code, captured, constructor, service = self.capture([])
        self.assertEqual(code, 0)
        self.assertEqual(captured[0]['host'], '192.168.0.125')
        self.assertEqual(captured[0]['ws_port'], 5999)
        self.assertEqual(captured[0]['shared_control']['role'], 'console')
        constructor.assert_called_once()
        service.start.assert_called_once()

    def test_dual_config_defaults_direct_without_overwriting_endpoints(self):
        code, captured, constructor, _ = self.capture([], dict(
            host='192.168.0.150', ws_port=5867, status_ws_port=5867,
            allow_motion=False, arms=DUAL_ARMS))
        self.assertEqual(code, 0)
        self.assertEqual(captured[0]['host'], '192.168.0.150')
        self.assertEqual(captured[0]['ws_port'], 5867)
        self.assertNotIn('shared_control', captured[0])
        self.assertFalse(captured[0]['allow_motion'])
        constructor.assert_not_called()

    def test_explicit_overrides_win_and_direct_removes_config_shared(self):
        code, captured, constructor, _ = self.capture(
            ['--direct','--host','127.0.0.1','--ws-port','1234','--read-only'],
            dict(host='example', ws_port=5867, status_ws_port=5888,
                 shared_control=dict(port=9876), allow_motion=True))
        self.assertEqual(code, 0)
        self.assertEqual(captured[0]['host'], '127.0.0.1')
        self.assertEqual(captured[0]['ws_port'], 1234)
        self.assertNotIn('shared_control', captured[0])
        self.assertFalse(captured[0]['allow_motion'])
        constructor.assert_not_called()

    def test_shared_override_and_dual_shared_config_are_accepted(self):
        code, captured, constructor, _ = self.capture(['--shared-port','12345'],
            dict(host='example', shared_control=dict(port=9876)))
        self.assertEqual(code, 0)
        self.assertEqual(captured[0]['shared_control']['port'], 12345)
        self.assertEqual(constructor.call_args.args[0], ('127.0.0.1', 12345))
        code, captured, constructor, _ = self.capture(['--shared'],
            dict(host='example', arms=DUAL_ARMS))
        self.assertEqual(code, 0)
        self.assertEqual(captured[0]['shared_control']['role'], 'console')
        constructor.assert_called_once()


if __name__ == '__main__':
    unittest.main()
