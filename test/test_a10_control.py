"""CLI regressions against the independent loopback follower server."""

import contextlib
import io
import json
from pathlib import Path
import socket
import subprocess
import sys
import unittest
from unittest.mock import patch

from scripts.a10_control import main, parse_command, CommandError
from test_a10_operator import FakeController


class ConsoleTest(unittest.TestCase):
    def test_script_entrypoint_from_another_directory(self):
        script = Path(__file__).resolve().parents[1] / 'scripts/a10_control.py'
        with FakeController() as c:
            c.activation = 'Disabled'
            result = subprocess.run([sys.executable, str(script), '--host', '127.0.0.1',
                '--ws-port', str(c.server.socket.getsockname()[1]),
                '--udp-port', str(c.udp.getsockname()[1]),
                '--joint-udp-port', str(c.joint_udp.getsockname()[1]), '--timeout', '1', 'shell'],
                input='manual_en\nfollower_aj\njoints --deg 0 90 -90 180 0 45\nstop_follower\nfollower_cart\nstop_follower\nmanual_ds\nexit\n',
                text=True, capture_output=True, cwd='/tmp', timeout=60)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertNotIn('操作未完成', result.stdout)
            self.assertIn('手动使能成功', result.stdout)
            self.assertEqual([cmd for cmd in c.commands if cmd != 'get'],
                ['manual_en', 'follower_aj', 'stop_follower', 'follower_cart', 'stop_follower', 'manual_ds'])
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
                             ['set_jog_coordinate --axis','follower_aj','stop_follower'])
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
                self.assertIn('本次跟随已停止',output)

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
                self.assertIn('本次跟随已停止', output)
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
            code, output = self.run_console(c, ['bogus', '"unterminated',
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
            for cmd in (['follower_cart'], ['stop_follower'], ['pq','0','0','0','0','0','0','1'], ['bad']):
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


if __name__ == '__main__':
    unittest.main()
