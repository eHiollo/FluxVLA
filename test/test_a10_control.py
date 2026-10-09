"""CLI regressions against the independent loopback follower server."""

import contextlib
import io
import json
import socket
import unittest
from unittest.mock import patch

from scripts.a10_control import main, parse_command, CommandError
from test_a10_operator import FakeController


class ConsoleTest(unittest.TestCase):
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
                       '--timeout', '.15', *extra, 'shell'],
                      operator_factory=factory, input_fn=read, output=output.append)
        return result, '\n'.join(output)

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
                'set_jog_coordinate --tool','follower_cart','stop_follower',
                'pq 0 0 0 0 0 0 1','g_move --pos=20','exit'], ['--read-only'])
            self.assertEqual(code, 0)
            self.assertTrue(all(cmd == 'get' for cmd in c.commands))
            self.assertIn('allow_motion is false', output)
            with self.assertRaises(socket.timeout):
                c.udp.recvfrom(65535)

    def test_malformed_commands_keep_shell_usable_and_send_no_controls(self):
        with FakeController() as c:
            code, output = self.run_console(c, ['bogus', '"unterminated',
                'manual_en --typo', 'set_jog_coordinate --axis',
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
                line = next(steps)
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
        with self.assertRaises(CommandError):
            parse_command(['set_jog_coordinate','--to'])


if __name__ == '__main__':
    unittest.main()
