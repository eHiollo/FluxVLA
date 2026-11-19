"""单入口网页、HTTP接口和假串口回归；不连接真实夹爪。"""

import contextlib
import http.client
import io
import json
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import MagicMock, patch

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
try:
    import a10_gripper_ui as ui
finally:
    sys.path.pop(0)


class BackendTests(unittest.TestCase):
    def setUp(self):
        self.backend = ui.GripperBackend(ui.devices.ROOT / 'configs/a10/a10_dual_devices.py')

    def test_constructor_does_not_scan_or_connect(self):
        with patch.object(ui.devices, 'scan_devices') as scan, patch.object(ui.devices, 'load_operator') as driver:
            self.setUp()
        scan.assert_not_called()
        driver.assert_not_called()

    def test_motion_uses_existing_identity_checks_and_side(self):
        with patch.object(ui.devices, 'scan_devices', return_value=[]) as scan, \
                patch.object(ui.devices, 'move_gripper', return_value={'outcome': 'timeout'}) as move:
            result = self.backend.execute('right', 'close')
        scan.assert_called_once()
        self.assertEqual(move.call_args.args[2:], ('right', 'close', None))
        self.assertEqual(result['outcome'], 'timeout')

    def test_speed_validation_and_override_does_not_change_config(self):
        original = self.backend.config_path.read_bytes()
        with patch.object(ui.devices, 'scan_devices', return_value=[]), \
                patch.object(ui.devices, 'move_gripper', return_value={}) as move:
            self.backend.execute('right', 'close', speed=50)
            args = move.call_args.args
            self.assertEqual(args[0]['grippers']['right']['speed'], 50)
            self.assertEqual(args[0]['grippers']['left']['speed'], 5)
            self.assertEqual(move.call_args.kwargs['max_speed'], 50)
        self.assertEqual(self.backend.config_path.read_bytes(), original)
        self.assertEqual(self.backend.config['grippers']['right']['speed'], 5)
        with patch.object(ui.devices, 'scan_devices') as scan:
            for speed in (0, 51, 100, True, 1.5, '50'):
                with self.subTest(speed=speed), self.assertRaises(ValueError):
                    self.backend.execute('both', 'close', speed=speed)
            scan.assert_not_called()

    def dual_transport(self):
        sys.path.insert(0, str(Path(__file__).parent))
        try:
            from test_zx_gripper import FakeSerial
        finally:
            sys.path.pop(0)
        rows, transports = [], {}
        for side, settings in self.backend.config['grippers'].items():
            rows.append(dict(group='grippers', name=side, label=side, status='matched',
                             serial=settings['identity']['serial'], configured_path=settings['port']))
            transport = FakeSerial()
            transport.alarms = (0, 0)
            transport.position = 4500
            transports[settings['port']] = transport
        return rows, transports

    def test_both_moves_run_concurrently_at_selected_speed(self):
        rows, transports = self.dual_transport()
        barrier = threading.Barrier(2)
        execute = ui.devices.execute_check_move
        def synchronized(client, settings, row, target, action, **kwargs):
            self.assertEqual(settings['speed'], 50)
            self.assertEqual(target, 0)
            barrier.wait(timeout=2)  # Sequential execution would fail here.
            return execute(client, settings, row, target, action, **kwargs)
        with patch.object(ui.devices, 'scan_devices', return_value=[]), \
                patch.object(ui.devices, 'match_devices', return_value=rows), \
                patch('serial.Serial', side_effect=lambda port, **kwargs: transports[port]), \
                patch.object(ui.devices, 'execute_check_move', side_effect=synchronized):
            result = self.backend.execute('both', 'open', speed=50)
        for side in ('left', 'right'):
            self.assertTrue(result['results'][side]['data']['arrived'])
            self.assertEqual(result['results'][side]['data']['speed'], 50)
        self.assertTrue(all(t.closed for t in transports.values()))

    def test_both_preflight_alarm_prevents_all_motion(self):
        rows, transports = self.dual_transport()
        transports[self.backend.config['grippers']['right']['port']].alarms = (1, 0)
        with patch.object(ui.devices, 'scan_devices', return_value=[]), \
                patch.object(ui.devices, 'match_devices', return_value=rows), \
                patch('serial.Serial', side_effect=lambda port, **kwargs: transports[port]):
            with self.assertRaisesRegex(RuntimeError, '报警'):
                self.backend.execute('both', 'close', speed=35)
        for transport in transports.values():
            self.assertTrue(transport.closed)
            self.assertTrue(all(packet[1] == 3 for packet in transport.writes))

    def test_drop_warning_only_allows_exact_drop_alarm_and_keeps_raw_codes(self):
        for policy, alarms, allowed in (
                ('stop', (0, 0x1000), False), ('warn', (0, 0x1000), True),
                ('warn', (1, 0x1000), False), ('warn', (0, 0x1001), False),
                ('warn', (0, 0x2000), False)):
            with self.subTest(policy=policy, alarms=alarms):
                rows, transports = self.dual_transport()
                transport = transports[self.backend.config['grippers']['left']['port']]
                transport.alarms = alarms
                with patch.object(ui.devices, 'scan_devices', return_value=[]), \
                        patch.object(ui.devices, 'match_devices', return_value=rows), \
                        patch('serial.Serial', side_effect=lambda port, **kwargs: transports[port]):
                    if allowed:
                        result = self.backend.execute('both', 'open', speed=50, drop_alarm_policy=policy)
                        feedback = result['results']['left']['data']
                        self.assertTrue(feedback['arrived'])
                        self.assertEqual(feedback['final']['alarm_b'], 0x1000)
                        self.assertEqual(feedback['drop_alarm_policy'], 'warn')
                    else:
                        with self.assertRaisesRegex(RuntimeError, '报警'):
                            self.backend.execute('both', 'open', speed=50, drop_alarm_policy=policy)
                        self.assertTrue(all(packet[1] == 3 for t in transports.values() for packet in t.writes))
                # Never reset alarms automatically, even when dropping the warning.
                self.assertEqual(transport.alarms, alarms)

    def test_both_keeps_other_side_result_when_one_side_fails(self):
        rows, transports = self.dual_transport()
        def execute(client, settings, row, *args, **kwargs):
            if row['name'] == 'left':
                raise RuntimeError('测试通信失败')
            return dict(outcome='position_reached', speed=settings['speed'])
        with patch.object(ui.devices, 'scan_devices', return_value=[]), \
                patch.object(ui.devices, 'match_devices', return_value=rows), \
                patch('serial.Serial', side_effect=lambda port, **kwargs: transports[port]), \
                patch.object(ui.devices, 'execute_check_move', side_effect=execute):
            result = self.backend.execute('both', 'close', speed=20)
        self.assertEqual(result['results']['left']['error'], '测试通信失败')
        self.assertEqual(result['results']['right']['data']['outcome'], 'position_reached')

    def test_missing_device_never_opens_serial(self):
        with patch.object(ui.devices, 'scan_devices', return_value=[]), \
                patch.object(ui.devices, 'load_operator') as driver:
            for action in ('status', 'open', 'close'):
                with self.subTest(action=action), self.assertRaises(ValueError):
                    self.backend.execute('right', action)
        driver.assert_not_called()


class WebTests(unittest.TestCase):
    def setUp(self):
        self.backend = MagicMock()
        self.backend.config = dict(grippers={s: dict(check=dict(min_raw=0, max_raw=9000))
                                            for s in ('left', 'right')})
        self.server = ui.DebugServer(('127.0.0.1', 0), self.backend)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.addCleanup(self.cleanup)

    def cleanup(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def request(self, method, path, body=None, token=True, host=None):
        conn = http.client.HTTPConnection(*self.server.server_address, timeout=3)
        headers = {'Content-Type': 'application/json'}
        if token:
            headers['X-Debug-Token'] = self.server.token
        if host:
            headers['Host'] = host
        conn.request(method, path, json.dumps(body) if body is not None else None, headers)
        response = conn.getresponse()
        result = response.status, response.read().decode()
        conn.close()
        return result

    def test_page_and_poll_do_not_connect_hardware(self):
        code, page = self.request('GET', '/')
        self.assertEqual(code, 200)
        self.assertIn(self.server.token, page)
        self.assertNotIn('__DEBUG_TOKEN__', page)
        self.assertEqual(self.request('GET', '/api/state')[0], 200)
        self.backend.execute.assert_not_called()

    def test_auth_host_and_position_validation_prevent_actions(self):
        action = dict(side='right', action='close')
        self.assertEqual(self.request('POST', '/api/action', action, token=False)[0], 403)
        self.assertEqual(self.request('GET', '/', host='example.com')[0], 403)
        for target in (-1, 9001, True, 1.5, '4500'):
            self.assertEqual(self.request('POST', '/api/action', dict(side='right', action='position', target=target))[0], 400)
        self.backend.execute.assert_not_called()

    def test_lan_page_requires_link_token_and_api_still_requires_header(self):
        self.server.public_mode = True
        self.assertEqual(self.request('GET', '/', host='192.168.0.10:8765')[0], 403)
        self.assertEqual(self.request('GET', '/?token=wrong')[0], 403)
        self.assertEqual(self.request('GET', '/?token=' + self.server.token,
                                      host='192.168.0.10:8765')[0], 200)
        self.assertEqual(self.request('GET', '/api/state', token=False)[0], 403)
        self.assertEqual(self.request('POST', '/api/action',
                                      dict(side='right', action='close'), token=False)[0], 403)
        self.backend.execute.assert_not_called()

    def test_speed_validation_and_dual_action_forwarding(self):
        for speed in (0, 51, 100, True, 1.5, '50'):
            code, _ = self.request('POST', '/api/action',
                                   dict(side='both', action='open', speed=speed))
            self.assertEqual(code, 400)
        for action in ('status', 'position'):
            code, _ = self.request('POST', '/api/action',
                                   dict(side='both', action=action, target=4500))
            self.assertEqual(code, 400)
        self.backend.execute.assert_not_called()
        self.backend.execute.return_value = dict(results=dict(left=dict(error='test'), right=dict(error='test')))
        self.assertEqual(self.request('POST', '/api/action',
                                      dict(side='both', action='close', speed=50))[0], 202)
        deadline = time.monotonic() + 2
        while self.server.busy and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertFalse(self.server.busy)
        self.backend.execute.assert_called_once_with('both', 'close', None, speed=50)
        state = json.loads(self.request('GET', '/api/state')[1])
        self.assertEqual(state['result']['data']['results']['right']['error'], 'test')

    def test_drop_policy_is_bounded_and_forwarded(self):
        for policy in ('ignore_all', True, 1, None):
            self.assertEqual(self.request('POST', '/api/action',
                             dict(side='left', action='status', drop_alarm_policy=policy))[0], 400)
        self.backend.execute.assert_not_called()
        self.backend.execute.return_value = {}
        self.assertEqual(self.request('POST', '/api/action',
                         dict(side='left', action='status', drop_alarm_policy='warn'))[0], 202)
        deadline = time.monotonic() + 2
        while self.server.busy and time.monotonic() < deadline:
            time.sleep(.01)
        self.backend.execute.assert_called_once_with('left', 'status', None, drop_alarm_policy='warn')

    def test_busy_rejects_repeated_action_and_returns_failure(self):
        released = threading.Event()
        self.addCleanup(released.set)
        def execute(*args):
            released.wait(2)
            raise RuntimeError('测试串口占用')
        self.backend.execute.side_effect = execute
        action = dict(side='right', action='close')
        self.assertEqual(self.request('POST', '/api/action', action)[0], 202)
        self.assertEqual(self.request('POST', '/api/action', action)[0], 409)
        released.set()
        deadline = time.monotonic() + 2
        while self.server.busy and time.monotonic() < deadline:
            time.sleep(.01)
        code, response = self.request('GET', '/api/state')
        state = json.loads(response)
        self.assertFalse(state['busy'])
        self.assertEqual(state['result']['error'], '测试串口占用')
        self.backend.execute.assert_called_once_with('right', 'close', None)

class EntryTests(unittest.TestCase):
    def test_default_entry_starts_browser_service_without_hardware(self):
        with patch.object(ui, 'serve') as serve, \
                patch.object(ui.devices, 'scan_devices') as scan, \
                patch.object(ui.devices, 'load_operator') as driver:
            ui.main([])
        serve.assert_called_once()
        self.assertIsInstance(serve.call_args.args[0], ui.GripperBackend)
        self.assertEqual(serve.call_args.args[1:], (8765, '192.168.110.119'))
        scan.assert_not_called()
        driver.assert_not_called()

    def test_invalid_port_and_obsolete_desktop_flag_do_not_start(self):
        for flags in (['--port', '0'], ['--port', '65536'], ['--desktop']):
            with self.subTest(flags=flags), patch.object(ui, 'serve') as serve, \
                    contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    ui.main(flags)
                self.assertEqual(error.exception.code, 2)
                serve.assert_not_called()

    def test_custom_connection_options(self):
        with patch.object(ui, 'serve') as serve:
            ui.main(['--host', '127.0.0.1', '--port', '8766'])
        self.assertEqual(serve.call_args.args[1:], (8766, '127.0.0.1'))


if __name__ == '__main__':
    unittest.main()
