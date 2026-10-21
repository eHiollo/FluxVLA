"""Frequency experiment against an independent packet-driven loopback plant."""
import contextlib
import io
import json
from pathlib import Path
import socket
import tempfile
import threading
import unittest

from scripts.a10_follower_frequency_test import main
from fluxvla.engines.communication.a10_shared_server import SharedControl, SharedControlServer
from test_a10_operator import FakeController, status_reply


class FrequencyTest(unittest.TestCase):
    def test_rates_change_response_and_logs_capture_targets_and_raw_state(self):
        with FakeController() as controller, tempfile.TemporaryDirectory() as directory:
            q = [0., 90., -90., 180., 0., 45.]
            lock, stop = threading.Lock(), threading.Event()
            errors = []
            def get():
                reply = status_reply(active=controller.active, mode=controller.mode)
                with lock:
                    reply['ret_context']['motion_msg']['actual_pos'] = [q.copy()]
                    reply['ret_context']['motion_msg']['motor_pos'] = [q.copy()]
                return reply
            controller.command_replies['get'] = get
            def plant():
                try:
                    while not stop.is_set():
                        try:
                            packet = controller.joint_udp.recv(65535)
                        except socket.timeout:
                            continue
                        text = packet[40:].decode()
                        self.assertTrue(text.startswith('jp --motor_pos='), text)
                        target = json.loads(text.split('=', 1)[1])
                        with lock:
                            for i, value in enumerate(target):
                                # Independent simulation deliberately advances once
                                # per packet, so higher rates must finish sooner.
                                delta = value - q[i]
                                q[i] += max(-.1, min(.1, delta))
                except BaseException as exc:
                    errors.append(exc)
            plant_thread = threading.Thread(target=plant)
            plant_thread.start()
            port = controller.server.socket.getsockname()[1]
            robot = SharedControl('127.0.0.1', port, port,
                controller.udp.getsockname()[1], controller.joint_udp.getsockname()[1])
            robot.connect()
            server = SharedControlServer(('127.0.0.1', 0), robot)
            thread = threading.Thread(target=server.serve_forever, kwargs=dict(poll_interval=.01))
            thread.start()
            try:
                output = io.StringIO()
                with contextlib.redirect_stdout(output):
                    result = main(['--host', '127.0.0.1', '--ws-port', str(port),
                        '--status-ws-port', str(port), '--shared-port', str(server.server_address[1]),
                        '--udp-port', str(robot.udp_port), '--joint-udp-port', str(robot.joint_udp_port),
                        '--rates', '20', '80', '--delta-deg', '1', '--tolerance-deg', '.05',
                        '--settle-s', '.08', '--timeout', '3', '--log-dir', directory])
                self.assertEqual(result, 0, output.getvalue())
                self.assertFalse(controller.active)
                self.assertFalse(errors)
                logdir = next(Path(directory).glob('*_follower_frequency'))
                report = json.loads((logdir / 'comparison.json').read_text())
                self.assertTrue(report['comparison_valid'], report)
                self.assertEqual(report['dropped_events'], 0)
                slow, fast = report['results']
                self.assertGreater(slow['time_to_90pct_s'], fast['time_to_90pct_s'] * 1.5)
                self.assertAlmostEqual(slow['measured_send_hz'], 20, delta=2)
                self.assertAlmostEqual(fast['measured_send_hz'], 80, delta=5)
                self.assertLess(max(abs(x-y) for x,y in zip(q, [0, 90, -90, 180, 0, 45])), .051)
                with (logdir / 'events.jsonl').open() as stream:
                    events = [json.loads(line) for line in stream]
                self.assertTrue(any(r['event'] == 'state' and 'motion_msg' in r['raw']['ret_context'] for r in events))
                sends = [r for r in events if r['event'] == 'send']
                self.assertTrue(sends)
                self.assertTrue(all(r['target_deg'][3] in (180, 181) for r in sends))
                self.assertGreater((logdir / 'sends.csv').stat().st_size, 100)
                self.assertGreater((logdir / 'states.csv').stat().st_size, 100)
            finally:
                server.shutdown()
                thread.join(2)
                server.server_close()
                robot.close()
                stop.set()
                plant_thread.join(2)

    def test_no_motion_timeout_stops_follower_and_keeps_partial_measurements(self):
        with FakeController() as controller, tempfile.TemporaryDirectory() as directory:
            port = controller.server.socket.getsockname()[1]
            # No simulated plant consumes targets: actual joints stay fixed.
            robot = SharedControl('127.0.0.1', port, port,
                controller.udp.getsockname()[1], controller.joint_udp.getsockname()[1])
            robot.connect()
            server = SharedControlServer(('127.0.0.1', 0), robot)
            thread = threading.Thread(target=server.serve_forever, kwargs=dict(poll_interval=.01))
            thread.start()
            try:
                with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                    result = main(['--host', '127.0.0.1', '--ws-port', str(port),
                        '--status-ws-port', str(port), '--shared-port', str(server.server_address[1]),
                        '--udp-port', str(robot.udp_port), '--joint-udp-port', str(robot.joint_udp_port),
                        '--rates', '30', '--settle-s', '.02', '--timeout', '.2', '--log-dir', directory])
                self.assertEqual(result, 1)
                self.assertFalse(controller.active)
                self.assertEqual(controller.commands.count('stop_follower'), 1)
                logdir = next(Path(directory).glob('*_follower_frequency'))
                report = json.loads((logdir / 'comparison.json').read_text())
                self.assertFalse(report['comparison_valid'])
                self.assertIn('TimeoutError', report['error'])
                with (logdir / 'events.jsonl').open() as stream:
                    events = [json.loads(line) for line in stream]
                self.assertTrue(any(r['event'] == 'send' for r in events))
                self.assertTrue(any(r['event'] == 'follower_stop' and r['confirmed'] for r in events))
                self.assertFalse(any(r.get('phase', '').endswith('_return') for r in events))
            finally:
                server.shutdown()
                thread.join(2)
                server.server_close()
                robot.close()

    def test_invalid_experiment_does_not_open_connections(self):
        from unittest.mock import patch
        for args in (['--delta-deg', '90'], ['--rates', '0'], ['--sample-hz', '500'],
                     ['--tolerance-deg', '5'], ['--rates', 'nan']):
            with self.subTest(args=args), patch('socket.socket', side_effect=AssertionError('network')), \
                    contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                main(args)
            self.assertEqual(error.exception.code, 2)


if __name__ == '__main__':
    unittest.main()
