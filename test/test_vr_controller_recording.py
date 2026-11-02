"""Controller recorder protocol/lifecycle: in-memory and loopback, no devices."""
import copy
import io
import json
import socket
import tempfile
import threading
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest.mock import patch

from mmengine import Config
from fluxvla.engines.operators.a10_operator import A10ControllerError, A10ProtocolError
from fluxvla.engines.operators.vr.runtime import build_runtime
from test_a10_operator import FakeController
from test_vr_reference_runtime import StoppableA10, Clock, runtime, sample

ROOT = Path(__file__).resolve().parents[1]


def status(**changes):
    result = dict(connected=True, recording=True, closing=False, session_id='synthetic-session',
                  file='log/robot_state/synthetic-session-0.rlog', attempted=250, written=250,
                  synced=240, last_synced_cycle=12345, dropped=0, stop_reason='none',
                  error='', heartbeat_timeout_ms=5000, sync_ms=100)
    result.update(changes)
    return result


def reply(**changes):
    return dict(ret_code=0, ret_msg='', ret_context=status(**changes))


class RecorderProtocolTest(unittest.TestCase):
    def test_real_wire_commands_get_and_concurrent_queries_share_one_connection(self):
        with FakeController() as server:
            for command in ('record_start', 'record_status', 'record_heartbeat'):
                server.command_replies[command] = reply()
            robot = server.operator(allow_motion=False)
            ws = robot._connection.ws
            first = robot.start_runtime_recording()
            self.assertEqual(robot.start_runtime_recording()['session_id'], first['session_id'])
            errors = []
            def query():
                try:
                    for _ in range(3):
                        robot.get_state()
                        self.assertEqual(robot.get_runtime_recording_status()['synced'], 240)
                        robot.heartbeat_runtime_recording()
                except BaseException as e:
                    errors.append(e)
            threads = [threading.Thread(target=query) for _ in range(2)]
            for thread in threads: thread.start()
            for thread in threads: thread.join(timeout=5)
            self.assertFalse(any(thread.is_alive() for thread in threads))
            self.assertEqual(errors, [])
            self.assertIs(robot._connection.ws, ws)
            self.assertEqual(set(server.commands), {'get', 'record_start', 'record_status', 'record_heartbeat'})
            with self.assertRaises(socket.timeout): server.udp.recv(1024)

    def test_ack_is_not_enough_to_claim_recording_started(self):
        cases = [dict(recording=False), dict(closing=True), dict(error='disk full'), dict(session_id='')]
        with FakeController() as server:
            robot = server.operator(allow_motion=False)
            for changes in cases:
                server.command_replies['record_start'] = reply(**changes)
                with self.subTest(changes=changes), self.assertRaises(RuntimeError):
                    robot.start_runtime_recording()
            server.command_replies['record_start'] = dict(ret_code=0, ret_context={})
            with self.assertRaises(A10ProtocolError): robot.start_runtime_recording()
            server.command_replies['record_start'] = dict(ret_code=-1, ret_msg='recorder unavailable')
            with self.assertRaises(A10ControllerError): robot.start_runtime_recording()
            self.assertFalse(robot.is_connected)

    def test_disconnected_heartbeat_is_not_reconnected_or_retried(self):
        with FakeController() as server:
            server.command_replies['record_heartbeat'] = None
            robot = server.operator(allow_motion=False)
            with self.assertRaises(Exception): robot.heartbeat_runtime_recording()
            with self.assertRaises(ConnectionError): robot.get_runtime_recording_status()
            self.assertEqual(server.commands.count('record_heartbeat'), 1)
            self.assertNotIn('record_start', server.commands)


class RecordingRobot(StoppableA10):
    def __init__(self):
        super().__init__()
        self.record_status = status()
        self.start_error = False

    def start_runtime_recording(self):
        self.events.append('record_start')
        if self.start_error: raise RuntimeError('recorder unavailable')
        return copy.deepcopy(self.record_status)

    def heartbeat_runtime_recording(self):
        self.events.append('record_heartbeat')
        return copy.deepcopy(self.record_status)


class RecorderRuntimeTest(unittest.TestCase):
    def test_records_before_motion_survives_stops_and_idle_heartbeats_correlate_logs(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'vr.jsonl'
            robot = RecordingRobot()
            rows = [sample(1), sample(2), sample(3, active=False)]
            rows += [sample(i, active=False) for i in range(4, 44)]
            rows += [sample(44, anchor=2), sample(45, anchor=2)]
            op, _ = runtime(rows, robot=robot, record_controller=True, diagnostics_path=path)
            clock = Clock()
            with patch('time.monotonic', clock.now):
                op.connect()
                for _ in rows:
                    clock.advance(); op.step()
                op.close()
                op.close()  # Final event is not duplicated.
            self.assertLess(robot.events.index('record_start'), robot.events.index(('start', 'cartesian')))
            self.assertEqual(robot.events.count('record_start'), 1)
            self.assertEqual(robot.events.count(('start', 'cartesian')), 2)
            self.assertEqual(robot.events.count('record_heartbeat'), 1)
            self.assertLess(robot.events.index('stop'), robot.events.index('close'))
            logs = [json.loads(line) for line in path.read_text().splitlines()]
            recorded = [d for d in logs if d.get('control_frame_index') is not None]
            self.assertTrue(all(d['controller_recording']['session_id'] == 'synthetic-session' for d in recorded))
            self.assertTrue(all(d['client_monotonic_ns'] > 0 and d['client_wall_ns'] > 0 for d in recorded))
            self.assertEqual(logs[-1]['controller_recording']['state'], 'disconnected_save_unconfirmed')
            self.assertEqual(len([d for d in logs if d.get('controller_recording', {}).get('state') == 'disconnected_save_unconfirmed']), 1)
            self.assertNotIn('record_stop', robot.events)

    def test_start_failure_and_short_heartbeat_budget_prevent_follower_start(self):
        for failure in ('start', 'short_timeout'):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as folder:
                robot = RecordingRobot()
                robot.start_error = failure == 'start'
                if failure == 'short_timeout': robot.record_status['heartbeat_timeout_ms'] = 1000
                op, _ = runtime([sample(1)], robot=robot, record_controller=True,
                                diagnostics_path=Path(folder) / 'vr.jsonl')
                with self.assertRaises(RuntimeError): op.connect()
                self.assertFalse(robot.targets)
                self.assertNotIn('tool', robot.events)
                self.assertNotIn(('start', 'cartesian'), robot.events)

    def test_interrupted_recording_stops_group_before_next_motion_and_never_restarts(self):
        for change in (dict(recording=False, stop_reason='heartbeat_timeout'),
                       dict(error='io_error'), dict(session_id='unexpected-session')):
            with self.subTest(change=change), tempfile.TemporaryDirectory() as folder:
                robot = RecordingRobot()
                rows = [sample(i) for i in range(1, 36)]
                op, _ = runtime(rows, robot=robot, record_controller=True,
                                diagnostics_path=Path(folder) / 'vr.jsonl')
                clock = Clock()
                with patch('time.monotonic', clock.now):
                    op.connect()
                    for _ in range(29): clock.advance(); op.step()
                    robot.record_status.update(change)
                    count = len(robot.targets)
                    clock.value = op._recording_next_heartbeat + .001
                    with self.assertRaisesRegex(RuntimeError, 'recording interrupted'):
                        op.step()
                    self.assertEqual(len(robot.targets), count)
                    self.assertEqual(robot.events.count('stop'), 1)
                    self.assertEqual(robot.events.count('record_start'), 1)
                    op.close()

    def test_observe_recording_queries_only_and_preview_never_connects(self):
        with tempfile.TemporaryDirectory() as folder:
            robot = RecordingRobot()
            op, _ = runtime([sample(1), sample(2)], observe=True, robot=robot,
                            record_controller=True, diagnostics_path=Path(folder) / 'vr.jsonl')
            op.connect(); op.step(); op.close()
            self.assertIn('record_start', robot.events)
            self.assertFalse(robot.targets)
            self.assertNotIn('stop', robot.events)
        from scripts.teleoperate import main
        with patch('socket.socket', side_effect=AssertionError('preview opened a socket')):
            with redirect_stdout(io.StringIO()) as printed:
                code = main(['--config', str(ROOT / 'configs/a10/a10_dual_arm_vr_teleop.py'),
                             '--observe', '--record-controller', '--preview'])
        self.assertEqual(code, 0)
        self.assertIn('record_controller=True', printed.getvalue())

    def test_observe_runtime_and_recorder_share_real_loopback_ws(self):
        with tempfile.TemporaryDirectory() as folder, FakeController() as server:
            server.command_replies['record_start'] = reply()
            server.command_replies['record_heartbeat'] = reply()
            robot = server.operator(allow_motion=False, feedback_position_unit='mm')
            op, _ = runtime([sample(1), sample(2), sample(3, active=False)], robot=robot,
                            observe=True, record_controller=True,
                            diagnostics_path=Path(folder) / 'vr.jsonl')
            op.connect()
            for _ in range(3): op.step()
            op.close()
            self.assertEqual(server.commands.count('record_start'), 1)
            self.assertEqual(set(server.commands), {'get', 'record_start'})
            with self.assertRaises(socket.timeout): server.udp.recv(1024)

    def test_record_flag_requires_feedback_and_local_diagnostics(self):
        cfg = Config.fromfile(ROOT / 'configs/a10/a10_dual_arm_vr_teleop.py').teleop
        cfg.record_controller = True
        with self.assertRaisesRegex(ValueError, 'live/observe'): build_runtime(cfg)
        cfg.vr_only = False; cfg.observe_only = True; cfg.diagnostics_path = None
        with self.assertRaisesRegex(ValueError, 'diagnostics_path'): build_runtime(cfg)


if __name__ == '__main__':
    unittest.main()
