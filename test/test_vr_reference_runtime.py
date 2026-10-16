"""Stage 3 integration: synthetic VR/feedback and loopback WS/UDP only."""
import json
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np
from mmengine import Config

from fluxvla.engines.operators.vr.runtime import VRTeleopRuntime, build_runtime, inspect_controller
from test_a10_operator import A10ControllerError, FakeController
from test_vr_operator import FakeA10, FakeMapper, FakeVR

ROOT = Path(__file__).resolve().parents[1]


class Clock:
    value = 100.

    def now(self):
        return self.value

    def advance(self):
        self.value += 1. / 30.


class AnchorMapper(FakeMapper):
    position_control_mode = 'safe_frame_delta'
    motion_shaping_mode = 'robot_controller'
    compute_anchor_shadow = True


class StoppableA10(FakeA10):
    stop_timeout = False

    def wait_stopped(self):
        self.events.append('wait_stopped')
        if self.stop_timeout:
            raise TimeoutError('Reported Stopped was not confirmed')


def sample(sequence, *, active=True, offset=0., anchor=1, age=0.):
    command = dict(active=active, session_id='onsite-test', anchor_id=anchor,
                   sample_sequence=sequence, offset=[offset, 0., 0., 0., 0., 0.], gripper=0.)
    return {'xlevr.enabled': active, 'xlevr.sample_age_s': age,
            'mapped': {'ee.enabled': active, '_xlevr.anchor_command': command, 'gripper.pos': 0.}}


def runtime(rows, *, observe=False, robot=None, **kwargs):
    robot = robot or StoppableA10()
    robot.allow_motion = not observe
    return VRTeleopRuntime(FakeVR(rows), AnchorMapper(), robot,
                          execute=not observe, observe_only=observe, control_mode='reference',
                          max_position_step_m=.02, max_rotation_step_rad=.2, **kwargs), robot


class ReferenceRuntimeTest(unittest.TestCase):
    def test_live_start_discards_old_sample_and_release_confirms_stop(self):
        clock = Clock()
        operator, robot = runtime([sample(1, offset=.1), sample(2), sample(3, offset=.01), sample(4, active=False)])
        with patch('time.monotonic', clock.now):
            operator.connect()
            for _ in range(4):
                clock.advance()
                operator.step()
            self.assertEqual(len(robot.targets), 3)
            np.testing.assert_allclose(robot.targets[0]['right'], [0, 0, 0, 0, 0, 0, 1])
            self.assertGreater(robot.targets[-1]['right'][0], 0.)
            self.assertEqual(robot.events.count('stop'), 1)
            self.assertEqual(robot.events.count('wait_stopped'), 1)
            self.assertTrue(operator.last_stop_status['reported_stopped'])
            operator.close()
            self.assertEqual(robot.events.count('stop'), 1)

    def test_observe_never_starts_sends_stops_or_selects_coordinates(self):
        clock = Clock()
        operator, robot = runtime([sample(1), sample(2), sample(3, offset=.01), sample(4, active=False)], observe=True)
        with patch('time.monotonic', clock.now):
            operator.connect()
            for _ in range(4):
                clock.advance()
                operator.step()
            self.assertTrue(operator.last_reference_result['observe_only'])
            self.assertFalse(operator.last_reference_result['transmitted'])
            operator.close()
        self.assertFalse(robot.targets)
        self.assertTrue(all(event in ('connect', 'read', 'close') for event in robot.events), robot.events)

    def test_stale_input_requests_one_stop_and_does_not_restart(self):
        clock = Clock()
        operator, robot = runtime([sample(1), sample(2), sample(3, age=.3)])
        with patch('time.monotonic', clock.now):
            operator.connect()
            clock.advance(); operator.step()
            clock.advance(); operator.step()
            clock.advance()
            with self.assertRaisesRegex(RuntimeError, 'input_stale'):
                operator.step()
            operator.close()
        self.assertEqual(robot.events.count(('start', 'cartesian')), 1)
        self.assertEqual(robot.events.count('stop'), 1)
        self.assertEqual(len(robot.targets), 2)

    def test_stop_timeout_is_visible_and_not_retried_in_cleanup(self):
        clock = Clock()
        robot = StoppableA10()
        robot.stop_timeout = True
        operator, robot = runtime([sample(1), sample(2), sample(3, active=False)], robot=robot)
        with patch('time.monotonic', clock.now):
            operator.connect()
            clock.advance(); operator.step()
            clock.advance(); operator.step()
            clock.advance()
            with self.assertRaisesRegex(TimeoutError, 'Stopped'):
                operator.step()
            self.assertFalse(operator.last_stop_status['reported_stopped'])
            operator.close()
        self.assertEqual(robot.events.count('stop'), 1)

    def test_input_disconnect_stops_even_when_step_is_called_directly(self):
        clock = Clock()
        operator, robot = runtime([sample(1)])
        with patch('time.monotonic', clock.now):
            operator.connect()
            clock.advance(); operator.step()
            operator.vr.is_connected = False
            with self.assertRaises(ConnectionError):
                operator.step()
            operator.close()
        self.assertEqual(robot.events.count('stop'), 1)
        self.assertEqual(robot.events.count('wait_stopped'), 1)

    def test_invalid_start_sample_never_changes_controller_state(self):
        clock = Clock()
        for row in [sample(1, age=.3), sample(1)]:
            if row['xlevr.sample_age_s'] == 0.:
                row['mapped']['_xlevr.anchor_command']['offset'] = [float('nan')] * 6
            operator, robot = runtime([row])
            with patch('time.monotonic', clock.now):
                operator.connect()
                clock.advance()
                with self.assertRaises((ValueError, RuntimeError)):
                    operator.step()
                operator.close()
            self.assertNotIn('tool', robot.events)
            self.assertFalse(robot.targets)
            self.assertEqual(robot.events.count(('start', 'cartesian')), 0)

    def test_partial_start_failure_still_requests_and_confirms_stop(self):
        clock = Clock()
        operator, robot = runtime([sample(1)])
        def interrupted_start(mode):
            robot.follower_stop_pending = True
            raise TimeoutError('start reply lost')
        robot.start_follower = interrupted_start
        with patch('time.monotonic', clock.now):
            operator.connect()
            clock.advance()
            with self.assertRaisesRegex(TimeoutError, 'start reply lost'):
                operator.step()
            operator.close()
        self.assertEqual(robot.events.count('stop'), 1)
        self.assertEqual(robot.events.count('wait_stopped'), 1)
        self.assertFalse(robot.targets)

    def test_slow_feedback_ages_the_input_and_stops_before_next_target(self):
        clock = Clock()
        operator, robot = runtime([sample(1), sample(2), sample(3, offset=.01)])
        with patch('time.monotonic', clock.now):
            operator.connect()
            clock.advance(); operator.step()
            clock.advance(); operator.step()
            original = robot.get_frame
            def slow_frame():
                clock.value += .3
                return original()
            robot.get_frame = slow_frame
            clock.advance()
            with self.assertRaisesRegex(RuntimeError, 'control_gap'):
                operator.step()
            operator.close()
        self.assertEqual(len(robot.targets), 2)
        self.assertEqual(robot.events.count('stop'), 1)

    def test_new_anchor_while_engaged_requires_release_before_restart(self):
        clock = Clock()
        rows = [sample(1), sample(2), sample(3, anchor=2), sample(4, anchor=2),
                sample(5, anchor=2, active=False, age=.3), sample(6, anchor=2),
                sample(7, anchor=2, active=False), sample(8, anchor=3)]
        operator, robot = runtime(rows)
        with patch('time.monotonic', clock.now):
            operator.connect()
            for index in range(8):
                clock.advance(); operator.step()
                if index in (2, 3, 4, 5, 6):
                    self.assertEqual(robot.events.count(('start', 'cartesian')), 1)
            self.assertEqual(robot.events.count(('start', 'cartesian')), 2)
            np.testing.assert_allclose(robot.targets[-1]['right'], [0, 0, 0, 0, 0, 0, 1])
            operator.close()

    def test_double_frontend_shaping_and_observer_accessories_are_rejected(self):
        mapper = AnchorMapper()
        mapper.motion_shaping_mode = 'lerobot_a1'
        with self.assertRaisesRegex(ValueError, 'shaping bypass'):
            VRTeleopRuntime(FakeVR([]), mapper, control_mode='reference')
        cfg = Config.fromfile(str(ROOT / 'configs/a10/a10_vr_teleop.py')).teleop
        cfg.update(vr_only=False, observe_only=True, control_mode='reference')
        cfg.operator.feedback_position_unit = 'mm'
        cfg.operator.allow_motion = True  # Builder must force this OFF for observation.
        observer = build_runtime(cfg)
        self.assertFalse(observer.execute)
        self.assertFalse(observer.robot.allow_motion)
        self.assertFalse(observer.robot.is_connected)
        self.assertEqual(observer.robot._connection.ws_port, 5888)
        observer.close()
        cfg.operator.gripper = dict(port='/never/open')
        with self.assertRaisesRegex(ValueError, 'does not open'):
            build_runtime(cfg)

    def test_real_mapper_produces_valid_udp_packets_through_reference_chain(self):
        from fluxvla.engines.operators.vr.mapper import XLeVRDeltaEEMapper
        def raw(sequence, y, enabled=True):
            return {'xlevr.enabled': enabled, 'xlevr.grip_active': enabled,
                    'xlevr.target_position': [0., y, 0.], 'xlevr.orientation_quat': [0., 0., 0., 1.],
                    'xlevr.sample_sequence': sequence, 'xlevr.sample_receive_time_s': 10. + sequence / 30.,
                    'xlevr.sample_age_s': 0.}
        rows = [raw(1, 0.), raw(2, .001), raw(3, .002), raw(4, .006), raw(5, .006, False)]
        with FakeController() as server, tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'reference.jsonl'
            robot = server.operator(feedback_position_unit='mm')
            operator = VRTeleopRuntime(FakeVR(rows), XLeVRDeltaEEMapper(pos_scale=1., pos_deadzone_m=0.),
                                      robot, execute=True, control_mode='reference', diagnostics_path=str(path),
                                      max_position_step_m=.02, max_rotation_step_rad=.2)
            self.assertEqual(operator.run(max_steps=5), 5)
            poses = []
            for _ in range(3):
                packet = server.udp.recv(4096)
                self.assertEqual(struct.unpack('<IIQqqq', packet[:40]), (len(packet) - 40, 0, 0, 0, 0, 0))
                poses.append(json.loads(packet[40:])['pq'][0])
            np.testing.assert_allclose(poses[0], [0, 0, 0, 0, 0, 0, 1])
            self.assertGreater(poses[-1][0], 0.)
            self.assertEqual(server.commands.count('follower_cart'), 1)
            self.assertEqual(server.commands.count('stop_follower'), 1)
            records = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertTrue(records[-1]['stop_status']['reported_stopped'])
            self.assertIsNone(records[-1]['sent_relative_target'])
            self.assertFalse(server.errors)

    def test_real_mapper_stale_deactivation_cannot_hide_fault_or_restart_held_hand(self):
        from fluxvla.engines.operators.vr.mapper import XLeVRDeltaEEMapper
        clock = Clock()
        rows = []
        for seq in range(1, 7):
            rows.append({'xlevr.enabled': True, 'xlevr.grip_active': True,
                         'xlevr.target_position': [0., .001 * seq, 0.],
                         'xlevr.orientation_quat': [0., 0., 0., 1.],
                         'xlevr.sample_sequence': seq, 'xlevr.sample_receive_time_s': 10. + seq / 30.,
                         'xlevr.sample_age_s': .3 if seq == 5 else 0.})
        robot = StoppableA10()
        operator = VRTeleopRuntime(FakeVR(rows), XLeVRDeltaEEMapper(), robot,
                                  execute=True, control_mode='reference',
                                  max_position_step_m=.02, max_rotation_step_rad=.2)
        with patch('time.monotonic', clock.now):
            operator.connect()
            for _ in range(4):
                clock.advance(); operator.step()
            clock.advance()
            with self.assertRaisesRegex(RuntimeError, 'input_stale'):
                operator.step()
            clock.advance(); operator.step()
            self.assertEqual(operator.last_reference_result['state'], 'release_required')
            operator.close()
        self.assertEqual(robot.events.count(('start', 'cartesian')), 1)
        self.assertEqual(robot.events.count('stop'), 1)


class ReadOnlyProtocolTest(unittest.TestCase):
    def test_observe_cli_preview_does_not_connect_devices(self):
        result = subprocess.run([sys.executable, str(ROOT / 'scripts/teleoperate.py'),
                                 '--config', str(ROOT / 'configs/a10/a10_vr_teleop.py'),
                                 '--observe', '--preview', '--cfg-options',
                                 'teleop.operator.feedback_position_unit=mm',
                                 'teleop.operator.allow_motion=True'],
                                text=True, capture_output=True, timeout=90)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('observe_only=True', result.stdout)
        self.assertIn('control_mode=reference', result.stdout)

    def test_unreachable_monitor_is_a_failed_readonly_report(self):
        with patch('fluxvla.engines.operators.a10_operator.A10Operator.connect',
                   side_effect=ConnectionRefusedError('monitor unavailable')):
            report = inspect_controller(dict(host='127.0.0.1'), samples=1)
        self.assertFalse(report['query_success'])
        self.assertEqual(report['get_attempts'], 0)
        self.assertEqual(report['commands'], [])
        self.assertEqual(report['udp_targets_sent'], 0)
        self.assertIn('ConnectionRefusedError', report['error'])

    def test_probe_uses_only_monitor_get_and_no_udp_or_accessories(self):
        with FakeController() as server:
            report = inspect_controller(dict(host='127.0.0.1',
                status_ws_port=server.server.socket.getsockname()[1],
                udp_port=server.udp.getsockname()[1], joint_udp_port=server.joint_udp.getsockname()[1],
                timeout_s=.3, gripper=dict(port='/never/open'), cameras={'bad': 'device'}, allow_motion=True),
                samples=2, interval_s=.001)
            self.assertEqual(server.commands, ['get', 'get'])
            self.assertEqual(report['udp_targets_sent'], 0)
            self.assertFalse(report['controller_sample_age_verified'])
            self.assertIsNone(report['samples'][0]['tcp_pq_m'])
            self.assertEqual(report['samples'][0]['tcp_pq_raw'][:3], [100, 200, 300])
            with self.assertRaises(socket.timeout):
                server.udp.recv(4096)

    def test_wait_stopped_needs_both_inactivity_and_stopped_and_reports_fault(self):
        with FakeController() as server:
            robot = server.operator()
            server.motion = 'Moving'
            timer = threading.Timer(.03, lambda: setattr(server, 'motion', 'Stopped'))
            timer.start()
            try:
                robot.wait_stopped(timeout_s=.3)
            finally:
                timer.join()
            server.active = True
            with self.assertRaises(TimeoutError):
                robot.wait_stopped(timeout_s=.04)
            server.active = False
            server.driver_fault = True
            with self.assertRaises(A10ControllerError):
                robot.wait_stopped()
            self.assertTrue(all(command == 'get' for command in server.commands))


if __name__ == '__main__':
    unittest.main()
