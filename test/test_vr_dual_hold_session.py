"""Dual hold/re-clutch and query failure handling; synthetic devices only."""
import json
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

import test_vr_dual_runtime as dual
from fluxvla.engines.operators.vr.control import ReferenceConfig, VRPoseController


class HoldSessionTest(unittest.TestCase):
    robot_type = dual.DualSDKRobot
    cartesian_adapter = 'sdk_tool_cumulative'
    setUp = dual.DualRuntimeTest.setUp
    tick = dual.DualRuntimeTest.tick
    move = dual.DualRuntimeTest.move

    def test_delayed_feedback_stops_without_exit_and_requires_fresh_reclutch(self):
        self.move()
        sent = len(self.robot.targets)
        get_frame = self.robot.get_frame
        delayed = False

        def slow_query_once():
            nonlocal delayed
            if not delayed:
                delayed = True
                self.clock.value += .24  # Successful, but much too late for this control tick.
            return get_frame()

        with patch.object(self.robot, 'get_frame', side_effect=slow_query_once):
            self.tick()
        pause = self.runtime.input_pause_status
        self.assertEqual(pause['reason'], 'control_gap')
        self.assertTrue(pause['stopped'])
        self.assertGreater(pause['control_gap_s'], .24)
        self.assertAlmostEqual(pause['io_timing_s']['robot_feedback_s'], .24)
        self.assertEqual(pause['max_control_gap_s'], .1)
        self.assertEqual(len(self.robot.targets), sent)
        self.assertEqual(self.robot.events.count('stop'), 1)
        self.assertTrue(self.vr.is_connected)
        self.assertIn('CONTROL LOOP DELAY', self.vr.input.hud[-1]['hint'])
        for _ in range(25):
            self.tick()  # Holding the old side keys cannot restart motion.
        self.assertEqual(self.robot.events.count('start'), 1)
        for _ in range(21):
            self.tick(False, False)
        self.assertEqual(self.runtime.input_pause_status['state'], 'ready')
        for _ in range(3):
            self.tick(True, False)
        self.assertIsNone(self.runtime.input_pause_status)
        self.assertEqual(self.robot.events.count('start'), 2)

    def test_control_gap_stop_failure_remains_fatal(self):
        self.move()
        sent = len(self.robot.targets)
        self.robot.stop_timeout = True
        self.clock.value += .12
        with self.assertRaisesRegex(TimeoutError, 'stop unconfirmed'):
            self.tick()
        self.assertTrue(self.runtime._dual_stop_failed)
        self.assertEqual(len(self.robot.targets), sent)
        self.assertFalse(self.runtime.input_pause_status['stopped'])

    def test_control_gap_during_read_only_observation_cannot_claim_a_stop(self):
        self.move()
        self.runtime.execute = False
        self.runtime.observe_only = True
        self.clock.value += .12
        with self.assertRaisesRegex(RuntimeError, 'control_gap'):
            self.tick()
        self.assertIsNone(self.runtime.input_pause_status)
        self.assertNotIn('stop', self.robot.events)

    def test_both_release_holds_without_stop_queries_and_reclutches_same_follower(self):
        self.runtime.stop_on_both_release = False
        self.move()
        for _ in range(30):
            self.tick(False, False)
        self.assertEqual(self.runtime.last_reference_result['state'], 'holding')
        self.assertTrue(all(c['state'] == 'holding' for c in self.runtime._arms.values()))
        self.assertNotIn('stop', self.robot.events)
        self.assertNotIn('wait_stopped', self.robot.events)
        for i in range(10):
            self.tick(True, False, .007 + i * .0003, .004)
        self.assertEqual(self.robot.events.count('start'), 1)
        self.assertEqual(self.robot.events.count('begin_sdk'), 1)
        self.assertEqual(self.runtime._arms['left']['state'], 'tracking')
        self.assertEqual(self.runtime._arms['right']['state'], 'holding')
        self.runtime.close()
        self.assertEqual(self.robot.events.count('stop'), 1)
        self.assertEqual(self.robot.events.count('wait_stopped'), 1)

    def fail_query(self, command='get'):
        self.robot.last_communication_failure = dict(command=command, failed_wall_ns=time.time_ns(),
                                                      events=[], error='synthetic timeout')
        raise TimeoutError(f"A10 command '{command}' reply timed out after 2.000s")

    def test_query_failure_keeps_vr_and_jsonl_without_more_queries_targets_or_restart(self):
        self.runtime.stop_on_both_release = False
        self.runtime.keep_session_on_query_error = True
        self.move()
        sent = len(self.robot.targets)
        from fluxvla.engines.operators.vr.diagnostics import XLeVRDiagnosticsWriter
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'diagnostics.jsonl'
            self.runtime._diagnostics = XLeVRDiagnosticsWriter(path)
            with patch.object(self.robot, 'get_frame', side_effect=self.fail_query) as query, \
                    patch.object(self.robot, 'stop_follower', side_effect=ConnectionError('closed')) as stop:
                self.tick()
                for _ in range(10):
                    self.tick()  # Still-held keys cannot resume an uncertain session.
                self.assertEqual(query.call_count, 1)
                self.assertEqual(stop.call_count, 1)
                self.assertTrue(self.runtime._connected)
                self.assertTrue(self.vr.is_connected)
                self.assertEqual(len(self.robot.targets), sent)
                self.assertFalse(self.runtime.communication_pause['stop_confirmed'])
                self.assertEqual(self.runtime.last_reference_result['state'], 'communication_paused')
                self.assertEqual(self.robot.events.count('start'), 1)
            self.runtime._diagnostics.close()
            self.runtime._diagnostics = None
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertTrue(rows[-1]['communication_pause']['restart_required'])
            self.assertFalse(rows[-1]['reference_control']['transmitted'])
            self.assertEqual(rows[-1]['communication_failure']['command'], 'get')

    def test_recorder_heartbeat_query_failure_also_retains_session(self):
        self.runtime.keep_session_on_query_error = True
        self.move()
        self.runtime.record_controller = True
        self.runtime._recording_next_heartbeat = 0.
        with patch.object(self.robot, 'heartbeat_runtime_recording', create=True,
                          side_effect=lambda: self.fail_query('record_heartbeat')) as heartbeat:
            self.tick()
            for _ in range(4): self.tick()
        self.assertEqual(heartbeat.call_count, 1)
        self.assertIsNotNone(self.runtime.communication_pause)
        self.assertIsNone(self.runtime._recording_next_heartbeat)
        self.assertTrue(self.vr.is_connected)

    def test_controller_fault_still_requests_stop_and_is_not_treated_as_query_timeout(self):
        self.runtime.keep_session_on_query_error = True
        self.move()
        self.robot.last_communication_failure = dict(command='get', failed_wall_ns=1)
        self.robot.frame_fault = True
        with self.assertRaisesRegex(RuntimeError, 'fault'):
            self.tick()
        self.assertIsNone(self.runtime.communication_pause)
        self.assertEqual(self.robot.events.count('stop'), 1)

    def test_diagnostic_tracking_policy_covers_runtime_guard_and_complete_sdk_packets(self):
        cfg = ReferenceConfig(enforce_tracking_error_limits=False,
                              rotation_follow_mode='incremental_velocity',
                              max_linear_speed_m_s=.17, max_linear_accel_m_s2=1.,
                              max_angular_speed_rad_s=1.4, max_angular_accel_rad_s2=6.,
                              max_angular_brake_rad_s2=10.)
        self.runtime.reference_control = VRPoseController(cfg)
        for context in self.runtime._arms.values():
            context['control'] = VRPoseController(cfg)
        self.runtime.stop_on_both_release = False
        self.robot.follow = False  # Fresh feedback stays far behind the command.
        for i in range(200):
            self.tick(True, False, .001 * i, 0., angle=.006 * i)
        result = self.runtime.last_reference_result['arms']['left']
        self.assertTrue(result['tracking_limit_exceeded'])
        self.assertGreater(result['command_tracking_error_m'], .05)
        self.assertGreater(result['command_tracking_error_rad'], .35)
        self.assertIsNotNone(self.runtime.last_tracking_warning)
        self.assertNotIn('stop', self.robot.events)
        for previous, current in zip(self.robot.targets, self.robot.targets[1:]):
            self.assertEqual(set(current), {'left', 'right'})
            for name in current:
                self.assertLessEqual(np.linalg.norm(current[name][:3] - previous[name][:3]), .17 / 30 + 1e-10)
                angle = (Rotation.from_quat(previous[name][3:]).inv()
                         * Rotation.from_quat(current[name][3:])).magnitude()
                self.assertLessEqual(angle, 1.4 / 30 + 1e-10)

    def test_short_release_then_press_keeps_incremental_rotation_continuous(self):
        cfg = ReferenceConfig(enforce_tracking_error_limits=False,
                              rotation_follow_mode='incremental_velocity',
                              max_angular_speed_rad_s=1.4, max_angular_accel_rad_s2=6.,
                              max_angular_brake_rad_s2=10.)
        self.runtime.reference_control = VRPoseController(cfg)
        for context in self.runtime._arms.values():
            context['control'] = VRPoseController(cfg)
        self.runtime.stop_on_both_release = False
        for i in range(24):
            self.tick(True, False, angle=.035 * i)
        self.assertGreater(np.linalg.norm(self.runtime._arms['left']['control'].angular_velocity), .8)
        for cycle in range(4):
            self.tick(False, False, angle=.035 * 23)
            for i in range(6):
                self.tick(True, False, angle=.035 * 23 + .02 * i)
            self.assertEqual(self.runtime._arms['left']['state'], 'tracking')
            self.assertFalse(self.runtime.last_reference_result['arms']['left']['reengage_requires_release'])
        self.assertEqual(self.robot.events.count('start'), 1)
        self.assertEqual(self.robot.events.count('begin_sdk'), 1)
        self.assertNotIn('stop', self.robot.events)
        for previous, current in zip(self.robot.targets, self.robot.targets[1:]):
            angle = (Rotation.from_quat(previous['left'][3:]).inv()
                     * Rotation.from_quat(current['left'][3:])).magnitude()
            self.assertLessEqual(angle, 1.4 / 30 + 1e-10)

    def test_stream_loss_in_holding_stops_repeating_old_sdk_targets(self):
        # Reproduce the incident even if a legacy profile leaves HOLD enabled.
        self.runtime.stop_on_both_release = False
        self.move()
        for _ in range(20): self.tick(False, False)
        self.assertEqual(self.runtime.last_reference_result['state'], 'holding')
        count = len(self.robot.targets)
        for sample in self.vr.input.rows.values():
            sample.update({'xlevr.enabled': False, 'xlevr.grip_active': False,
                           'xlevr.verification_tracking': False, 'xlevr.sample_age_s': 46.7})
        self.clock.advance()
        self.runtime.step()
        self.assertEqual(len(self.robot.targets), count)
        self.assertEqual(self.runtime.input_pause_status['reason'], 'vr_stream_stale')
        self.assertTrue(self.runtime.input_pause_status['stopped'])
        self.assertEqual(self.robot.events.count('stop'), 1)
        self.assertEqual(self.robot.events.count('wait_stopped'), 1)
        self.assertFalse(self.robot.follower_stop_pending)
        for _ in range(10):
            self.clock.advance()
            self.runtime.step()  # Re-reading the old snapshot must never replay it.
        self.assertEqual(len(self.robot.targets), count)
        self.assertEqual(self.robot.events.count('start'), 1)

    def test_normal_both_release_confirms_stop_then_new_press_reclutches_directly(self):
        self.runtime.stop_on_both_release = True
        self.move()
        sent = len(self.robot.targets)
        self.tick(False, False)
        self.assertEqual(len(self.robot.targets), sent)
        self.assertEqual(self.runtime.last_reference_result['state'], 'stopped')
        self.assertFalse(self.robot.follower_stop_pending)
        self.assertFalse(self.runtime._dual_require_release)
        # No extra released sample between confirmed stop and the next press.
        for _ in range(5): self.tick(True, False)
        self.assertEqual(self.robot.events.count('start'), 2)
        self.assertEqual(self.runtime._arms['left']['state'], 'tracking')
        self.assertFalse(self.runtime.last_reference_result['arms']['left']['reengage_requires_release'])

    def test_missing_stream_before_first_clutch_never_starts_robot(self):
        self.tick(False, False, age=1.)
        for sample in self.vr.input.rows.values():
            sample['xlevr.sample_age_s'] = 1.
        self.clock.advance()
        self.runtime.step()
        self.assertEqual(self.runtime.last_reference_result['state'], 'idle')
        self.assertEqual(self.robot.targets, [])
        self.assertNotIn('start', self.robot.events)
        self.assertNotIn('stop', self.robot.events)

    def test_release_still_stops_if_unused_controller_has_no_fresh_sample(self):
        self.runtime.stop_on_both_release = True
        self.move(left=False, right=True)
        sent = len(self.robot.targets)
        self.tick(False, False, age=2.)  # Only the unused left controller is stale.
        self.assertEqual(len(self.robot.targets), sent)
        self.assertEqual(self.runtime.last_reference_result['state'], 'stopped')
        self.assertEqual(self.robot.events.count('stop'), 1)
        self.assertFalse(self.robot.follower_stop_pending)


if __name__ == '__main__':
    unittest.main()
