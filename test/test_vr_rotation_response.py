"""Offline motion-following rotation: no hardware or transport connections."""
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from fluxvla.engines.operators.vr.control import ReferenceConfig, VRPoseController


IDENTITY = np.array([0., 0., 0., 0., 0., 0., 1.])
DT = 1. / 30.


def packet(sequence, angle=0., *, anchor=0, active=True, axis=(0., 0., 1.), x=0.):
    return dict(session_id='rotation-test', anchor_id=anchor,
                sample_sequence=sequence, active=active,
                offset=[x, 0., 0., *(np.asarray(axis) * angle)], gripper=0.)


def angular_distance(first, second):
    return float((Rotation.from_quat(first[3:]).inv()
                  * Rotation.from_quat(second[3:])).magnitude())


class IncrementalRotationTest(unittest.TestCase):
    def make(self, **options):
        defaults = dict(rotation_follow_mode='incremental_velocity',
                        max_angular_speed_rad_s=1.4, max_angular_accel_rad_s2=6.,
                        max_angular_brake_rad_s2=10., max_reference_lead_rad=.3)
        defaults.update(options)
        result = VRPoseController(ReferenceConfig(**defaults))
        result.start(IDENTITY)
        result.step(packet(0), IDENTITY, DT)
        return result

    def test_stop_hand_discards_backlog_even_while_clutch_is_held(self):
        control = self.make()
        actual = IDENTITY.copy()
        for i in range(1, 22):
            out = control.step(packet(i, .07 * i), actual, DT)
            actual = np.array(out['command_pose'])
        stopped_hand = 21 * .07
        before = actual.copy()
        speed = np.linalg.norm(control.angular_velocity)
        for i in range(22, 60):
            old_speed = np.linalg.norm(control.angular_velocity)
            out = control.step(packet(i, stopped_hand), actual, DT)
            self.assertLessEqual(old_speed - np.linalg.norm(control.angular_velocity), 10. * DT + 1e-10)
            actual = np.array(out['command_pose'])
        endpoint = actual.copy()
        self.assertLessEqual(angular_distance(before, endpoint), speed * speed / 20. + speed * DT)
        self.assertTrue(out['rotation_input_stationary'])
        self.assertGreater(out['rotation_backlog_discarded_rad'], .3)
        np.testing.assert_allclose(control.angular_velocity, 0., atol=1e-12)
        for i in range(60, 80):
            out = control.step(packet(i, stopped_hand), actual, DT)
            np.testing.assert_allclose(out['command_pose'], endpoint, atol=1e-12)
            np.testing.assert_allclose(out['reference_pose'][3:], endpoint[3:], atol=1e-12)

    def test_speed_acceleration_and_braking_bounds_and_full_speed(self):
        control = self.make()
        actual = IDENTITY.copy()
        previous = np.zeros(3)
        peak = 0.
        for i in range(1, 41):
            out = control.step(packet(i, .065 * i), actual, DT)
            w = np.asarray(out['angular_velocity_rad_s'])
            self.assertLessEqual(np.linalg.norm(w), 1.4 + 1e-10)
            self.assertLessEqual(np.linalg.norm(w - previous), 6. * DT + 1e-10)
            peak = max(peak, np.linalg.norm(w))
            actual, previous = np.asarray(out['command_pose']), w
        self.assertAlmostEqual(peak, 1.4)
        for i in range(41, 50):
            out = control.step(packet(i, .065 * 40), actual, DT)
            w = np.asarray(out['angular_velocity_rad_s'])
            self.assertLessEqual(np.linalg.norm(w - previous), 10. * DT + 1e-10)
            actual, previous = np.asarray(out['command_pose']), w
        np.testing.assert_allclose(previous, 0., atol=1e-12)

    def test_duplicate_stale_and_reanchor_do_not_reapply_input(self):
        control = self.make()
        out = control.step(packet(1, .02), IDENTITY, DT)
        actual = np.asarray(out['command_pose'])
        for command in (packet(1, .02), packet(0, .8)):
            out = control.step(command, actual, DT)
            np.testing.assert_allclose(out['rotation_input_base_rad_s'], 0.)
            actual = np.asarray(out['command_pose'])
        control.step_released(actual, DT)
        out = control.step(packet(3, 1.5, anchor=1), actual, DT,
                           continuous_reanchor=True)
        np.testing.assert_allclose(out['rotation_input_base_rad_s'], 0.)
        self.assertLessEqual(angular_distance(actual, out['command_pose']), 1.4 * DT + 1e-10)
        out = control.step(packet(4, 1.5, anchor=1), out['command_pose'], DT)
        np.testing.assert_allclose(out['rotation_input_base_rad_s'], 0., atol=1e-12)

    def test_rotated_start_preserves_local_axis_direction(self):
        start = np.r_[np.zeros(3), Rotation.from_euler('xyz', [.4, -.8, 1.1]).as_quat()]
        control = self.make()
        control.start(start)
        control.step(packet(0), start, DT)
        out = control.step(packet(1, .02, axis=(1., 0., 0.)), start, DT)
        expected_base = Rotation.from_quat(start[3:]).apply([.6, 0., 0.])
        np.testing.assert_allclose(out['rotation_input_base_rad_s'], expected_base, atol=1e-12)
        local_delta = (Rotation.from_quat(start[3:]).inv()
                       * Rotation.from_quat(out['command_pose'][3:])).as_rotvec()
        self.assertGreater(local_delta[0], 0.)
        np.testing.assert_allclose(local_delta[1:], 0., atol=1e-12)

    def test_translation_is_identical_to_original_anchor_mode(self):
        incremental = self.make()
        anchor = self.make(rotation_follow_mode='anchor')
        actual = IDENTITY.copy()
        for i in range(1, 35):
            command = packet(i, .03 * i, x=.002 * i)
            first = incremental.step(command, actual, DT)
            second = anchor.step(command, actual, DT)
            np.testing.assert_allclose(first['command_pose'][:3], second['command_pose'][:3], atol=1e-12)
            np.testing.assert_allclose(first['linear_velocity_m_s'], second['linear_velocity_m_s'], atol=1e-12)
            actual[:3] = first['command_pose'][:3]
            actual[3:] = first['command_pose'][3:]

    def test_soft_lead_and_hard_tracking_fault_remain_active(self):
        control = self.make(max_reference_lead_rad=.1)
        throttled = False
        for i in range(1, 50):
            out = control.step(packet(i, i * .03), IDENTITY, DT)
            throttled |= out['reference_progress_scale']['rotation'] < .5
            self.assertLess(angular_distance(IDENTITY, out['command_pose']), .35)
        self.assertTrue(throttled)
        far = np.r_[np.zeros(3), Rotation.from_euler('z', -1.).as_quat()]
        out = control.step(packet(50, 1.5), far, DT)
        self.assertEqual(out['state'], 'fault')
        self.assertEqual(out['fault_reason'], 'command_tracking_error')
        self.assertTrue(out['stop_requested'])

    def test_large_lead_and_jitter_with_fixed_feedback_preserve_hard_cap(self):
        # The old 0.3 rad budget must not become a larger implicit hard cap.
        for dt in (1. / 30., .066, .1):
            with self.subTest(dt=dt):
                control = self.make(max_reference_lead_rad=.3)
                maximum = 0.
                for i in range(1, 85):
                    out = control.step(packet(i, i * 3. * dt), IDENTITY, dt)
                    self.assertFalse(out['stop_requested'])
                    maximum = max(maximum, angular_distance(IDENTITY, out['command_pose']))
                self.assertLess(maximum, .35)
                self.assertLess(np.linalg.norm(control.angular_velocity), .001)

    def test_release_reuses_faster_rotation_braking(self):
        control = self.make()
        actual = IDENTITY.copy()
        for i in range(1, 12):
            out = control.step(packet(i, i * .06), actual, DT)
            actual = np.asarray(out['command_pose'])
        old_speed = np.linalg.norm(control.angular_velocity)
        out = control.step_released(actual, DT)
        self.assertAlmostEqual(old_speed - np.linalg.norm(out['angular_velocity_rad_s']), 10. * DT)
        self.assertTrue(out['rotation_input_stationary'])

    def test_deadzone_and_braking_fallback(self):
        control = self.make(rotation_velocity_deadzone_rad_s=.03,
                            max_angular_brake_rad_s2=None)
        out = control.step(packet(1, .0005), IDENTITY, DT)
        np.testing.assert_array_equal(out['angular_velocity_rad_s'], [0., 0., 0.])
        actual = np.asarray(out['command_pose'])
        for i in range(2, 12):
            out = control.step(packet(i, i * .03), actual, DT)
            actual = np.asarray(out['command_pose'])
        previous = control.angular_velocity.copy()
        out = control.step(packet(12, 11 * .03), actual, DT)
        self.assertLessEqual(np.linalg.norm(previous - out['angular_velocity_rad_s']), 6. * DT + 1e-10)

    def test_disabled_tracking_limits_log_large_errors_without_stopping(self):
        for mode in ('anchor', 'incremental_velocity'):
            with self.subTest(mode=mode):
                control = self.make(rotation_follow_mode=mode,
                                    enforce_tracking_error_limits=False,
                                    max_reference_lead_m=None, max_reference_lead_rad=None)
                previous_pose = IDENTITY.copy()
                previous_v = previous_w = np.zeros(3)
                exceeded = False
                for i in range(1, 100):
                    out = control.step(packet(i, .06 * i, x=.006 * i), IDENTITY, DT)
                    self.assertEqual(out['state'], 'tracking')
                    self.assertFalse(out['stop_requested'])
                    self.assertFalse(out['enforce_tracking_error_limits'])
                    exceeded |= out['tracking_limit_exceeded']
                    now = np.asarray(out['command_pose'])
                    v, w = np.asarray(out['linear_velocity_m_s']), np.asarray(out['angular_velocity_rad_s'])
                    self.assertLessEqual(np.linalg.norm(v), control.config.max_linear_speed_m_s + 1e-10)
                    self.assertLessEqual(np.linalg.norm(w), 1.4 + 1e-10)
                    self.assertLessEqual(np.linalg.norm(now[:3] - previous_pose[:3]), control.config.max_linear_speed_m_s * DT + 1e-10)
                    self.assertLessEqual(angular_distance(now, previous_pose), 1.4 * DT + 1e-10)
                    self.assertLessEqual(np.linalg.norm(v - previous_v), control.config.max_linear_accel_m_s2 * DT + 1e-10)
                    self.assertLessEqual(np.linalg.norm(w - previous_w), 10. * DT + 1e-10)
                    previous_pose, previous_v, previous_w = now, v, w
                self.assertTrue(exceeded)
                self.assertGreater(out['command_tracking_error_m'], .05)

    def test_disabled_tracking_release_only_confirms_frontend_hold(self):
        control = self.make(enforce_tracking_error_limits=False,
                            max_reference_lead_rad=None, release_timeout_s=.15)
        actual = IDENTITY.copy()
        for i in range(1, 12):
            out = control.step(packet(i, i * .06), actual, DT)
            actual = np.asarray(out['command_pose'])
        for i in range(1, 80):
            moving_actual = np.r_[[i * .002, 0., 0.], Rotation.from_euler('z', -.02 * i).as_quat()]
            out = control.step_released(moving_actual, DT)
            self.assertFalse(out['stop_requested'])
            self.assertNotEqual(out['state'], 'fault')
        self.assertEqual(out['state'], 'holding')
        self.assertTrue(out['command_stopped'])
        self.assertFalse(out['feedback_quiet'])
        self.assertTrue(out['tracking_limit_exceeded'])
        self.assertGreater(out['tracking_error_rad'], .35)

    def test_disabled_tracking_keeps_input_feedback_and_controller_faults(self):
        cases = [('controller_fault', dict(has_error=True), DT),
                 ('feedback_stale', dict(feedback_age_s=.3), DT),
                 ('control_gap', {}, .11),
                 ('input_stale', {}, DT)]
        for reason, options, dt in cases:
            with self.subTest(reason=reason):
                control = self.make(enforce_tracking_error_limits=False, max_input_age_s=.02)
                command = None if reason == 'input_stale' else packet(1, .02)
                out = control.step(command, IDENTITY, dt, **options)
                self.assertEqual(out['state'], 'fault')
                self.assertEqual(out['fault_reason'], reason)
                self.assertTrue(out['stop_requested'])
        control = self.make(enforce_tracking_error_limits=False)
        invalid = IDENTITY.copy()
        invalid[0] = np.nan
        with self.assertRaises(ValueError):
            control.step(packet(1), invalid, DT)

    def test_tracking_switch_defaults_and_lead_validation(self):
        self.assertTrue(ReferenceConfig().enforce_tracking_error_limits)
        with self.assertRaises(ValueError):
            ReferenceConfig(enforce_tracking_error_limits='false')
        with self.assertRaisesRegex(ValueError, 'tracking fault limit'):
            ReferenceConfig(max_reference_lead_rad=.4)
        cfg = ReferenceConfig(enforce_tracking_error_limits=False, max_reference_lead_rad=.4)
        self.assertEqual(cfg.max_reference_lead_rad, .4)

    def test_validation_and_existing_mapping_modes(self):
        self.assertEqual(ReferenceConfig().rotation_follow_mode, 'anchor')
        calibration = dict(vr_to_robot_quat_xyzw=[0., 0., 0., 1.],
                           hand_to_tool_quat_xyzw=[0., 0., 0., 1.])
        legacy = VRPoseController(ReferenceConfig(), pose_mapping='calibrated_absolute',
                                 pose_calibration=calibration)
        self.assertEqual(legacy.config.rotation_follow_mode, 'anchor')
        self.assertIsNone(legacy._result('inactive')['rotation_input_stationary'])
        with self.assertRaisesRegex(ValueError, 'anchor_relative'):
            VRPoseController(ReferenceConfig(rotation_follow_mode='incremental_velocity'),
                             pose_mapping='calibrated_absolute', pose_calibration=calibration)
        for options in (dict(rotation_follow_mode='other'),
                        dict(rotation_follow_mode='incremental_velocity', limit_acceleration=False),
                        dict(rotation_velocity_deadzone_rad_s=-1.),
                        dict(rotation_velocity_deadzone_rad_s=float('nan')),
                        dict(max_angular_brake_rad_s2=0.)):
            with self.subTest(options=options), self.assertRaises(ValueError):
                ReferenceConfig(**options)


class CachedReleaseFeedbackTest(unittest.TestCase):
    def make(self):
        control = VRPoseController(ReferenceConfig())
        control.start(IDENTITY)
        return control

    def test_repeated_cached_pose_cannot_confirm_stopped(self):
        control = self.make()
        for _ in range(20):
            result = control.step_released(IDENTITY, 1/90, feedback_time_s=10.)
        self.assertEqual(result['state'], 'releasing')
        self.assertEqual(result['release_stable_s'], 0.)
        self.assertFalse(result['feedback_updated'])
        for index in range(1, 5):
            result = control.step_released(IDENTITY, 1/90, feedback_time_s=10.+index/30)
        self.assertEqual(result['state'], 'holding')

    def test_speed_uses_feedback_interval_not_control_interval(self):
        control = self.make()
        control.step_released(IDENTITY, 1/90, feedback_time_s=10.)
        actual = IDENTITY.copy()
        actual[0] = .00025  # 7.5 mm/s over 1/30 s, below quietness threshold.
        result = control.step_released(actual, 1/90, feedback_time_s=10.+1/30)
        self.assertTrue(result['feedback_quiet'])
        self.assertAlmostEqual(result['feedback_interval_s'], 1/30)
        result = control.step_released(actual, 1/90, feedback_time_s=9.)
        self.assertTrue(result['stop_requested'])
        self.assertEqual(result['fault_reason'], 'feedback_timestamp_regression')


if __name__ == '__main__':
    unittest.main()
