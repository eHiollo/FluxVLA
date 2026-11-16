"""Stale robot feedback pauses dual VR without physical devices or wall sleeps."""
import copy
import time
import unittest
from unittest.mock import patch

import numpy as np

from test_vr_dual_runtime import (
    DualRobot, DualVRTeleopRuntime, HandInput, VROperator, configuration, hand,
)
from test_vr_reference_runtime import Clock


class MemoryFeedbackRobot(DualRobot):
    """The consumer can keep an old snapshot while direct stop queries succeed."""

    def __init__(self):
        super().__init__()
        self.cache = None
        self.sequence = 0
        self.motion = 'Stopped'
        self.activation = 'Enabled'

    def get_frame(self):
        self.events.append('fresh_get')
        result = super().get_frame()
        result.update(motion=self.motion, activation=self.activation)
        return result

    def start_feedback_polling(self, hz):
        self.events.append(('poll', hz))
        self.publish()

    def publish(self):
        self.sequence += 1
        self.cache = self.get_frame()
        self.cache['controller_feedback_sequence'] = self.sequence

    def get_cached_frame(self):
        self.events.append('memory_get')
        return copy.deepcopy(self.cache)

    def expire(self, *, arm=None):
        if arm is None:
            self.cache['received_monotonic_s'] = time.monotonic() - .4
        else:
            self.cache['arms'][arm]['received_monotonic_s'] = time.monotonic() - .4


class FeedbackRecoveryTest(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.clock.value = 100.
        self.clock_patch = patch('time.monotonic', self.clock.now)
        self.clock_patch.start()
        self.addCleanup(self.clock_patch.stop)
        cfg = configuration(identity_bindings=True)
        self.vr = VROperator(**{k: v for k, v in cfg.input.items() if k != 'type'})
        self.vr.input = HandInput()
        self.robot = MemoryFeedbackRobot()
        self.runtime = DualVRTeleopRuntime(
            self.vr, self.robot, execute=True, control_mode='reference',
            cartesian_adapter='legacy_anchor', feedback_poll_hz=30,
            max_position_step_m=.005, max_rotation_step_rad=.015)
        self.runtime.connect()
        self.addCleanup(self.runtime.close)
        self.seq = 0

    def tick(self, pressed=False, *, publish=True, x=0.):
        self.clock.value += 1./90.
        self.seq += 1
        if publish:
            self.robot.publish()
        self.vr.input.rows = {side: hand(self.seq, pressed, x=x)
                              for side in ('left', 'right')}
        return self.runtime.step()

    def move(self):
        self.tick(False)
        for index in range(8):
            self.tick(True, x=index * .0001)
        self.assertTrue(self.runtime._started_follower)
        self.assertTrue(self.robot.targets)

    def stale_pause(self, *, arm=None, moving=True):
        if moving:
            self.move()
        self.robot.expire(arm=arm)
        before = len(self.robot.targets)
        self.tick(moving, publish=False)
        self.assertEqual(self.runtime.input_pause_status['reason'], 'feedback_stale')
        self.assertEqual(len(self.robot.targets), before)
        self.assertTrue(self.runtime._connected)
        self.assertTrue(self.vr.input.is_connected)
        return before

    def ready(self):
        for _ in range(65):
            self.tick(False)
        self.assertEqual(self.runtime.input_pause_status['state'], 'ready')

    def test_idle_stale_keeps_ui_and_confirms_stop_without_starting_motion(self):
        self.stale_pause(moving=False)
        self.assertNotIn('start', self.robot.events)
        self.assertNotIn('stop', self.robot.events)
        self.assertEqual(self.robot.events.count('wait_stopped'), 1)
        self.assertTrue(self.vr.input.hud)
        self.assertEqual(self.vr.input.hud[-1]['phase'], 'input_paused')
        self.assertEqual(self.runtime.input_pause_status['state'], 'feedback_wait')

    def test_stale_active_feedback_stops_once_without_target_replay(self):
        count = self.stale_pause()
        self.assertEqual(self.robot.events.count('stop'), 1)
        self.assertEqual(self.robot.events.count('wait_stopped'), 1)
        # Old cache still says active. Do not mistake a pre-stop snapshot for
        # a newly restarted follower, and do not repeatedly send stop commands.
        self.assertTrue(self.robot.cache['follower_active'])
        for _ in range(45):
            self.tick(False, publish=False)
        self.assertEqual(self.robot.events.count('stop'), 1)
        self.assertEqual(self.robot.events.count('wait_stopped'), 1)
        self.assertEqual(self.robot.events.count('start'), 1)
        self.assertEqual(len(self.robot.targets), count)
        self.assertEqual(self.runtime.input_pause_status['state'], 'feedback_wait')

    def test_single_arm_timestamp_expiry_pauses_whole_target_group(self):
        self.stale_pause(arm='right')
        self.assertEqual(self.robot.events.count('stop'), 1)

    def test_held_grips_never_auto_restart_then_new_press_reanchors_actual_pose(self):
        count = self.stale_pause()
        for _ in range(65):
            self.tick(True)
        self.assertNotEqual(self.runtime.input_pause_status['state'], 'ready')
        self.assertEqual(len(self.robot.targets), count)
        self.assertEqual(self.robot.events.count('start'), 1)
        self.ready()
        self.robot.poses['left'][:3] += [.01, .02, 0.]
        self.robot.poses['right'][:3] += [-.01, 0., .02]
        actual = copy.deepcopy(self.robot.poses)
        self.tick(True)  # New grip edge clears pause, never replays an old target.
        self.assertIsNone(self.runtime.input_pause_status)
        self.assertEqual(len(self.robot.targets), count)
        self.tick(True)
        self.tick(True)
        self.assertEqual(self.robot.events.count('start'), 2)
        for arm in actual:
            np.testing.assert_allclose(self.robot.anchors[arm], actual[arm])

    def test_duplicate_feedback_cannot_complete_stability_gate(self):
        self.stale_pause()
        # Give the same feedback enough age budget to isolate the stability
        # gate from simple expiry. Fresh VR samples alone are insufficient.
        self.runtime.max_feedback_age_s = 1.
        self.tick(False)
        for _ in range(60):
            self.tick(False, publish=False)
        self.assertNotEqual(self.runtime.input_pause_status['state'], 'ready')
        self.assertEqual(self.robot.events.count('start'), 1)
        self.ready()

    def test_ready_accepts_new_press_between_feedback_ticks(self):
        count = self.stale_pause()
        self.ready()
        self.tick(True, publish=False)  # A 90Hz input edge between 30Hz GETs.
        self.assertIsNone(self.runtime.input_pause_status)
        self.assertEqual(len(self.robot.targets), count)

    def test_recent_feedback_that_reports_motion_cannot_unlock(self):
        self.stale_pause()
        self.robot.motion = 'Running'
        for _ in range(65):
            self.tick(False)
        self.assertNotEqual(self.runtime.input_pause_status['state'], 'ready')
        self.assertEqual(self.robot.events.count('start'), 1)

    def test_disabled_motors_cannot_unlock_or_auto_enable(self):
        count = self.stale_pause()
        self.robot.activation = 'Disabled'
        for _ in range(65):
            self.tick(False)
        self.tick(True)
        self.assertNotEqual(self.runtime.input_pause_status['state'], 'ready')
        self.assertEqual(self.robot.activation, 'Disabled')
        self.assertEqual(self.robot.events.count('start'), 1)
        self.assertEqual(len(self.robot.targets), count)
        # Simulate an explicit external enable, then require stability and a
        # new press; the recovery routine itself never enables the motors.
        self.robot.activation = 'Enabled'
        self.ready()
        self.assertEqual(self.robot.events.count('start'), 1)
        self.assertEqual(len(self.robot.targets), count)

    def test_unconfirmed_stop_retains_hud_and_permanently_blocks_session_restart(self):
        self.move()
        self.robot.stop_timeout = True
        self.robot.expire()
        self.tick(True, publish=False)
        self.assertTrue(self.runtime._connected)
        self.assertEqual(self.runtime.input_pause_status['state'], 'stop_unconfirmed')
        count = len(self.robot.targets)
        self.robot.stop_timeout = False
        for _ in range(65):
            self.tick(False)
        self.tick(True)
        self.assertEqual(self.runtime.input_pause_status['state'], 'stop_unconfirmed')
        self.assertEqual(self.robot.events.count('start'), 1)
        self.assertEqual(self.robot.events.count('stop'), 1)
        self.assertEqual(len(self.robot.targets), count)

    def test_true_controller_fault_is_not_reclassified_as_recoverable_staleness(self):
        self.stale_pause()
        self.robot.frame_fault = True
        with self.assertRaisesRegex(RuntimeError, 'fault'):
            self.tick(False)
        self.assertEqual(self.robot.events.count('start'), 1)

    def test_invalid_global_or_arm_timestamps_are_not_recoverable_staleness(self):
        for arm in (None, 'left', 'right'):
            for stamp in (float('nan'), float('inf'), -1., True, self.clock.now() + 1.):
                with self.subTest(arm=arm, stamp=stamp):
                    self.robot.publish()
                    target = self.robot.cache if arm is None else self.robot.cache['arms'][arm]
                    target['received_monotonic_s'] = stamp
                    with self.assertRaisesRegex(RuntimeError, 'timestamp is invalid'):
                        self.runtime._checked_frame(require_active=False)
                    self.assertIsNone(self.runtime.input_pause_status)

    def test_future_dated_frame_during_motion_does_not_enter_recovery_path(self):
        self.move()
        count = len(self.robot.targets)
        self.robot.cache['received_monotonic_s'] = self.clock.now() + 1.
        with self.assertRaisesRegex(RuntimeError, 'timestamp is invalid'):
            self.tick(True, publish=False)
        self.assertIsNone(self.runtime.input_pause_status)
        self.assertEqual(len(self.robot.targets), count)
        self.assertEqual(self.robot.events.count('stop'), 1)


if __name__ == '__main__':
    unittest.main()
