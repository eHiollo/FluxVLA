"""Timing protection uses synthetic robots only; recording cannot rearm motion."""
import unittest

import test_vr_dual_runtime as dual


class ControlGapProtectionTest(unittest.TestCase):
    def test_100ms_gap_stops_without_targets_and_requires_release_and_new_press(self):
        fixture = dual.DualSDKRuntimeTest('test_sdk_initializes_once_and_reclutch_never_resets_other_arm')
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        runtime, robot = fixture.runtime, fixture.robot
        self.assertEqual(runtime.reference_control.config.max_dt_s, .1)
        fixture.move()
        sent = len(robot.sdk_packets)
        starts = robot.events.count('start')
        fixture.clock.value += .11
        fixture.tick()
        self.assertEqual(runtime.input_pause_status['reason'], 'control_gap')
        self.assertTrue(runtime.input_pause_status['stopped'])
        self.assertEqual(len(robot.sdk_packets), sent)
        for _ in range(25): fixture.tick()
        self.assertEqual(robot.events.count('start'), starts)
        self.assertEqual(len(robot.sdk_packets), sent)
        fixture.wait_input_ready()
        fixture.tick(False, False)
        self.assertEqual(robot.events.count('start'), starts)
        fixture.tick(True, False)  # New press clears the latch, never replays a target.
        self.assertIsNone(runtime.input_pause_status)
        self.assertEqual(len(robot.sdk_packets), sent)
        fixture.tick(True, False)
        fixture.tick(True, False)  # Mapper arms first, then accepts a fresh sample.
        self.assertEqual(robot.events.count('start'), starts + 1)


if __name__ == '__main__':
    unittest.main()
