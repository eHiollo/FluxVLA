"""Initialization planning and cancellation, without ROS or hardware."""

import unittest
from unittest.mock import patch

import numpy as np

from fluxvla.engines.operators.base_operator import BaseOperator
from fluxvla.engines.operators.kaanh_commands.m_init import (
    configure_init_motion, move_to_init, plan_init_motion)


class Clock:
    now = 0.

    def monotonic(self):
        return self.now


class Event:
    def __init__(self, clock):
        self.clock, self.cancelled = clock, False

    def clear(self):
        self.cancelled = False

    def is_set(self):
        return self.cancelled

    def set(self):
        self.cancelled = True

    def wait(self, dt):
        self.clock.now += dt
        return self.cancelled


class MemoryOperator(BaseOperator):
    command_mode = 'cartesian'

    def __init__(self, dofs, *, follow=True):
        super().__init__()
        self.clock = Clock()
        self._traj_stop_event = Event(self.clock)
        configure_init_motion(self, joint_dofs=dofs,
                                   init_position={n: np.full(d, .1) for n, d in dofs.items()})
        self.actual = {n: np.zeros(d) for n, d in dofs.items()}
        self.follow, self.follower_stop_pending = follow, False
        self.calls, self.rows, self.reads = [], [], 0
        self.failure = None

    def _claim_init_motion(self):
        pass

    def _release_init_motion(self):
        pass

    def _prepare_init_motion(self):
        self.calls.append('prepare')

    def _read_init_joints(self):
        self.reads += 1
        return self.actual

    def start_follower(self, mode=None):
        self.calls.append(('start', mode))
        self.command_mode = mode
        self.follower_stop_pending = True

    def stop_follower(self):
        self.calls.append('stop')
        self.follower_stop_pending = False

    def send_joints(self, targets):
        self.rows.append({n: q.copy() for n, q in targets.items()})
        if self.failure:
            raise self.failure
        if self.follow:
            self.actual = targets


class InitMotionTest(unittest.TestCase):
    def test_synchronized_planning_dimensions_endpoints_and_limits(self):
        for dofs in ({'right': 6}, {'right': 6, 'left': 6},
                     {'right': 7, 'left': 7}, {'right': 6, 'left': 7}):
            start = {n: np.linspace(-.3, .2, d) for n, d in dofs.items()}
            target = {n: np.linspace(.1, .8, d) for n, d in dofs.items()}
            duration, rows = plan_init_motion(start, target, dofs, publish_hz=50.,
                max_velocity_rad_s=.1, max_acceleration_rad_s2=.2)
            rows = list(rows)
            self.assertAlmostEqual(duration, (len(rows) - 1) / 50.)
            for n, d in dofs.items():
                q = np.array([row[n] for row in rows])
                self.assertEqual(q.shape[1], d)
                np.testing.assert_array_equal(q[0], start[n])
                np.testing.assert_array_equal(q[-1], target[n])
                self.assertLessEqual(np.max(np.abs(np.diff(q, axis=0))) * 50., .100001)
                self.assertLessEqual(np.max(np.abs(np.diff(q, n=2, axis=0))) * 2500., .200001)

    def test_arrival_and_gripper_placeholder_for_six_and_seven_axes(self):
        for dofs in ({'right': 6}, {'right': 6, 'left': 6}, {'right': 7, 'left': 7}):
            op = MemoryOperator(dofs)
            with patch('fluxvla.engines.operators.kaanh_commands.m_init.time.monotonic', op.clock.monotonic):
                result = move_to_init(op)
            self.assertTrue(result['completed'])
            self.assertFalse(result['grippers_initialized'])
            self.assertEqual(op.init_gripper_positions, {n: 0. for n in dofs})
            self.assertEqual(op.calls, ['prepare', ('start', 'joint'), 'stop'])
            self.assertEqual(op.command_mode, 'cartesian')
            self.assertGreaterEqual(op.reads, 5)

    def test_sent_targets_cannot_substitute_for_actual_arrival(self):
        op = MemoryOperator({'right': 6}, follow=False)
        with patch('fluxvla.engines.operators.kaanh_commands.m_init.time.monotonic', op.clock.monotonic):
            with self.assertRaises(TimeoutError):
                move_to_init(op)
        self.assertFalse(op.last_init_motion['arms_arrived'])
        self.assertTrue(op.last_init_motion['follower_stopped'])
        self.assertEqual(op.calls[-1], 'stop')

    def test_failure_interrupt_and_cancellation_stop_the_follower(self):
        for failure in (RuntimeError('send failed'), KeyboardInterrupt(), InterruptedError('cancelled')):
            op = MemoryOperator({'right': 6})
            op.failure = failure
            with patch('fluxvla.engines.operators.kaanh_commands.m_init.time.monotonic', op.clock.monotonic):
                with self.assertRaises(type(failure)):
                    move_to_init(op)
            self.assertEqual(op.calls[-1], 'stop')
            self.assertFalse(op.last_init_motion['completed'])
            self.assertFalse(op._init_motion_lock.locked())

    def test_invalid_targets_have_no_side_effects(self):
        for target in ([0] * 7, [np.nan] * 6, [False] * 6, [False, 0, 0, 0, 0, 0], ['0'] * 6):
            op = MemoryOperator({'right': 6})
            op.init_position['right'] = target
            with self.assertRaises(ValueError):
                move_to_init(op)
            self.assertEqual(op.calls, [])

    def test_gripper_extension_receives_named_zero_widths_after_arm_arrival(self):
        op = MemoryOperator({'right': 7, 'left': 7})
        with patch('fluxvla.engines.operators.kaanh_commands.m_init.initialize_grippers', return_value=True) as grippers:
            with patch('fluxvla.engines.operators.kaanh_commands.m_init.time.monotonic', op.clock.monotonic):
                result = move_to_init(op)
        grippers.assert_called_once_with(op, {'right': 0., 'left': 0.})
        self.assertTrue(result['grippers_initialized'])

    def test_stop_event_cancels_further_targets_and_releases_lock(self):
        op = MemoryOperator({'right': 6})
        original = op.send_joints
        def send(targets):
            original(targets)
            op._traj_stop_event.set()
        with patch.object(op, 'send_joints', side_effect=send):
            with patch('fluxvla.engines.operators.kaanh_commands.m_init.time.monotonic', op.clock.monotonic):
                with self.assertRaises(InterruptedError):
                    move_to_init(op)
        self.assertEqual(len(op.rows), 1)
        self.assertEqual(op.calls[-1], 'stop')
        self.assertFalse(op._init_motion_lock.locked())

    def test_late_arrival_feedback_cannot_report_success_after_deadline(self):
        op = MemoryOperator({'right': 6})
        op.init_position = {'right': np.zeros(6)}
        original = op._read_init_joints
        def read():
            if op.rows:
                op.clock.now += 1000.
            return original()
        with patch.object(op, '_read_init_joints', side_effect=read):
            with patch('fluxvla.engines.operators.kaanh_commands.m_init.time.monotonic', op.clock.monotonic):
                with self.assertRaises(TimeoutError):
                    move_to_init(op)
        self.assertFalse(op.last_init_motion['arms_arrived'])
        self.assertEqual(op.calls[-1], 'stop')

    def test_configuration_is_copied_and_limits_validated(self):
        op = MemoryOperator({'right': 6})
        original = {'right': [0.] * 6}
        configure_init_motion(op, joint_dofs={'right': 6}, init_position=original)
        original['right'][0] = 10.
        self.assertEqual(op.init_position['right'][0], 0.)
        for settings in ({'publish_hz': 0}, {'max_velocity_rad_s': np.inf},
                         {'tolerance_rad': True}, {'unknown': 1}):
            with self.assertRaises(ValueError):
                configure_init_motion(op, joint_dofs={'right': 6}, init_position=original,
                                         init_motion=settings)


if __name__ == '__main__':
    unittest.main()
