"""Offline VR-to-A10 pose and lifecycle tests; no network or robot motion."""

import time
import unittest
import json
import struct

import numpy as np
from scipy.spatial.transform import Rotation

from fluxvla.engines.operators.vr.runtime import VRTeleopRuntime
from fluxvla.engines.operators.vr.runtime import build_runtime
from mmengine import Config
from pathlib import Path
from fluxvla.engines.operators.vr.mapper import XLeVRDeltaEEMapper
from fluxvla.engines.operators.vr.input import VRInput
from test_a10_operator import FakeController


def action(*, enabled=True, translation=(0., 0., 0.),
           rotation=(0., 0., 0.), gripper=0.):
    return dict({'ee.enabled': enabled, 'gripper.pos': gripper},
                **{f'ee.delta_{axis}': value for axis, value in zip('xyz', translation)},
                **{f'ee.delta_r{axis}': value for axis, value in zip('xyz', rotation)})


class FakeVR:
    def __init__(self, samples):
        self.samples = iter(samples)
        self.is_connected = False
        self.events = []

    def connect(self):
        self.is_connected = True

    def disconnect(self):
        self.is_connected = False

    def get_action(self):
        return next(self.samples)

    def get_vr_events(self):
        return {'stop_recording': self.events.pop(0) if self.events else False}


class FakeMapper:
    stale_timeout_s = .25
    def __init__(self):
        self.resets = 0

    def __call__(self, raw):
        return raw['mapped']

    def reset(self):
        self.resets += 1


class FakeA10:
    allow_motion = True
    command_mode = 'cartesian'
    feedback_position_unit = 'mm'
    arm_name = 'right'

    def __init__(self):
        self.events = []
        self.targets = []
        self.follower_stop_pending = False
        self.pose = np.array([.1, .2, .3, 0., 0., 0., 1.])
        self.fault = False
        self.stale = False

    def connect(self):
        self.events.append('connect')

    def get_frame(self):
        self.events.append('read')
        return dict(tcp_pq_m=self.pose, has_error=self.fault,
                    received_monotonic_s=time.monotonic() - (2 if self.stale else 0),
                    follower_active=self.follower_stop_pending,
                    follower_mode='Cart' if self.follower_stop_pending else None)

    def set_tool_coordinate(self):
        self.events.append('tool')

    def start_follower(self, mode):
        self.events.append(('start', mode))
        self.follower_stop_pending = True

    def send_eepose(self, target):
        self.events.append('send')
        self.targets.append(target)

    def stop_follower(self):
        self.events.append('stop')
        self.follower_stop_pending = False

    def close(self):
        self.events.append('close')


class FakeGripper:
    max_width_mm = 85

    def __init__(self):
        self.events = []
        self.position = 40.

    def connect(self):
        self.events.append('connect')

    def get_position_mm(self):
        return self.position

    def set_position_mm(self, value):
        self.events.append(('set', value))

    def disconnect(self):
        self.events.append('disconnect')


def make_live(samples, robot=None, **kwargs):
    robot = robot or FakeA10()
    return VRTeleopRuntime(FakeVR(samples), FakeMapper(), robot, execute=True,
                      max_position_step_m=.02, max_rotation_step_rad=.2,
                      **kwargs), robot


class VRTeleopRuntimeTest(unittest.TestCase):
    def test_project_entrypoint_builds_vr_only_and_blocks_unverified_motion(self):
        cfg = Config.fromfile(str(Path(__file__).resolve().parents[1] / 'configs/a10/a10_vr_teleop.py')).teleop
        self.assertIsInstance(build_runtime(cfg), VRTeleopRuntime)
        cfg.vr_only = False
        with self.assertRaises(ValueError):
            build_runtime(cfg)

    def test_vr_only_never_connects_robot(self):
        vr = FakeVR([{'mapped': action()}])
        operator = VRTeleopRuntime(vr, FakeMapper())
        self.assertEqual(operator.run(max_steps=1), 1)
        self.assertFalse(vr.is_connected)

    def test_live_requires_feedback_units_and_limits(self):
        robot = FakeA10()
        robot.feedback_position_unit = None
        with self.assertRaisesRegex(ValueError, 'feedback_position_unit'):
            VRTeleopRuntime(FakeVR([]), FakeMapper(), robot, execute=True,
                       max_position_step_m=.02, max_rotation_step_rad=.2)
        robot.feedback_position_unit = 'mm'
        with self.assertRaisesRegex(ValueError, 'max_position_step_m'):
            VRTeleopRuntime(FakeVR([]), FakeMapper(), robot, execute=True,
                       max_rotation_step_rad=.2)

    def test_local_delta_composes_with_current_pose_and_stops(self):
        robot = FakeA10()
        robot.pose[3:] = Rotation.from_euler('z', 90, degrees=True).as_quat()
        sample = {'mapped': action(translation=(.01, 0., 0.), rotation=(0., 0., .1))}
        operator, robot = make_live([sample, sample], robot)
        self.assertEqual(operator.run(max_steps=2), 2)
        np.testing.assert_allclose(robot.targets[0]['right'], [0, 0, 0, 0, 0, 0, 1], atol=1e-8)
        target = robot.targets[1]['right']
        np.testing.assert_allclose(target[:3], [.01, 0, 0], atol=1e-8)
        expected = Rotation.from_rotvec([0, 0, .1])
        np.testing.assert_allclose(target[3:], expected.as_quat(), atol=1e-8)
        self.assertEqual(robot.events.count('send'), 2)
        self.assertEqual(robot.events[-2:], ['stop', 'close'])

    def test_released_control_and_reset_send_no_pose(self):
        samples = [{'mapped': action(enabled=False)},
                   {'mapped': action(), 'xlevr.reset_arm': True}]
        operator, robot = make_live(samples)
        operator.run(max_steps=2)
        self.assertNotIn('send', robot.events)
        self.assertNotIn('tool', robot.events)
        self.assertEqual(operator.mapper.resets, 2)

    def test_accumulation_preserves_displacement_with_lagging_feedback(self):
        samples = [{'mapped': action(translation=(.01, 0, 0))}] * 4
        operator, robot = make_live(samples)
        operator.run(max_steps=4)
        np.testing.assert_allclose([t['right'][0] for t in robot.targets],
                                   [0., .01, .02, .03], atol=1e-8)

    def test_release_holds_actual_and_reengagement_reanchors(self):
        samples = [{'mapped': action(translation=(.01, 0, 0))}] * 2
        samples += [{'mapped': action(enabled=False)}] * 2
        samples += [{'mapped': action(translation=(.01, 0, 0))}]
        operator, robot = make_live(samples)
        operator.run(max_steps=5)
        np.testing.assert_allclose([t['right'][0] for t in robot.targets],
                                   [0., .01, 0., 0., .01], atol=1e-8)

    def test_duplicate_zero_delta_keeps_target_and_reset_holds_actual(self):
        samples = [{'mapped': action(translation=(.01, 0, 0))}] * 2
        samples += [{'mapped': action()},
                    {'mapped': action(), 'xlevr.reset_arm': True}]
        operator, robot = make_live(samples)
        operator.run(max_steps=4)
        np.testing.assert_allclose([t['right'][0] for t in robot.targets],
                                   [0., .01, .01, 0.], atol=1e-8)

    def test_tracking_error_stops_without_sending_excess_target(self):
        operator, robot = make_live(
            [{'mapped': action(translation=(.01, 0, 0))}] * 5,
            max_position_tracking_error_m=.025)
        with self.assertRaisesRegex(RuntimeError, 'tracking error'):
            operator.run(max_steps=5)
        self.assertEqual(len(robot.targets), 3)
        self.assertEqual(robot.events[-2:], ['stop', 'close'])

    def test_rotation_accumulates_in_target_tool_frame(self):
        samples = [{'mapped': action(rotation=(0, 0, .1))}] * 2
        samples += [{'mapped': action(translation=(.01, 0, 0))}]
        operator, robot = make_live(samples)
        operator.run(max_steps=3)
        target = robot.targets[-1]['right']
        np.testing.assert_allclose(target[:3],
                                   [.01 * np.cos(.1), .01 * np.sin(.1), 0.], atol=1e-8)
        np.testing.assert_allclose(target[3:], Rotation.from_rotvec([0, 0, .1]).as_quat())

    def test_rotation_tracking_limit_stops_accumulation(self):
        operator, robot = make_live(
            [{'mapped': action(rotation=(0, 0, .1))}] * 4,
            max_rotation_tracking_error_rad=.15)
        with self.assertRaisesRegex(RuntimeError, 'tracking error'):
            operator.run(max_steps=4)
        self.assertEqual(len(robot.targets), 2)
        self.assertEqual(robot.events[-2:], ['stop', 'close'])

    def test_anchor_orientation_is_removed_before_sending_quaternion(self):
        robot = FakeA10()
        robot.pose[3:] = Rotation.from_euler('xyz', [.4, -.5, .7]).as_quat()
        samples = [{'mapped': action()},
                   {'mapped': action(rotation=(.1, 0, 0))},
                   {'mapped': action(rotation=(0, 0, .1))}]
        operator, robot = make_live(samples, robot)
        operator.run(max_steps=3)
        expected = (Rotation.from_rotvec([.1, 0, 0])
                    * Rotation.from_rotvec([0, 0, .1]))
        np.testing.assert_allclose(robot.targets[-1]['right'][3:], expected.as_quat(), atol=1e-8)
        np.testing.assert_allclose(robot.targets[-1]['right'][:3], [0, 0, 0], atol=1e-8)

    def test_release_at_moved_tcp_holds_relative_to_original_anchor(self):
        robot = FakeA10()
        robot.pose[3:] = Rotation.from_euler('z', 90, degrees=True).as_quat()
        anchor = robot.pose.copy()
        samples = [{'mapped': action()},
                   {'mapped': action(translation=(.01, 0, 0))},
                   {'mapped': action(enabled=False)},
                   {'mapped': action(), 'xlevr.reset_arm': True},
                   {'mapped': action(translation=(.01, 0, 0))}]
        operator, robot = make_live(samples, robot)
        operator.connect()
        try:
            operator.step()
            operator.step()
            # Mutate feedback storage: it must never mutate the startup anchor.
            robot.pose[1] += .006
            operator.step()
            operator.step()
            operator.step()
            np.testing.assert_allclose([t['right'][0] for t in robot.targets],
                                       [0., .01, .006, .006, .016], atol=1e-8)
            np.testing.assert_allclose(operator._follower_anchor_pose, anchor)
            self.assertEqual(robot.events.count(('start', 'cartesian')), 1)
        finally:
            operator.close()

    def test_restart_refreshes_position_and_orientation_anchor(self):
        samples = [{'mapped': action()}] * 2
        samples += [{'mapped': action(translation=(0, 0, .01))}]
        operator, robot = make_live(samples)
        operator.run(max_steps=1)
        self.assertIsNone(operator._follower_anchor_pose)
        robot.pose = np.array([.7, -.4, .6, *Rotation.from_euler('y', 70, degrees=True).as_quat()])
        operator.run(max_steps=2)
        for index in (0, 1):
            np.testing.assert_allclose(robot.targets[index]['right'],
                                       [0, 0, 0, 0, 0, 0, 1], atol=1e-8)
        np.testing.assert_allclose(robot.targets[2]['right'][:3], [0, 0, .01], atol=1e-8)
        self.assertEqual(robot.events.count(('start', 'cartesian')), 2)

    def test_relative_hold_quaternion_tracks_actual_orientation(self):
        robot = FakeA10()
        initial_rotation = Rotation.from_euler('xyz', [.3, .4, -.2])
        robot.pose[3:] = initial_rotation.as_quat()
        operator, robot = make_live([{'mapped': action()}] * 2
                                   + [{'mapped': action(enabled=False)}], robot)
        operator.connect()
        try:
            operator.step()
            operator.step()
            robot.pose[3:] = (initial_rotation * Rotation.from_rotvec([.02, -.03, .01])).as_quat()
            operator.step()
            np.testing.assert_allclose(robot.targets[-1]['right'][3:],
                                       Rotation.from_rotvec([.02, -.03, .01]).as_quat(), atol=1e-8)
        finally:
            operator.close()

    def test_stale_vr_gripper_does_not_repeat_motion(self):
        gripper = FakeGripper()
        sample = {'mapped': action(enabled=False, gripper=1.),
                  'xlevr.sample_age_s': 1.}
        operator, robot = make_live([sample] * 2, gripper=gripper)
        operator.run(max_steps=2)
        self.assertFalse(any(isinstance(x, tuple) for x in gripper.events))

    def test_partial_follower_start_is_stopped_on_failure(self):
        operator, robot = make_live([{'mapped': action()}])
        def interrupted_start(mode):
            robot.follower_stop_pending = True
            raise TimeoutError('start confirmation timeout')
        robot.start_follower = interrupted_start
        with self.assertRaises(TimeoutError):
            operator.run(max_steps=1)
        self.assertEqual(robot.events[-2:], ['stop', 'close'])

    def test_input_disconnect_is_safe_before_connect(self):
        vr = VRInput(None)
        vr.disconnect()
        vr.disconnect()

    def test_anchor_release_carries_current_input_sequence(self):
        mapper = XLeVRDeltaEEMapper()
        def sample(sequence, grip):
            # VRInput already folds squeeze state into xlevr.enabled.
            return {'xlevr.enabled': grip, 'xlevr.grip_active': grip,
                    'xlevr.target_position': [0., 0., 0.],
                    'xlevr.orientation_quat': [0., 0., 0., 1.],
                    'xlevr.sample_sequence': sequence,
                    'xlevr.sample_receive_time_s': 10. + sequence / 30.,
                    'xlevr.sample_age_s': 0.}
        mapper(sample(10, True))
        mapper(sample(11, True))
        release = mapper(sample(12, False))['_xlevr.anchor_command']
        self.assertFalse(release['active'])
        self.assertEqual(release['sample_sequence'], 12)

    def test_real_mapper_and_operator_send_accumulated_udp_pose(self):
        # A fixed actual pose deliberately simulates controller tracking lag.
        # Browser Y maps to tool X. Include a duplicate and a stale sample.
        def raw(seq, y, *, age=0.):
            return {'xlevr.enabled': True, 'xlevr.grip_active': True,
                    'xlevr.target_position': [0., y, 0.],
                    'xlevr.orientation_quat': [0., 0., 0., 1.],
                    'xlevr.sample_sequence': seq,
                    'xlevr.sample_receive_time_s': 10. + seq / 30.,
                    'xlevr.sample_age_s': age}
        samples = [raw(1, 0.), raw(2, .01), raw(3, .02),
                   raw(4, .03), raw(5, .04), raw(5, .04),
                   raw(5, .04, age=1.)]
        with FakeController() as server:
            robot = server.operator(feedback_position_unit='mm')
            mapper = XLeVRDeltaEEMapper(pos_scale=1., pos_deadzone_m=0.)
            operator = VRTeleopRuntime(FakeVR(samples), mapper, robot, execute=True,
                                  max_position_step_m=.02, max_rotation_step_rad=.2)
            operator.run(max_steps=len(samples))
            poses = []
            for _ in range(6):
                packet = server.udp.recv(4096)
                header = struct.unpack('<IIQqqq', packet[:40])
                self.assertEqual(header, (len(packet) - 40, 0, 0, 0, 0, 0))
                poses.append(json.loads(packet[40:])['pq'][0])
            np.testing.assert_allclose([p[0] for p in poses],
                                       [0., 0., .01, .02, .02, 0.], atol=1e-8)
            self.assertEqual(server.commands.count('follower_cart'), 1)
            self.assertEqual(server.commands.count('stop_follower'), 1)
            self.assertFalse(server.errors)

    def test_left_stick_stop_is_checked_before_motion(self):
        operator, robot = make_live([{'mapped': action(translation=(.01, 0, 0))}])
        operator.vr.events = [True]
        self.assertEqual(operator.run(max_steps=1), 0)
        self.assertNotIn('tool', robot.events)
        self.assertNotIn('send', robot.events)

    def test_stale_or_bad_delta_never_starts_follower(self):
        robot = FakeA10()
        robot.stale = True
        operator, robot = make_live([{'mapped': action()}], robot)
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            operator.run(max_steps=1)
        self.assertNotIn('tool', robot.events)
        robot = FakeA10()
        operator, robot = make_live([{'mapped': action(translation=(.03, 0, 0))}], robot)
        with self.assertRaisesRegex(ValueError, 'step limit'):
            operator.run(max_steps=1)
        self.assertNotIn('tool', robot.events)

    def test_gripper_thumbstick_works_without_arm_engagement(self):
        gripper = FakeGripper()
        samples = [{'mapped': action(enabled=False, gripper=1.)}] * 2
        operator, robot = make_live(samples, gripper=gripper)
        operator.run(max_steps=2)
        self.assertNotIn('send', robot.events)
        self.assertEqual(len([x for x in gripper.events if isinstance(x, tuple)]), 1)
        self.assertEqual(gripper.events[-1], 'disconnect')


if __name__ == '__main__':
    unittest.main()
