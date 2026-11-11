"""Reference VR uses time-based motion bounds, including jittered control ticks."""
import json
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

import test_vr_dual_runtime as dual
import test_vr_reference_runtime as single
import test_a10_operator as controller
from fluxvla.engines.operators.a10_operator import SDKToolCumulativeAdapter
from fluxvla.engines.operators.vr.control import ReferenceConfig, VRPoseController
from fluxvla.engines.operators.vr.runtime import VRTeleopRuntime
from test_vr_operator import FakeVR


class DualReferenceSpeedTest(unittest.TestCase):
    robot_type = dual.DualRobot
    cartesian_adapter = 'legacy_anchor'
    setUp = dual.DualRuntimeTest.setUp
    tick = dual.DualRuntimeTest.tick

    def test_jitter_uses_speed_and_acceleration_instead_of_fixed_steps(self):
        # Old CLI limits remain accepted but are irrelevant to reference mode.
        self.runtime.max_position_step_m = 1e-7
        self.runtime.max_rotation_step_rad = 1e-7
        cfg = ReferenceConfig(max_linear_speed_m_s=.14, max_angular_speed_rad_s=.7,
                              max_linear_accel_m_s2=.4, max_angular_accel_rad_s2=1.5)
        for context in self.runtime._arms.values():
            context['control'] = VRPoseController(cfg)
        max_distance = max_angle = 0.
        previous = None
        for i in range(65):
            dt = (.033, .044, .06, .04)[i % 4]
            self.clock.value += dt - 1. / 30.
            self.tick(xleft=.006 * i, xright=.003 * i, angle=.08 * i)
            result = self.runtime.last_reference_result
            if result['state'] != 'running':
                continue
            out = result['arms']['left']
            pose = np.array(out['command_pose'])
            v = np.array(out['linear_velocity_m_s'])
            w = np.array(out['angular_velocity_rad_s'])
            self.assertLessEqual(np.linalg.norm(v), .14 + 1e-10)
            self.assertLessEqual(np.linalg.norm(w), .7 + 1e-10)
            if previous is not None:
                p, oldv, oldw = previous
                distance = np.linalg.norm(pose[:3] - p[:3])
                angle = (Rotation.from_quat(p[3:]).inv() * Rotation.from_quat(pose[3:])).magnitude()
                self.assertLessEqual(distance, .14 * dt + 1e-10)
                self.assertLessEqual(angle, .7 * dt + 1e-10)
                self.assertLessEqual(np.linalg.norm(v - oldv), .4 * dt + 1e-10)
                self.assertLessEqual(np.linalg.norm(w - oldw), 1.5 * dt + 1e-10)
                max_distance, max_angle = max(max_distance, distance), max(max_angle, angle)
            previous = pose, v, w
            self.assertTrue(result['transmitted'])
        self.assertGreater(max_distance, .005)
        self.assertGreater(max_angle, .025)
        self.assertNotIn('stop', self.robot.events)
        count = len(self.robot.targets)
        self.clock.value += .11 - 1. / 30.
        with self.assertRaisesRegex(RuntimeError, 'control_gap'):
            self.tick(xleft=.006 * 64, xright=.003 * 64, angle=.08 * 64)
        self.assertEqual(len(self.robot.targets), count)
        self.assertIn('stop', self.robot.events)


class DualSDKReferenceSpeedTest(DualReferenceSpeedTest):
    robot_type = dual.DualSDKRobot
    cartesian_adapter = 'sdk_tool_cumulative'


class SingleReferenceSpeedTest(unittest.TestCase):
    def test_reference_accepts_no_step_limits_and_keeps_speed_bound(self):
        rows = [single.sample(i, offset=.01 * i) for i in range(50)]
        robot = single.StoppableA10()
        runtime = VRTeleopRuntime(FakeVR(rows), single.AnchorMapper(), robot,
            execute=True, control_mode='reference',
            reference_options=dict(max_linear_speed_m_s=.14))
        clock = single.Clock()
        clock.value = 100.
        distances = []
        with patch('time.monotonic', clock.now):
            try:
                runtime.connect()
                previous = None
                for _ in rows:
                    clock.value += .06
                    runtime.step()
                    pose = runtime.reference_control.command_pose.copy()
                    if previous is not None:
                        distances.append(np.linalg.norm(pose[:3] - previous[:3]))
                    previous = pose
                    robot.pose = pose.copy()  # Ideal feedback, no hardware involved.
            finally:
                runtime.close()
        self.assertGreater(max(distances), .005)
        self.assertLessEqual(max(distances), .14 * .06 + 1e-10)


class SDKOptionalStepBoundsTest(unittest.TestCase):
    def test_explicit_console_bounds_and_geometry_validation_still_apply(self):
        start = np.array([0., 0., 0., 0., 0., 0., 1.])
        adapter = SDKToolCumulativeAdapter(start)
        target = np.r_[[.005097733846, 0., 0.], Rotation.from_euler('z', .027916577503).as_quat()]
        with self.assertRaisesRegex(ValueError, 'step limit'):
            adapter.propose(target, max_position_step_m=.005, max_rotation_step_rad=.025)
        adapter.propose(target, max_position_step_m=None, max_rotation_step_rad=None)
        np.testing.assert_array_equal(adapter.previous_pose, start)
        for bad in (0., -1., float('nan'), float('inf')):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                adapter.propose(target, max_position_step_m=bad, max_rotation_step_rad=None)
        target[0] = float('nan')
        with self.assertRaises(ValueError):
            adapter.propose(target, max_position_step_m=None, max_rotation_step_rad=None)
        target = np.r_[[0., 0., 0.], Rotation.from_euler('z', np.pi).as_quat()]
        with self.assertRaisesRegex(ValueError, 'pi/2'):
            adapter.propose(target, max_position_step_m=None, max_rotation_step_rad=None)

    def test_real_codec_sends_complete_dual_group_without_fixed_step_bounds(self):
        with controller.FakeController() as server:
            helper = controller.DualSDKConsoleTest()
            robot, poses = helper.start(server)
            try:
                poses['left'][0] += .005097733846
                poses['right'][3:] = Rotation.from_euler('z', .027916577503).as_quat()
                reply = robot.send_sdk_tool_pose(poses, max_position_step_m=None,
                                                max_rotation_step_rad=None)
                packet = json.loads(server.udp.recv(4096)[40:])
                self.assertEqual(packet['type'], '321')
                self.assertEqual(np.asarray(packet['pe']).shape, (2, 6))
                self.assertTrue(reply['local_send_success'])
                self.assertGreater(np.linalg.norm(packet['pe'][0][:3]), .005)
                self.assertGreater(np.linalg.norm(packet['pe'][1][3:]), .025)
            finally:
                robot.stop_follower()
                robot.close()


if __name__ == '__main__':
    unittest.main()
