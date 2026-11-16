"""World translation/local rotation: synthetic input and simulated robots only."""
import subprocess
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
from mmengine import Config
from scipy.spatial.transform import Rotation

from fluxvla.engines.operators.vr.control import (
    AnchorState, WorldTranslationAnchorState, ReferenceConfig, VRPoseController,
)
from fluxvla.engines.operators.vr.mapper import XLeVRDeltaEEMapper
from fluxvla.engines.operators.vr.runtime import build_runtime
import test_vr_dual_runtime as dual

ROOT = Path(__file__).resolve().parents[1]
MODE = 'world_translation_local_rotation'
PROFILE = 'configs/a10/a10_dual_arm_vr_teleop_world_translation.py'


def sample(seq, position, quat, *, active=True, age=0., trigger=0.):
    return {'xlevr.enabled': active, 'xlevr.grip_active': active,
            'xlevr.buttons': {'squeeze': active}, 'xlevr.trigger': trigger,
            'xlevr.target_position': list(position), 'xlevr.orientation_quat': quat,
            'xlevr.sample_sequence': seq, 'xlevr.sample_receive_time_s': 10. + seq / 90.,
            'xlevr.sample_age_s': age, 'xlevr.thumbstick': {}}


def mapper(world=True, **options):
    return XLeVRDeltaEEMapper(anchor_translation_frame='robot_base' if world else 'tool_anchor',
                             **options)


def command(m, seq, position, quat, **options):
    return m(sample(seq, position, quat, **options))['_xlevr.anchor_command']


class WorldTranslationTest(unittest.TestCase):
    def test_signed_axes_and_scales_are_independent_of_both_anchor_attitudes(self):
        origin = np.array([.1, .2, .3])
        hand_origin = np.array([.2, .5, -.1])
        for hand_angles in ([0., 0., 0.], [.8, -.6, 1.2]):
            hq = Rotation.from_euler('xyz', hand_angles).as_quat()
            for tool_angles in ([0., 0., 0.], [np.pi, .3, -.7]):
                rq = Rotation.from_euler('xyz', tool_angles).as_quat()
                for axis in range(3):
                    for sign in (-1., 1.):
                        with self.subTest(hand=hand_angles, tool=tool_angles, axis=axis, sign=sign):
                            m = mapper(pos_scale=.6, vr_to_robot_scale=1.5)
                            anchor = WorldTranslationAnchorState()
                            actual = np.r_[origin, rq]
                            anchor.update(command(m, 1, hand_origin, hq), actual)
                            np.testing.assert_allclose(anchor.target, actual, atol=1e-12)
                            delta = np.eye(3)[axis] * sign * .003
                            anchor.update(command(m, 2, hand_origin + delta, hq), actual)
                            expected = origin + np.asarray(m._position_axes) @ delta * .6 * 1.5
                            np.testing.assert_allclose(anchor.target[:3], expected, atol=1e-12)
                            np.testing.assert_allclose(anchor.target[3:], rq, atol=1e-12)

    def test_translation_only_matrix_override_does_not_change_rotation(self):
        matrix = [[-1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]
        q0 = Rotation.from_euler('xyz', [.3, -.6, .2]).as_quat()
        q1 = Rotation.from_euler('xyz', [.32, -.59, .21]).as_quat()
        world, old = mapper(position_axis_remap=matrix), mapper(False, position_axis_remap=matrix)
        for m in (world, old):
            command(m, 1, [0., 0., 0.], q0)
        wc = command(world, 2, [.002, .003, .004], q1)
        oc = command(old, 2, [.002, .003, .004], q1)
        np.testing.assert_allclose(wc['offset'][:3], np.array([-.002, .003, .004]) * .9)
        np.testing.assert_allclose(wc['offset'][3:], oc['offset'][3:], atol=1e-12)
        self.assertNotIn('translation_frame', oc)
        self.assertEqual(wc['translation_frame'], 'robot_base')

    def test_pure_rotation_and_duplicate_do_not_move_world_position(self):
        m = mapper()
        q0 = Rotation.from_euler('x', .4).as_quat()
        q1 = Rotation.from_euler('xyz', [.41, .02, -.03]).as_quat()
        command(m, 1, [0., 0., 0.], q0)
        fresh = command(m, 2, [0., 0., 0.], q1)
        duplicate = command(m, 2, [.5, .5, .5], q0)
        self.assertEqual(fresh, duplicate)
        np.testing.assert_array_equal(fresh['offset'][:3], [0., 0., 0.])
        self.assertGreater(np.linalg.norm(fresh['offset'][3:]), 0.)
        diagnostics = m.get_last_diagnostics()
        self.assertEqual(diagnostics['translation_frame'], 'robot_base')
        self.assertIsNone(diagnostics['anchor_shadow_translation_ee_m'])

    def test_deadzone_release_reanchor_and_stale_guards(self):
        m = mapper(pos_deadzone_m=.0005)
        q = [0., 0., 0., 1.]
        command(m, 1, [0., 0., 0.], q)
        small = command(m, 2, [.0001, 0., 0.], q)
        np.testing.assert_array_equal(small['offset'][:3], [0., 0., 0.])
        release = command(m, 3, [.1, 0., 0.], q, active=False)
        self.assertFalse(release['active'])
        new = command(m, 4, [.1, 0., 0.], q)
        np.testing.assert_array_equal(new['offset'], [0.] * 6)
        self.assertGreater(new['anchor_id'], small['anchor_id'])
        stale = command(m, 5, [.101, 0., 0.], q, age=.3)
        self.assertFalse(stale['active'])

    def test_frame_mismatch_is_rejected_before_composition(self):
        q = [0., 0., 0., 1.]
        actual = np.r_[[.1, .2, .3], q]
        old = command(mapper(False), 1, [0., 0., 0.], q)
        world = command(mapper(), 1, [0., 0., 0.], q)
        for anchor, wrong in ((AnchorState(), world), (WorldTranslationAnchorState(), old)):
            with self.assertRaises(ValueError):
                anchor.update(wrong, actual)
            self.assertFalse(anchor.active)

    def test_incremental_rotation_is_identical_at_90hz_and_motion_bounds_hold(self):
        config = ReferenceConfig(rotation_follow_mode='incremental_velocity')
        actual = np.r_[[.1, .2, .3], Rotation.from_euler('xyz', [2.8, .2, -.4]).as_quat()]
        controllers = [VRPoseController(config, pose_mapping=mode)
                       for mode in ('anchor_relative', MODE)]
        mappers = [mapper(False), mapper()]
        previous_velocity = np.zeros(3)
        for c in controllers:
            c.start(actual)
        for seq in range(1, 70):
            results = []
            for m, c in zip(mappers, controllers):
                q = Rotation.from_euler('xyz', [.4, -.3 + seq * .002, .2]).as_quat()
                cmd = command(m, seq, [.0002 * seq, 0., 0.], q)
                results.append(c.step(cmd, actual, 1. / 90.))
            np.testing.assert_allclose(results[0]['command_pose'][3:], results[1]['command_pose'][3:], atol=1e-12)
            velocity = np.array(results[1]['linear_velocity_m_s'])
            self.assertLessEqual(np.linalg.norm(velocity), config.max_linear_speed_m_s + 1e-12)
            self.assertLessEqual(np.linalg.norm(velocity - previous_velocity), config.max_linear_accel_m_s2 / 90. + 1e-12)
            previous_velocity = velocity

    def test_profile_inherits_tuning_and_never_reads_calibration(self):
        old = Config.fromfile(str(ROOT / 'configs/a10/a10_dual_arm_vr_teleop.py')).teleop
        cfg = Config.fromfile(str(ROOT / PROFILE)).teleop
        for key in old:
            if key not in ('pose_mapping', 'pose_calibration_path', 'diagnostics_path', 'input', 'record_controller', 'confirm_stop', 'resilient_takeover', 'continuous_feedback', 'stop_on_both_release', 'operator'):
                self.assertEqual(cfg[key], old[key], key)
        self.assertEqual({k:v for k,v in cfg.input.items() if k not in ('mapping_by_hand', 'mapping')},
                         {k:v for k,v in old.input.items() if k != 'mapping'})
        self.assertTrue(cfg.input.mapping.squeeze_translation_only)
        self.assertEqual({k:v for k,v in cfg.input.mapping.items() if k != 'squeeze_translation_only'},
                         dict(old.input.mapping))
        self.assertEqual(cfg.fps, 90)
        self.assertFalse(cfg.record_controller)
        self.assertFalse(cfg.confirm_stop)
        self.assertTrue(cfg.resilient_takeover)
        self.assertFalse(cfg.continuous_feedback)
        self.assertFalse(cfg.stop_on_both_release)
        self.assertFalse(cfg.operator.shared_control.require_feedback)
        self.assertEqual({k: v for k, v in cfg.operator.items() if k != 'shared_control'},
                         {k: v for k, v in old.operator.items() if k != 'shared_control'})
        self.assertEqual({k: v for k, v in cfg.operator.shared_control.items() if k != 'require_feedback'},
                         dict(old.operator.shared_control))
        self.assertIsNone(cfg.pose_calibration_path)
        cfg.update(vr_only=True, record_controller=False)
        cfg.zx_grippers.enabled = False
        # A corrupt/unavailable site file is irrelevant to this mapping.
        cfg.pose_calibration_path = '/not-present/invalid-calibration.json'
        with patch.object(Path, 'read_text', side_effect=AssertionError('Calibration was read')):
            runtime = build_runtime(cfg)
        self.assertEqual(runtime.pose_mapping, MODE)
        self.assertTrue(runtime.resilient_takeover)
        self.assertFalse(runtime.continuous_feedback)
        for context in runtime._arms.values():
            self.assertEqual(context['mapper'].anchor_translation_frame, 'robot_base')
            self.assertIsInstance(context['control'].anchor, WorldTranslationAnchorState)
        runtime.close()

    def test_live_profile_physical_bindings_and_world_axes_preserve_local_mode(self):
        cfg = Config.fromfile(str(ROOT / PROFILE)).teleop
        cfg.update(vr_only=True, record_controller=False)
        cfg.zx_grippers.enabled = False
        runtime = build_runtime(cfg)
        self.assertEqual(runtime.teleoperator.hand_to_arm, {'left':'right','right':'left'})
        expected = {'left': np.array([[0.,0.,-1.],[-1.,0.,0.],[0.,1.,0.]]),
                    'right': np.array([[0.,0.,1.],[1.,0.,0.],[0.,1.,0.]])}
        for hand, logical in runtime.teleoperator.hand_to_arm.items():
            m = runtime._arms[logical]['mapper']
            np.testing.assert_allclose(m._position_axes, expected[hand])
            for vr, table in (([0.,0.,-1.],[1.,0.,0.]),([-1.,0.,0.],[0.,1.,0.]),([0.,1.,0.],[0.,0.,1.])):
                base_delta = np.asarray(m._position_axes) @ vr
                table_from_base = np.eye(3) if hand=='left' else np.diag([-1.,-1.,1.])
                np.testing.assert_allclose(table_from_base @ base_delta, table)
            m.anchor_translation_frame = 'tool_anchor'
            np.testing.assert_allclose(m._position_axes, [[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]])
        runtime.close()
        with self.assertRaises(ValueError):
            mapper(world_position_axis_remap=[[1.,0.,0.],[0.,0.,0.],[0.,0.,1.]])

    def test_cli_preview_and_offline_replay(self):
        commands = [
            ['scripts/teleoperate.py', '--config', PROFILE, '--preview'],
            ['scripts/replay_vr_control.py', '--demo', '--pose-mapping', MODE, '--fps', '90'],
        ]
        for args in commands:
            result = subprocess.run([sys.executable, *args], cwd=ROOT, text=True,
                                    capture_output=True, timeout=90)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


class StopRequestRuntimeTest(unittest.TestCase):
    def test_both_release_stops_targets_without_stop_confirmation_calls(self):
        fixture = dual.DualSDKRuntimeTest('test_release_one_arm_keeps_other_moving_and_whole_packets')
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        fixture.runtime.confirm_stop = False
        def request_only():
            fixture.robot.events.append('stop_request')
            fixture.robot.follower_stop_pending = False
            return dict(requested=True, confirmed=False)
        fixture.robot.request_stop_follower = request_only
        fixture.move()
        count = len(fixture.robot.targets)
        with patch.object(fixture.robot, 'stop_follower', side_effect=AssertionError('Confirmed stop called')),              patch.object(fixture.robot, 'wait_stopped', side_effect=AssertionError('Stop state polled')):
            fixture.tick(False, False)
            fixture.runtime.close()
        self.assertEqual(len(fixture.robot.targets), count)
        self.assertEqual(fixture.robot.events.count('stop_request'), 1)
        self.assertEqual(fixture.runtime.last_reference_result['state'], 'stop_requested')
        self.assertFalse(fixture.runtime.last_stop_status['reported_stopped'])
        self.assertFalse(fixture.runtime.last_stop_status['mode_inactive'])


if __name__ == '__main__':
    unittest.main()
