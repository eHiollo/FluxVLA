"""Relative-only EE retargeting and normal-loop integration, without hardware."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import runpy
import unittest
from unittest.mock import Mock, patch

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('_wallx_delta_test', ROOT / 'fluxvla/projects/a10/wallx_execution.py')
ex = importlib.util.module_from_spec(spec); spec.loader.exec_module(ex)


class Clock:
    def __init__(self): self.now = 1.
    def monotonic(self): return self.now
    def sleep(self, duration): self.now += max(0., duration)


class DeltaTests(unittest.TestCase):
    def setUp(self):
        self.geometry = ex.g.A10WallXGeometry(ROOT / 'configs/a10/a10_wallx_geometry.json')
        self.contract = ex.ModelContract(json.loads((ROOT / 'configs/a10/a10_wallx_forward_experiment.json').read_text()))
        self.clock = Clock()
        self.devices = ex.SimTimedEEDevices(self.geometry, self.clock)
        self.current = self.geometry.observation_geometry(self.devices.sample()['robot_reply'])
        self.settings = {}
        for name in ('a10_wallx_inference.py', 'a10_wallx_deploy.py', 'a10_wallx_timed_ee.py', 'a10_wallx_timed_ee_delta.py'):
            self.settings.update(runpy.run_path(str(ROOT / 'configs/a10' / name))['wallx'])
        self.settings['print_progress'] = False

    def chunk(self, positions, rotations):
        return dict(source_rows=list(range(len(positions))),
                    poses={s: np.array([self.contract.from_model(s, np.r_[p, r.as_quat()])
                                       for p, r in zip(positions, rotations)]) for s in ('left', 'right')},
                    grippers={s: np.linspace(.25, .75, len(positions)) for s in ('left', 'right')})

    def test_first_prediction_holds_robot_and_world_deltas_keep_physical_axes(self):
        points = np.array([[.8, -.3, .9], [.803, -.302, .901]])
        chunk = self.chunk(points, [Rotation.from_euler('xyz', [.7, -.4, 1.])] * 2)
        original = deepcopy(chunk)
        result, info = ex.reference_actions(chunk, self.current, self.current, self.contract, self.settings)
        for side in ('left', 'right'):
            current = self.devices.poses[side]
            self.assertLess(ex.distance(result['poses'][side][0], current)[0], 1e-12)
            self.assertLess(ex.distance(result['poses'][side][0], current)[1], 1e-12)
            np.testing.assert_allclose(result['poses'][side][1][:3]-current[:3], [.003, -.002, .001], atol=1e-12)
            np.testing.assert_array_equal(chunk['poses'][side], original['poses'][side])
            np.testing.assert_array_equal(result['grippers'][side], original['grippers'][side])
            native0 = self.geometry.controller_pose_from_grasp(side, current, 1)
            native1 = self.geometry.controller_pose_from_grasp(side, result['poses'][side][1], 1)
            np.testing.assert_allclose(native1[:3]-native0[:3], [.003, -.002, .001] if side=='left' else [-.003, .002, .001], atol=1e-12)
        self.assertTrue(info['first_future_motion_discarded'])

    def test_rotation_uses_world_relative_quaternion_and_correct_model_tcp_offset(self):
        first = Rotation.from_euler('xyz', [.4, -.6, 1.2])
        change = Rotation.from_rotvec([0., .03, 0.])
        chunk = self.chunk([[.7, .2, .8]] * 2, [first, change * first])
        result, _ = ex.reference_actions(chunk, self.current, self.current, self.contract, self.settings)
        for side in ('left', 'right'):
            start_model = self.contract.to_model(side, self.devices.poses[side])
            target_model = self.contract.to_model(side, result['poses'][side][1])
            np.testing.assert_allclose(target_model[:3], start_model[:3], atol=1e-12)
            relative = Rotation.from_quat(target_model[3:]) * Rotation.from_quat(start_model[3:]).inv()
            np.testing.assert_allclose(relative.as_rotvec(), [0., .03, 0.], atol=1e-12)
            # Rotation about model EE/tool0 sweeps the offset tool1 grasp centre.
            self.assertGreater(ex.distance(result['poses'][side][1], self.devices.poses[side])[0], .008)

    def test_rpy_wrap_is_two_degrees_and_not_358_degrees(self):
        chunk = self.chunk([[.7, .2, .8]] * 2, Rotation.from_euler('z', [179., -179.], degrees=True))
        result, _ = ex.reference_actions(chunk, self.current, self.current, self.contract, self.settings)
        for side in ('left', 'right'):
            self.assertAlmostEqual(np.rad2deg(ex.distance(result['poses'][side][0], result['poses'][side][1])[1]), 2.)

    def test_fresh_robot_pose_is_anchor_not_pre_inference_observation(self):
        newer = deepcopy(self.current)
        for side in ('left', 'right'):
            newer['arms'][side + '_ee']['grasp_rig_m_xyzw'][0] += .005
        chunk = self.chunk([[.8, -.3, .9]], [Rotation.identity()])
        result, _ = ex.reference_actions(chunk, self.current, newer, self.contract, self.settings)
        for side in ('left', 'right'):
            np.testing.assert_allclose(result['poses'][side][0], newer['arms'][side + '_ee']['grasp_rig_m_xyzw'], atol=1e-12)

    def test_constant_predictions_hold_and_scale_defaults_do_not_amplify(self):
        first = Rotation.from_euler('xyz', [.7, -.4, 1.])
        chunk = self.chunk([[.8, -.3, .9], [.804, -.302, .902]], [first, Rotation.from_rotvec([0., 0., .04])*first])
        result, _ = ex.reference_actions(chunk, self.current, self.current, self.contract,
            dict(self.settings, delta_translation_scale=.5, delta_rotation_scale=.5))
        for side in ('left', 'right'):
            start = self.contract.to_model(side, self.devices.poses[side]); end = self.contract.to_model(side, result['poses'][side][-1])
            np.testing.assert_allclose(end[:3]-start[:3], [.002, -.001, .001], atol=1e-12)
            self.assertAlmostEqual(ex.distance(start, end)[1], .02)
        result, _ = ex.reference_actions(chunk, self.current, self.current, self.contract,
            dict(self.settings, delta_translation_scale=0., delta_rotation_scale=0.))
        for side in ('left', 'right'):
            for pose in result['poses'][side]: self.assertLess(sum(ex.distance(pose, self.devices.poses[side])), 1e-12)

    def test_absolute_mode_preserves_exact_existing_targets(self):
        chunk = self.chunk([[.8, -.3, .9]], [Rotation.identity()])
        result, info = ex.reference_actions(chunk, self.current, self.current, self.contract, {})
        self.assertIs(result, chunk); self.assertIsNone(info)

    def test_invalid_settings_and_missing_first_reference_rejected(self):
        for change in (dict(ee_action_mode='bogus'),dict(delta_translation_scale=-1.),dict(delta_rotation_scale=float('nan')),dict(delta_rotation_scale=True)):
            with self.subTest(change=change), self.assertRaises(ValueError):
                ex.validate_settings(dict(self.settings, **change))
        chunk = self.chunk([[.8, -.3, .9]], [Rotation.identity()]); chunk['source_rows']=[1]
        with self.assertRaisesRegex(ValueError, 'row 0'):
            ex.reference_actions(chunk,self.current,self.current,self.contract,self.settings)

    def test_normal_timed_loop_preserves_row_cadence_and_removes_first_pose_bias(self):
        policy = ex.SimPolicy(); original = policy.infer
        def infer(payload):
            result = original(payload)
            for side in ('left', 'right'):
                result['actions'][side + '_ee']['position'] += [.2, -.1, .1]
                result['actions'][side + '_ee']['rotation_rpy'][:] = [.2, .3, .4]
            return result
        policy.infer = infer; log=Mock()
        with patch.object(ex.bridge, 'time', self.clock):
            result = ex.run(self.devices,policy,self.geometry,self.contract,self.settings,log,cycles=2,read_only=False,clock=self.clock)
        self.assertTrue(result['completed'])
        self.assertEqual((self.devices.starts,self.devices.sent,self.devices.stops),(1,12,1))
        self.assertEqual(self.devices.final_flags,[False]*11+[True])
        self.assertAlmostEqual(self.devices.poses['left'][0],.410)
        self.assertEqual(sum(c.args[0]=='action_reference' for c in log.call_args_list),2)

    def test_large_relative_motion_still_rejected_before_motion(self):
        chunk=self.chunk([[.8, -.3, .9],[1.1,-.3,.9]],[Rotation.identity()]*2)
        chunk,_=ex.reference_actions(chunk,self.current,self.current,self.contract,self.settings)
        with self.assertRaisesRegex(ValueError,'step exceeds'):
            ex.validate_timed_ee_chunk(chunk,self.current,self.current,self.devices.sample(),self.geometry,self.settings)


if __name__ == '__main__': unittest.main()
