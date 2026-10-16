"""Pure offline control tests, including results from the original C++ source.

python -m unittest discover -s test -p test_vr_control.py -v
The source file is loaded directly to avoid unrelated model/GPU initialization.
"""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('vr_reference_control', ROOT / 'fluxvla/engines/operators/vr/control.py')
control = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = control
spec.loader.exec_module(control)
IDENTITY = np.array([0., 0., 0., 0., 0., 0., 1.])


def from_matrix(values):
    matrix = np.asarray(values).reshape(4, 4)
    return np.r_[matrix[:3, 3], Rotation.from_matrix(matrix[:3, :3]).as_quat()]


def to_matrix(value):
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(value[3:]).as_matrix()
    matrix[:3, 3] = value[:3]
    return matrix.ravel()


def command(seq=1, *, anchor=1, active=True, offset=(0.,)*6):
    return dict(active=active, session_id='unit-test', anchor_id=anchor,
                sample_sequence=seq, offset=list(offset), gripper=0.)


class CppParityTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture = json.loads((ROOT / 'test/data/vr_control/reference.json').read_text())

    def test_anchor_composition_order_and_duplicate_match_cpp(self):
        shadow = control.AnchorState()
        for row, expected in zip(self.fixture['inputs']['anchors'], self.fixture['expected']['anchors']):
            with self.subTest(command=row['command']):
                update = shadow.update(row['command'], from_matrix(row['actual_pm']))
                self.assertEqual(update, expected['packet_status'])
                self.assertEqual(shadow.active, expected['active'])
                np.testing.assert_allclose(to_matrix(shadow.robot_anchor), expected['robot_anchor_pm'], atol=1e-11)
                np.testing.assert_allclose(to_matrix(shadow.target), expected['target_pm'], atol=1e-11)

    def test_reference_speed_rotation_freeze_release_and_fault_match_cpp(self):
        for case, expected_rows in zip(self.fixture['inputs']['governors'], self.fixture['expected']['governors']):
            governor = control.ReferenceGovernor(control.ReferenceConfig(tracking_fault_after_s=case['fault_cycles'] * .002))
            for index, (row, expected) in enumerate(zip(case['rows'], expected_rows)):
                with self.subTest(case=case['rows'][0], frame=index):
                    actual = from_matrix(row['actual_pm'])
                    if row['operation'] == 'step':
                        governor.step(from_matrix(row['target_pm']), actual, row['dt_s'], hold_reference=row.get('hold', False))
                    else:
                        getattr(governor, {'fault': 'force_fault'}.get(row['operation'], row['operation']))(actual)
                    self.assertEqual(governor.state, expected['state'])
                    np.testing.assert_allclose(to_matrix(governor.reference), expected['reference_pm'], atol=1e-10)
                    self.assertAlmostEqual(governor.tracking_error_m, expected['tracking_error_m'], places=10)
                    self.assertAlmostEqual(governor.tracking_error_rad, expected['tracking_error_rad'], places=7)

    def test_velocity_clamp_and_component_slew_match_cpp(self):
        for row, expected in zip(self.fixture['inputs']['velocities'], self.fixture['expected']['velocities']):
            bounded = control.clamp_vector(row['target'], row['maximum'])
            np.testing.assert_allclose(bounded, expected['clamped'], atol=1e-13)
            np.testing.assert_allclose(control.slew_vector(row['current'], bounded, row['maximum_delta']),
                                       expected['slewed'], atol=1e-13)


class FrontendReferenceTest(unittest.TestCase):
    def make_controller(self, **kwargs):
        controller = control.VRPoseController(control.ReferenceConfig(**kwargs))
        controller.start(IDENTITY)
        return controller

    def test_two_anchors_are_independent_in_rotated_frame(self):
        startup = np.r_[[.2, -.1, .3], Rotation.from_euler('z', 90, degrees=True).as_quat()]
        target = control.compose_offset(startup, [.02, 0., 0., 0., 0., .1])
        np.testing.assert_allclose(control.follower_relative(startup, target)[:3], [.02, 0., 0.], atol=1e-12)
        controller = self.make_controller(limit_acceleration=False)
        controller.step(command(), IDENTITY, .02)
        moved = IDENTITY.copy()
        moved[0] = .015
        controller.step(command(seq=2, active=False), moved, .02)
        result = controller.step(command(seq=3, anchor=2), moved, .02)
        np.testing.assert_allclose(result['relative_target'][:3], [.015, 0., 0.], atol=1e-12)
        np.testing.assert_allclose(controller.startup_anchor, IDENTITY)

    def test_duplicate_and_stale_release_do_not_reapply_or_refresh_input(self):
        controller = self.make_controller(limit_acceleration=False)
        controller.step(command(seq=2, offset=(.02, 0., 0., 0., 0., 0.)), IDENTITY, .03)
        result = controller.step(command(seq=1, active=False), IDENTITY, .03)
        self.assertEqual(result['packet_status'], 'stale')
        self.assertEqual(result['state'], 'tracking')
        for _ in range(9):
            result = controller.step(command(seq=2), IDENTITY, .03)
        self.assertEqual(result['fault_reason'], 'input_stale')
        self.assertIsNone(result['relative_target'])

    def test_legacy_equal_sequence_release_stops_instead_of_being_dropped(self):
        controller = self.make_controller()
        controller.step(command(seq=2), IDENTITY, .02)
        result = controller.step(command(seq=2, active=False), IDENTITY, .02)
        self.assertEqual(result['packet_status'], 'inactive')
        self.assertTrue(result['stop_requested'])
        self.assertIsNone(result['relative_target'])

    def test_release_withholds_targets_and_fault_requires_explicit_restart(self):
        controller = self.make_controller()
        controller.step(command(), IDENTITY, .02)
        result = controller.step(command(seq=2, active=False), IDENTITY, .02)
        self.assertTrue(result['stop_requested'])
        self.assertIsNone(result['relative_target'])
        result = controller.step(command(seq=3), IDENTITY, .02, feedback_age_s=.3)
        self.assertEqual(result['fault_reason'], 'feedback_stale')
        result = controller.step(command(seq=4, anchor=2), IDENTITY, .02)
        self.assertEqual(result['state'], 'fault')
        controller.start(IDENTITY)
        self.assertEqual(controller.step(command(seq=5, anchor=3), IDENTITY, .02)['state'], 'tracking')

    def test_variable_rate_freeze_uses_elapsed_seconds_not_50_frontend_frames(self):
        governor = control.ReferenceGovernor()
        governor.engage(IDENTITY)
        actual = IDENTITY.copy()
        actual[0] = .1
        for dt in [.02, .03, .049]:
            self.assertEqual(governor.step(IDENTITY, actual, dt), 'frozen')
        self.assertEqual(governor.step(IDENTITY, actual, .001), 'fault')

    def test_reference_extension_bounds_velocity_and_acceleration_on_turns(self):
        controller = self.make_controller()
        actual = IDENTITY.copy()
        last_linear = last_angular = np.zeros(3)
        for index in range(1, 150):
            offset = [.04 if index < 70 else -.01, .03, 0., 0., .08, -.05]
            result = controller.step(command(seq=index, offset=offset), actual, .01)
            self.assertEqual(result['state'], 'tracking')
            linear = np.array(result['linear_velocity_m_s'])
            angular = np.array(result['angular_velocity_rad_s'])
            self.assertLessEqual(np.linalg.norm(linear), .12 + 1e-12)
            self.assertLessEqual(np.linalg.norm(angular), .32 + 1e-12)
            self.assertLessEqual(np.linalg.norm(linear - last_linear), .4 * .01 + 1e-12)
            self.assertLessEqual(np.linalg.norm(angular - last_angular), .96 * .01 + 1e-12)
            actual = np.array(result['command_pose'])  # Explicit perfect synthetic follower.
            last_linear, last_angular = linear, angular

    def test_input_age_feedback_fault_control_gap_and_bad_numbers(self):
        for kwargs, reason in [({'sample_age_s': .26}, 'input_stale'),
                               ({'feedback_age_s': .26}, 'feedback_stale'),
                               ({'has_error': True}, 'controller_fault')]:
            result = self.make_controller().step(command(), IDENTITY, .02, **kwargs)
            self.assertEqual(result['fault_reason'], reason)
            self.assertIsNone(result['relative_target'])
        self.assertEqual(self.make_controller().step(command(), IDENTITY, .11)['fault_reason'], 'control_gap')
        for changes in [{'offset': [float('nan')] * 6}, {'sample_sequence': True},
                        {'active': 'yes'}, {'gripper': 2.}]:
            bad = command()
            bad.update(changes)
            with self.assertRaises(ValueError):
                self.make_controller().step(bad, IDENTITY, .02)
        with self.assertRaises(ValueError):
            control.pose([0.] * 7)

    def test_offline_cli_demo_and_missing_feedback_are_explicit(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / 'result.jsonl'
            result = subprocess.run([sys.executable, str(ROOT / 'scripts/replay_vr_control.py'),
                                     '--demo', '--output', str(output)], capture_output=True, text=True, check=True)
            summary = json.loads(result.stdout)
            self.assertEqual(summary['frames'], 120)
            self.assertTrue(summary['synthetic_feedback'])
            self.assertFalse(summary['transmitted'])
            self.assertFalse(summary['input_lowpass_enabled'])
            rows = [json.loads(line) for line in output.read_text().splitlines()]
            self.assertEqual(rows[-1]['state'], 'inactive')
            self.assertTrue(all(row['offline_only'] and not row['transmitted'] for row in rows))
            source = Path(temporary) / 'input.jsonl'
            source.write_text(json.dumps({'mapped_action': {'_xlevr.anchor_command': command()}}) + '\n')
            result = subprocess.run([sys.executable, str(ROOT / 'scripts/replay_vr_control.py'),
                                     '--input', str(source)], capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('缺少真实末端反馈', result.stderr)


if __name__ == '__main__':
    unittest.main()
