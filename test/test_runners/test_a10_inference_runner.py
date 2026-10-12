"""Runner contract tests: fake hardware and predictions, no network or CUDA."""

import unittest
from unittest.mock import patch
from types import SimpleNamespace

import numpy as np
import torch

from fluxvla.engines import RUNNERS
from fluxvla.engines.runners.a10_inference_runner import A10InferenceRunner
from fluxvla.engines.runners.base_inference_runner import BaseInferenceRunner


class FakeOperator:
    command_mode = 'joint'

    def __init__(self):
        self.events = []
        self.batches = []
        self.failure = None
        self.frame = dict(
            qpos=np.array([.2] * 6 + [40], dtype=np.float32),
            base_0_rgb=np.zeros((8, 8, 3), dtype=np.uint8),
            right_wrist_0_rgb=np.ones((8, 8, 3), dtype=np.uint8),
            has_error=False, received_monotonic_s=12.)

    def connect(self):
        self.events.append('connect')

    def get_frame(self):
        self.events.append('observe')
        return self.frame

    def start_follower(self):
        self.events.append('start')
        if self.failure == 'start':
            raise TimeoutError('start confirmation')

    def execute_trajectory(self, **kwargs):
        self.events.append('execute')
        if self.failure == 'execute':
            raise ConnectionError('execution failed')
        self.batches.append(kwargs)

    def stop_follower(self):
        self.events.append('stop')
        if self.failure == 'stop':
            raise ConnectionError('stop failed')

    def close(self):
        self.events.append('close')


class A10RunnerTest(unittest.TestCase):
    def make_runner(self, **overrides):
        operator = FakeOperator()
        settings = dict(operator={'type': 'Fake'}, remote_inference={},
                        instruction='pick fruit', action_chunk=4,
                        execute_horizon=2, max_publish_step=3,
                        max_joint_step_rad=.1, enable_mixed_precision=False)
        settings.update(overrides)
        with patch('fluxvla.engines.runners.base_inference_runner.build_operator_from_cfg',
                   return_value=operator), patch.object(BaseInferenceRunner, '_init_zmq_client'):
            runner = A10InferenceRunner(**settings)
        runner.ping = lambda: True
        runner._server_address = 'fake://policy'
        runner._poll_keyboard_pause = lambda: False
        outputs = torch.tensor([[[.21] * 6 + [42]] * 4])
        runner._predict_action = lambda inputs: outputs
        return runner, operator

    def test_registered_and_dry_run_has_no_motion(self):
        self.assertIs(RUNNERS.get('A10InferenceRunner'), A10InferenceRunner)
        runner, operator = self.make_runner()
        runner.run_setup()  # Matches scripts/inference.py; run must not reconnect.
        runner.run()
        self.assertEqual(operator.events, ['connect', 'observe', 'observe', 'observe', 'close'])
        self.assertEqual(runner.last_cycle['steps'], 3)
        self.assertEqual(runner.last_cycle['executed'], 0)
        np.testing.assert_allclose(runner.last_actions[0], [.21] * 6 + [42])

    def test_unlimited_run_stops_on_user_pause_and_cleans_up(self):
        runner, operator = self.make_runner(dry_run=False, max_publish_step=None)
        with patch.object(runner, '_poll_keyboard_pause', side_effect=[False, False, True]):
            runner.run()
        self.assertEqual(runner.last_cycle['steps'], 6)
        self.assertEqual([len(b['arm_trajectories']['right']) for b in operator.batches], [2, 2, 2])
        self.assertEqual(operator.events[-2:], ['stop', 'close'])

    def test_step_limit_reports_offending_action_without_starting(self):
        runner, operator = self.make_runner(
            dry_run=False, max_joint_step_rad=.005,
            execute_horizon=4, max_publish_step=4)
        with self.assertRaisesRegex(
                ValueError, r'action 1/4, joint 1, 0\.010000 rad .* > 0\.005000 rad; '
                r'no motion target sent'):
            runner.run()
        self.assertNotIn('start', operator.events)
        self.assertNotIn('execute', operator.events)

        runner, operator = self.make_runner(
            dry_run=True, max_joint_step_rad=None,
            execute_horizon=4, max_publish_step=4)
        with self.assertLogs('fluxvla.engines.runners.a10_inference_runner',
                             level='INFO') as logs:
            runner.run()
        self.assertTrue(any('dry-run planned largest joint step: 0.010000 rad' in line
                            and 'action 1/4, joint 1' in line for line in logs.output))
        self.assertNotIn('start', operator.events)

    def test_joint_gripper_split_partial_final_chunk_and_dt(self):
        runner, operator = self.make_runner(dry_run=False, action_dt=.04)
        runner.run()
        self.assertEqual(operator.events, ['connect', 'observe', 'observe', 'start',
                                         'execute', 'observe', 'execute', 'stop', 'close'])
        self.assertEqual([len(x['arm_trajectories']['right']) for x in operator.batches], [2, 1])
        np.testing.assert_allclose(runner.last_cycle['observed_qpos'], [.2] * 6 + [40])
        np.testing.assert_allclose(runner.last_cycle['last_action'], [.21] * 6 + [42])
        np.testing.assert_allclose(runner.last_cycle['tracking_error_at_observation_rad'], [-.01] * 6,
                                   atol=1e-7)
        for batch in operator.batches:
            self.assertEqual(batch['dt'], .04)
            self.assertFalse(batch['async_exec'])
            np.testing.assert_allclose(batch['arm_trajectories']['right'], .21)
            np.testing.assert_allclose(batch['gripper_trajectories']['right'], 42)
        runner.cleanup()
        self.assertEqual(operator.events.count('close'), 1)

    def test_model_gripper_overshoot_clamps_without_changing_joints(self):
        runner, operator = self.make_runner(dry_run=False, max_publish_step=4)
        predicted = torch.tensor([[.21] * 6 + [width]
                                  for width in (-.5, 42., 85., 86.)])
        runner._predict_action = lambda inputs: predicted
        with self.assertLogs('fluxvla.engines.runners.a10_inference_runner', level='WARNING') as logs:
            runner.run()
        widths = np.concatenate([b['gripper_trajectories']['right'] for b in operator.batches])
        # Both cycles predict the same chunk and select its first two rows.
        np.testing.assert_allclose(widths, [0., 42., 0., 42.])
        self.assertTrue(any('clamping 2 rows' in line for line in logs.output))
        for batch in operator.batches:
            np.testing.assert_allclose(batch['arm_trajectories']['right'], .21)

        runner, _ = self.make_runner()
        runner._use_remote = True
        np.testing.assert_allclose(runner._postprocess_actions(predicted)[:, 6], [0., 42., 85., 85.])
        runner.cleanup()

    def test_invalid_observations_close_without_starting(self):
        for field, value in [('qpos', None), ('has_error', True),
                             ('base_0_rgb', np.zeros((3, 8, 8))),
                             ('right_wrist_0_rgb', None)]:
            with self.subTest(field=field):
                runner, operator = self.make_runner(dry_run=False)
                operator.frame[field] = value
                with self.assertRaises((ValueError, RuntimeError)):
                    runner.run()
                self.assertNotIn('start', operator.events)
                self.assertEqual(operator.events[-1], 'close')

    def test_bad_actions_never_start_follower(self):
        for value in [torch.zeros(0, 7), torch.zeros(2, 4, 7),
                      torch.full((4, 7), float('nan')),
                      torch.tensor([[.21] * 6 + [float('inf')]]),
                      torch.tensor([[1.] * 6 + [40]])]:
            with self.subTest(shape=value.shape):
                runner, operator = self.make_runner(dry_run=False)
                runner._predict_action = lambda inputs: value
                with self.assertRaises(ValueError):
                    runner.run()
                self.assertNotIn('start', operator.events)
                self.assertEqual(operator.events[-1], 'close')

    def test_start_and_execution_failure_stop_and_close(self):
        for failure in ['start', 'execute']:
            runner, operator = self.make_runner(dry_run=False)
            operator.failure = failure
            with self.assertRaises((TimeoutError, ConnectionError)):
                runner.run()
            self.assertEqual(operator.events[-2:], ['stop', 'close'])

    def test_stop_failure_is_reported_and_resources_close(self):
        runner, operator = self.make_runner(dry_run=False)
        operator.failure = 'stop'
        with self.assertLogs('fluxvla.engines.runners.a10_inference_runner', level='ERROR'):
            runner.run()
        self.assertEqual(operator.events[-1], 'close')

    def test_keyboard_pause_and_interrupt_cleanup(self):
        runner, operator = self.make_runner(dry_run=False)
        runner._poll_keyboard_pause = lambda: True
        runner.run()
        self.assertEqual(len(operator.batches), 1)
        runner, operator = self.make_runner()
        with patch.object(runner, '_predict_action', side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                runner.run()
        self.assertEqual(operator.events[-1], 'close')

    def test_local_denormalizer_uses_original_state_once(self):
        runner, operator = self.make_runner()
        runner._use_remote = False
        runner.dataset = lambda obs: {'state': obs['qpos'] * 0}
        runner._preprocess('pick fruit')
        original = operator.frame['qpos'].copy()
        def denormalize(data):
            np.testing.assert_array_equal(data['state'], original)
            return data['action'][:, :7] + data['state']
        runner.denormalize_action = denormalize
        actions = runner._postprocess_actions(torch.zeros(1, 4, 32))
        self.assertEqual(actions.shape, (4, 7))
        np.testing.assert_array_equal(actions[0], original)
        runner.cleanup()

    def test_invalid_configuration_rejected(self):
        for options in [dict(execute_horizon=5), dict(action_dt=0),
                        dict(dry_run=False, max_joint_step_rad=None),
                        dict(gripper_mm_range=(100, 0)), dict(max_publish_step=0)]:
            with self.subTest(options=options), self.assertRaises(ValueError):
                self.make_runner(**options)

    def test_local_config_model_and_device_transfer(self):
        operator = FakeOperator()
        model = torch.nn.Linear(1, 1)
        calls = []
        def predict(**inputs):
            calls.append(inputs['states'].device)
            return torch.tensor([[[.21] * 6 + [42]] * 4])
        model.predict_action = predict
        dataset = lambda obs: {'states': torch.tensor(obs['qpos'][None])}
        with patch('fluxvla.engines.runners.base_inference_runner.build_operator_from_cfg',
                   return_value=operator), \
                patch('fluxvla.engines.build_vla_from_cfg', return_value=model), \
                patch('fluxvla.engines.build_dataset_from_cfg', return_value=dataset), \
                patch('fluxvla.engines.build_transform_from_cfg',
                      return_value=lambda data: data['action']), \
                patch.object(BaseInferenceRunner, 'run_setup'):
            runner = A10InferenceRunner(
                model_from_config=True, cfg=SimpleNamespace(inference_model={}),
                dataset={}, denormalize_action={}, operator={'type': 'Fake'},
                instruction='pick fruit', action_chunk=4, execute_horizon=2,
                max_publish_step=2, enable_mixed_precision=False)
            runner._poll_keyboard_pause = lambda: False
            runner.run()
        self.assertEqual(calls, [torch.device('cpu')])
        self.assertEqual(runner.last_actions.shape, (2, 7))
        self.assertNotIn('start', operator.events)


class A10EntrypointTest(unittest.TestCase):
    def test_script_selects_its_worktree_from_another_directory(self):
        import subprocess
        import sys
        import tempfile
        from pathlib import Path
        root = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as directory:
            cfg = Path(directory) / 'invalid_operator.py'
            cfg.write_text("inference=dict(type='A10InferenceRunner', remote_inference={}, "
                           "operator=dict(type='A10Operator', host=''))\n")
            result = subprocess.run([sys.executable, str(root / 'scripts/inference.py'),
                                     '--config', str(cfg)], cwd=directory,
                                    capture_output=True, text=True, timeout=60)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('host must be a controller hostname or IP', result.stderr)
        self.assertIn(str(root / 'fluxvla/engines/operators/a10_operator.py'), result.stderr)
        self.assertIn(str(root / 'fluxvla/engines/runners/a10_inference_runner.py'), result.stderr)


if __name__ == '__main__':
    unittest.main()
