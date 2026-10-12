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

    def test_joint_gripper_split_partial_final_chunk_and_dt(self):
        runner, operator = self.make_runner(dry_run=False, action_dt=.04)
        runner.run()
        self.assertEqual(operator.events, ['connect', 'observe', 'observe', 'start',
                                         'execute', 'observe', 'execute', 'stop', 'close'])
        self.assertEqual([len(x['arm_trajectories']['right']) for x in operator.batches], [2, 1])
        for batch in operator.batches:
            self.assertEqual(batch['dt'], .04)
            self.assertFalse(batch['async_exec'])
            np.testing.assert_allclose(batch['arm_trajectories']['right'], .21)
            np.testing.assert_allclose(batch['gripper_trajectories']['right'], 42)
        runner.cleanup()
        self.assertEqual(operator.events.count('close'), 1)

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
                      torch.tensor([[.21] * 6 + [101]]),
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


if __name__ == '__main__':
    unittest.main()
