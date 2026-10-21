"""RTC bridge regressions without robot hardware, checkpoints or CUDA."""
import io
import json
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import torch
import zmq

from fluxvla.transforms.a10_openpi import OpenPIA10DenormalizeAction, A10RTCDenormalizeAction
from fluxvla.engines.utils.a10_rtc import inject_rtc, validate_rtc
from fluxvla.engines.runners.a10_inference_runner import A10InferenceRunner
from fluxvla.engines.runners.base_inference_runner import BaseInferenceRunner
from fluxvla.engines.runners.serving.zmq_server import create_server
from fluxvla.engines.runners.serving.serializers import MsgSerializer, ObsSerializer
import test_a10_inference_runner as fixtures


CONFIG = dict(enabled=True, method='guidance', prefix_len=1, decay_end=2,
              schedule='exp', max_guidance_weight=5., use_vjp=False)


def denormalizer():
    return OpenPIA10DenormalizeAction(dict(norm_stats=dict(
        state=dict(q01=[-2.] * 7, q99=[2.] * 7),
        actions=dict(q01=[-1.] * 7, q99=[1.] * 7))))


class Policy(torch.nn.Module):
    n_action_steps = 4

    def predict_action(self, states, prev_actions=None, prefix_len=0, rtc_config=None, **kwargs):
        from fluxvla.engines.utils.rtc_guidance import apply_rtc_guidance, compute_prefix_weights
        actions = torch.zeros(1, 4, 7)
        if prev_actions is not None:
            weights = compute_prefix_weights(4, prefix_len, rtc_config['decay_end'])
            actions = apply_rtc_guidance(actions, lambda x: torch.zeros_like(x),
                prev_actions, weights, .5, rtc_config['max_guidance_weight'])
        return actions


class RTCGuidanceTest(unittest.TestCase):
    def test_real_pi05_inherited_guidance_loop_changes_the_prefix(self):
        from fluxvla.models.vlas.pi0_flowmatching import PI0FlowMatching
        state = SimpleNamespace(n_action_steps=4)
        initial = torch.zeros(1, 4, 7)
        previous = torch.full((1, 2, 7), .5)
        guided = PI0FlowMatching._predict_action_guidance_rtc(
            state, initial.clone(), lambda x, t: torch.zeros_like(x), 1,
            torch.tensor(-.1), 'cpu', previous, 1, CONFIG)
        self.assertGreater(float(guided[0, 0, 0]), .3)
        self.assertEqual(float(guided[0, 3, 0]), 0.)

    def test_absolute_prefix_roundtrip_uses_new_observation_for_both_stats_formats(self):
        native = A10RTCDenormalizeAction(dict(a10=dict(action=dict(
            mean=[.1] * 7, std=[.2] * 7))))
        targets = np.array([[.3] * 6 + [40.], [.4] * 6 + [42.]], dtype=np.float32)
        snapshot = targets.copy()
        for transform in (denormalizer(), native):
            for q in (.1, .25):
                state = np.array([q] * 6 + [35.])
                encoded = transform.normalize_absolute(targets, state)
                decoded = transform(dict(action=encoded, state=state))
                np.testing.assert_allclose(decoded, targets, atol=3e-6)
        np.testing.assert_array_equal(targets, snapshot)

    def test_remote_guidance_reaches_model_and_acknowledges_application(self):
        transform = denormalizer()
        server = create_server(Policy(), dataset=lambda obs: dict(states=torch.zeros(1, 7)),
                               denormalize_action=transform, device='cpu', host='127.0.0.1', port=0)
        client = zmq.Context().socket(zmq.REQ)
        context = client.context
        client.setsockopt(zmq.RCVTIMEO, 3000)
        client.setsockopt(zmq.LINGER, 0)
        client.connect(server.socket.getsockopt_string(zmq.LAST_ENDPOINT))
        worker = threading.Thread(target=server.run)
        worker.start()
        try:
            values = []
            for prefix in (None, np.array([[.7] * 6 + [.2], [.8] * 6 + [.2]])):
                observation = dict(qpos=np.array([.2] * 6 + [40.]),
                    a10_rtc_config=json.dumps(CONFIG))
                if prefix is not None:
                    observation['a10_rtc_actions'] = prefix
                client.send(MsgSerializer.to_bytes(dict(endpoint='predict_action', data=dict(
                    obs_data=ObsSerializer.to_bytes(observation), unnorm_key='a10'))))
                result = MsgSerializer.from_bytes(client.recv())
                self.assertNotIn('error', result)
                self.assertEqual(result['debug']['rtc']['applied'], prefix is not None)
                values.append(np.load(io.BytesIO(result['action_data']), allow_pickle=False))
            self.assertGreater(float(np.max(np.abs(values[1] - values[0]))), .1)
            encoded = result['debug']['rtc']['normalized_prefix']
            np.testing.assert_allclose(transform(dict(action=encoded, state=observation['qpos'])), prefix, atol=1e-6)
        finally:
            server.close()
            worker.join(3)
            client.close()
            context.term()
        self.assertFalse(worker.is_alive())

    def test_runner_reuses_only_unexecuted_targets_and_does_not_count_repeat_packets(self):
        runner, robot = fixtures.A10RunnerTest().make_runner(rtc_config=CONFIG, joint_target_repeat=2,
                                                   max_publish_step=4, dry_run=False)
        requests = []
        outputs = torch.tensor([[.21 + i * .005] * 6 + [42.] for i in range(4)])
        def predict(inputs):
            requests.append(inputs.get('a10_rtc_actions'))
            runner.last_profile = dict(server_debug=dict(rtc=dict(enabled=True,
                applied=requests[-1] is not None, prefix_len=0 if requests[-1] is None else 1)))
            return outputs.clone()
        runner._predict_action = A10InferenceRunner._predict_action.__get__(runner)
        with patch.object(BaseInferenceRunner, '_predict_action', side_effect=predict):
            runner.run()
        self.assertIsNone(requests[0])
        np.testing.assert_array_equal(requests[1], outputs.numpy()[2:])
        self.assertEqual(len(robot.batches), 2)

    def test_old_server_and_unsupported_models_fail_before_motion(self):
        runner, robot = fixtures.A10RunnerTest().make_runner(rtc_config=CONFIG, dry_run=False)
        runner._predict_action = A10InferenceRunner._predict_action.__get__(runner)
        runner.last_profile = {}
        with patch.object(BaseInferenceRunner, '_predict_action', return_value=torch.zeros(4, 7)):
            with self.assertRaisesRegex(RuntimeError, 'did not acknowledge'):
                runner.run()
        self.assertNotIn('start', robot.events)
        with self.assertRaisesRegex(ValueError, 'acceleration off'):
            inject_rtc(dict(states=torch.zeros(1, 7)), dict(config=CONFIG), state=None,
                       denormalizer=denormalizer(), model=SimpleNamespace(predict_action=lambda **kw: None))
        with self.assertRaises(ValueError):
            validate_rtc(dict(CONFIG, prefix_len=3), 4, 2)


if __name__ == '__main__':
    unittest.main()
