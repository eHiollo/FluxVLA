"""Offline OpenPI compatibility checks; real GPU forward is opt-in."""

import ast
import io
import logging
import os
import tempfile
import threading

import zmq
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np
import torch
from mmengine import Config
from safetensors.torch import save_file

from fluxvla.datasets.a10_openpi_inference import OpenPIA10InferenceDataset
from fluxvla.engines import build_dataset_from_cfg, build_transform_from_cfg, build_vla_from_cfg
from fluxvla.models.vlas.openpi_a10 import checkpoint_mapping
from fluxvla.transforms.a10_openpi import OpenPIA10DenormalizeAction, load_a10_stats

ROOT = Path(__file__).resolve().parents[2]
OPENPI = ROOT.parent / 'openpi'


def observation():
    return dict(qpos=np.array([-.2, .3, 1., -.4, .5, -.6, 40], dtype=np.float32),
                base_0_rgb=np.arange(17 * 31 * 3, dtype=np.uint8).reshape(17, 31, 3),
                right_wrist_0_rgb=np.full((19, 11, 3), 172, dtype=np.uint8),
                task_description='Pick_the fruit\non table')


def reference_method(path, class_name, method):
    """Execute the actual reference function without importing JAX or hardware."""
    tree = ast.parse(path.read_text())
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name)
    function = next(n for n in node.body if isinstance(n, ast.FunctionDef) and n.name == method)
    function.decorator_list = []
    source = 'from __future__ import annotations\n' + ast.unparse(function)
    namespace = dict(np=np, logging=logging)
    exec(compile(source, str(path), 'exec'), namespace)
    return namespace[method]


class A10OpenPITest(unittest.TestCase):
    def setUp(self):
        self.stats = {'norm_stats': {
            'state': {'q01': [-1.] * 7, 'q99': [2.] * 7},
            'actions': {'q01': [-.1] * 6 + [0.], 'q99': [.1] * 6 + [100.]}}}

    def test_stats_and_denormalization_preserve_source(self):
        stats = load_a10_stats(self.stats)
        np.testing.assert_array_equal(stats['proprio']['q01'], [-1.] * 7)
        denorm = OpenPIA10DenormalizeAction(self.stats)
        state = observation()['qpos']
        result = denorm(dict(action=np.zeros((1, 3, 32)), state=state))
        self.assertEqual(result.shape, (3, 7))
        for row in result:
            np.testing.assert_allclose(row[:6], state[:6] + 5e-7, atol=1e-7)
            self.assertAlmostEqual(float(row[6]), 50., places=5)
        with self.assertRaises(ValueError):
            denorm(dict(action=np.full((3, 7), np.nan), state=state))

    def test_image_and_prompt_contract(self):
        tokenizer = SimpleNamespace(encode=lambda prompt, add_bos: [2, 7, 9])
        with patch('sentencepiece.SentencePieceProcessor', return_value=tokenizer):
            dataset = OpenPIA10InferenceDataset(self.stats, 'unused')
        data = dataset(observation())
        self.assertEqual(tuple(data['images'].shape), (1, 9, 224, 224))
        self.assertEqual(tuple(data['states'].shape), (1, 32))
        self.assertEqual(data['img_masks'].tolist(), [[True, False, True]])
        self.assertTrue(torch.all(data['images'][:, 3:6] == -1))
        self.assertEqual(data['lang_tokens'][0, :4].tolist(), [2, 7, 9, 0])
        self.assertEqual(int(data['lang_masks'].sum()), 3)
        self.assertTrue(torch.all(data['states'][:, 7:] == 0))
        bad = observation()
        bad['qpos'] = np.zeros(6)
        with self.assertRaises(ValueError):
            dataset(bad)

    def test_strict_mapping_rejects_shape_and_unused_weights(self):
        model = torch.nn.Linear(2, 3, bias=False)
        model._mapped_name_candidates = lambda name: [('test', 'source.' + name)]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'model.safetensors'
            save_file({'source.weight': torch.ones(3, 2)}, str(path))
            self.assertEqual(checkpoint_mapping(model, path), {'weight': 'source.weight'})
            save_file({'source.weight': torch.ones(2, 3)}, str(path))
            with self.assertRaises(ValueError):
                checkpoint_mapping(model, path)
            save_file({'source.weight': torch.ones(3, 2), 'extra': torch.ones(1)}, str(path))
            with self.assertRaises(ValueError):
                checkpoint_mapping(model, path)

    @unittest.skipUnless((OPENPI / 'src/openpi/transforms.py').exists(), 'OpenPI reference absent')
    def test_matches_actual_openpi_transforms_and_tokenizer(self):
        cfg = Config.fromfile(str(ROOT / 'configs/a10/a10_openpi_inference.py'))
        if not Path(cfg.inference.dataset.tokenizer_path).exists():
            self.skipTest('Original tokenizer absent')
        dataset = build_dataset_from_cfg(cfg.inference.dataset)
        denorm = build_transform_from_cfg(cfg.inference.denormalize_action)
        raw = observation()
        actual = dataset(raw)
        stats = SimpleNamespace(**dataset.stats)
        normalize = reference_method(OPENPI / 'src/openpi/transforms.py', 'Normalize', '_normalize_quantile')
        state = normalize(None, raw['qpos'], stats)
        tokenize = reference_method(OPENPI / 'src/openpi/models/tokenizer.py', 'PaligemmaTokenizer', 'tokenize')
        ids, mask = tokenize(SimpleNamespace(_max_len=128, _tokenizer=dataset.tokenizer),
                             raw['task_description'], state)
        np.testing.assert_array_equal(actual['lang_tokens'][0], ids)
        np.testing.assert_array_equal(actual['lang_masks'][0], mask)
        np.testing.assert_array_equal(actual['states'][0, :7], state.astype(np.float32))
        unnormalize = reference_method(OPENPI / 'src/openpi/transforms.py', 'Unnormalize', '_unnormalize_quantile')
        normalized = np.linspace(-1.5, 1.5, 3 * 32, dtype=np.float32).reshape(3, 32)
        expected = unnormalize(None, normalized, SimpleNamespace(q01=denorm.low, q99=denorm.high))[:, :7]
        expected[:, :6] += raw['qpos'][:6]
        np.testing.assert_array_equal(denorm(dict(action=normalized, state=raw['qpos'])), expected.astype(np.float32))
        camera_source = ROOT.parent / 'A10_new/client/usb_camera.py'
        if camera_source.exists():
            tree = ast.parse(camera_source.read_text())
            function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == '_resize_with_pad')
            namespace = dict(np=np, cv2=cv2)
            exec(compile(ast.Module(body=[function], type_ignores=[]), str(camera_source), 'exec'), namespace)
            expected = namespace['_resize_with_pad'](raw['base_0_rgb'], 224, 224)
            expected = expected.transpose(2, 0, 1).astype(np.float32) / 255. * 2. - 1.
            np.testing.assert_array_equal(actual['images'][0, :3], expected)

    @unittest.skipUnless(os.environ.get('A10_TEST_REAL_MODEL') == '1', 'Opt-in real GPU model test')
    def test_real_checkpoint_forward_and_service(self):
        from fluxvla.engines.runners.serving.zmq_server import create_server
        cfg = Config.fromfile(str(ROOT / 'configs/a10/a10_openpi_inference.py'))
        dataset = build_dataset_from_cfg(cfg.inference.dataset)
        denorm = build_transform_from_cfg(cfg.inference.denormalize_action)
        model = build_vla_from_cfg(cfg.inference_model)
        self.assertEqual(len(list(model.parameters())), 811)
        server = create_server(model, dataset, denorm, task_suite_name='a10',
                               host='127.0.0.1', port=0, device='cuda:0',
                               mixed_precision_dtype=torch.float32)
        from fluxvla.engines.runners.serving.serializers import MsgSerializer, ObsSerializer
        context = zmq.Context()
        client = context.socket(zmq.REQ)
        client.setsockopt(zmq.RCVTIMEO, 30000)
        client.setsockopt(zmq.SNDTIMEO, 30000)
        client.setsockopt(zmq.LINGER, 0)
        thread = None
        try:
            # Local native model path, then actual serialized ZMQ request.
            torch.manual_seed(7)
            inputs = {k: v.cuda() for k, v in dataset(observation()).items()}
            with torch.inference_mode():
                raw = model.predict_action(**inputs).cpu().numpy()
            local_actions = denorm(dict(action=raw, state=observation()['qpos']))
            self.assertEqual(local_actions.shape, (30, 7))
            self.assertTrue(np.isfinite(local_actions).all())
            client.connect(server.socket.getsockopt_string(zmq.LAST_ENDPOINT))
            thread = threading.Thread(target=server.run)
            thread.start()
            torch.manual_seed(7)
            client.send(MsgSerializer.to_bytes(dict(endpoint='predict_action', data=dict(
                obs_data=ObsSerializer.to_bytes(observation(), compress=True), unnorm_key='a10'))))
            result = MsgSerializer.from_bytes(client.recv())
            self.assertNotIn('error', result)
            actions = np.load(io.BytesIO(result['action_data']), allow_pickle=False)
            self.assertEqual(actions.shape, (30, 7))
            self.assertTrue(np.isfinite(actions).all())
            np.testing.assert_allclose(actions, local_actions, rtol=1e-5, atol=1e-5)
            print('REAL A10: 811 parameters, local + ZMQ=(30,7), finite, parity PASS', flush=True)
        finally:
            server.close()
            if thread is not None:
                thread.join(2)
                self.assertFalse(thread.is_alive())
            else:
                server.socket.close(linger=0)
                server.context.term()
            client.close()
            context.term()



if __name__ == '__main__':
    unittest.main()
