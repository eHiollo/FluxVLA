"""Offline native RTC checkpoint loading and training-preprocessing contracts."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import torch
from mmengine import Config
from safetensors.torch import save_file

from fluxvla.models.vlas.openpi_a10 import load_native_a10_policy
from fluxvla.transforms.a10_openpi import A10RTCDenormalizeAction

ROOT = Path(__file__).resolve().parents[2]
BUNDLE = ROOT / 'checkpoints/FluxVLA-A10-RTC-50k'


class A10RTCInferenceTest(unittest.TestCase):
    def test_native_loader_is_strict_and_uses_all_exported_tensors(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'model.safetensors'
            save_file({'weight': torch.ones(2, 3)}, str(path))
            with patch('fluxvla.engines.build_vla_from_cfg', return_value=torch.nn.Linear(3, 2, bias=False)) as build:
                model = load_native_a10_policy(directory, dict(type='Test', use_lora=True))
            self.assertFalse(build.call_args.args[0]['use_lora'])
            self.assertIsNone(build.call_args.args[0]['pretrained_name_or_path'])
            self.assertTrue(torch.equal(model.weight, torch.ones(2, 3)))
            save_file({'weight': torch.ones(2, 3), 'extra': torch.ones(1)}, str(path))
            with patch('fluxvla.engines.build_vla_from_cfg', return_value=torch.nn.Linear(3, 2, bias=False)):
                with self.assertRaises(RuntimeError):
                    load_native_a10_policy(directory, dict(type='Test'))

    def test_mean_std_actions_restore_joints_but_keep_gripper_absolute(self):
        mean = np.arange(7, dtype=np.float32)
        std = np.arange(1, 8, dtype=np.float32)
        transform = A10RTCDenormalizeAction({'a10': {'action': {'mean': mean, 'std': std}}})
        state = np.full(7, 10, dtype=np.float32)
        result = transform(dict(action=np.ones((1, 30, 32)), state=state))
        expected = mean + std
        expected[:6] += 10
        self.assertEqual(result.shape, (30, 7))
        np.testing.assert_allclose(result, np.broadcast_to(expected, result.shape))
        with self.assertRaises(ValueError):
            transform(dict(action=np.full((30, 32), np.nan), state=state))

    @unittest.skipUnless((BUNDLE / 'config.json').exists(), 'Local model bundle absent')
    def test_bundle_inputs_match_training_mean_std_and_task(self):
        from fluxvla.engines import build_dataset_from_cfg
        cfg = Config.fromfile(str(ROOT / 'configs/a10/a10_rtc_inference.py'))
        self.assertEqual(cfg.inference_model.type, 'NativeA10Policy')
        self.assertEqual(cfg.inference_model.model_config.n_action_steps, 30)
        self.assertEqual(cfg.inference_model.model_config.rtc_training_config.max_delay, 9)
        self.assertIn('Grasp the bottle', cfg.inference.instruction)
        dataset = build_dataset_from_cfg(cfg.inference.dataset)
        state = np.array([-2., -.5, 2.4, -1., -1., 0., 40.], dtype=np.float32)
        obs = dict(qpos=state, base_0_rgb=np.full((31, 47, 3), 255, np.uint8),
                   right_wrist_0_rgb=np.zeros((29, 41, 3), np.uint8),
                   task_description=cfg.inference.instruction)
        batch = dataset(obs)
        stats = json.loads((BUNDLE / 'dataset_statistics.json').read_text())['a10']['proprio']
        expected = (state - np.asarray(stats['mean'], dtype=np.float32)) / (np.asarray(stats['std'], dtype=np.float32) + np.float32(1e-6))
        np.testing.assert_array_equal(batch['states'][0, :7].numpy(), expected)
        self.assertTrue(torch.all(batch['states'][0, 7:] == 0))
        self.assertEqual(tuple(batch['images'].shape), (1, 9, 224, 224))
        self.assertTrue(torch.all(batch['images'][:, 3:6] == -1))
        self.assertEqual(batch['img_masks'].tolist(), [[True, False, True]])
        self.assertEqual(tuple(batch['lang_tokens'].shape), (1, 128))


if __name__ == '__main__':
    unittest.main()
