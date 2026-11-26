"""Backend selection must never change the shared robot control configuration."""
import os
import unittest
from pathlib import Path
from unittest.mock import patch

from mmengine import Config
from fluxvla.engines.utils.a10_inference_backends import select_a10_backend

ROOT = Path(__file__).resolve().parents[1]


class BackendTests(unittest.TestCase):
    def test_model_independent_async_preset_builds_without_rtc_capabilities(self):
        from fluxvla.engines import build_runner_from_cfg
        with patch.dict(os.environ, {'A10_MODEL_BACKEND': 'fluxvla'}):
            cfg = Config.fromfile(ROOT / 'configs/a10/a10_dual_openpi_async_inference.py')
        select_a10_backend(cfg)
        cfg.inference.mode = 'preview'
        cfg.inference.cfg = cfg
        runner = build_runner_from_cfg(cfg.inference)
        runner.run_setup()
        self.assertEqual(runner.remote['protocol'], 'openpi_ws')
        self.assertEqual(runner.remote['server_port'], 8001)
        self.assertTrue(runner.async_execution)
        self.assertEqual(runner.execute_horizon, 0)
        self.assertFalse(runner.rtc_config['enabled'])
        self.assertTrue(runner.ros_operator.action_fusion.config['enabled'])
        self.assertEqual(runner.ros_operator.action_fusion.config['method'], 'velocity')
        self.assertEqual(runner.ros_operator.action_fusion.config['gripper_alpha'], .35)
        self.assertTrue(runner.ros_operator.action_fusion.config['continuous_clock'])
        self.assertTrue(runner.ros_operator.action_fusion.config['persistent_sender'])
        self.assertEqual(runner.ros_operator.action_fusion.config['position_gain'], 2.0)
        self.assertEqual(cfg.grippers.left.reversal_confirm_s, .25)
        self.assertEqual(cfg.grippers.right.reversal_deadband_raw, 450)

    def test_onsite_defaults_and_rtc_entrypoint_resolve_to_openpi_8001(self):
        from fluxvla.engines import build_runner_from_cfg
        with patch.dict(os.environ, {}, clear=False):
            os.environ.pop('A10_MODEL_BACKEND', None)
            common = Config.fromfile(ROOT / 'configs/a10/a10_dual_inference.py')
        self.assertEqual(common.model_backend, 'openpi')
        self.assertEqual(common.inference.remote_inference.protocol, 'openpi_ws')
        self.assertEqual(common.inference.remote_inference.server_port, 8001)
        self.assertFalse(common.inference.rtc_config.enabled)
        with patch.dict(os.environ, {'A10_MODEL_BACKEND': 'fluxvla'}):
            rtc = Config.fromfile(ROOT / 'configs/a10/a10_dual_rtc_inference.py')
        select_a10_backend(rtc)
        self.assertEqual(rtc.model_backend, 'openpi')
        self.assertEqual(rtc.inference.remote_inference.protocol, 'openpi_ws')
        self.assertEqual(rtc.inference.remote_inference.server_port, 8001)
        self.assertNotIn('inference_model', rtc)
        self.assertNotIn('dataset', rtc.inference)
        rtc.inference.mode = 'preview'
        rtc.inference.cfg = rtc
        runner = build_runner_from_cfg(rtc.inference)
        runner.run_setup()
        self.assertEqual(runner.remote['protocol'], 'openpi_ws')
        self.assertTrue(runner.rtc_config['enabled'])

    def test_native_compatibility_entry_still_resolves_its_own_pipeline(self):
        with patch.dict(os.environ, {'A10_MODEL_BACKEND': 'openpi'}):
            cfg = Config.fromfile(ROOT / 'configs/a10/a10_dual_fold_inference.py')
        self.assertEqual(cfg.inference.remote_inference.protocol, 'fluxvla_zmq')
        self.assertEqual(cfg.inference.remote_inference.server_port, 5556)
        self.assertEqual(cfg.inference.dataset.type, 'A10NativeInferenceDataset')

    def config(self):
        with patch.dict(os.environ, {'A10_MODEL_BACKEND': 'fluxvla',
                                     'A10_DUAL_CHECKPOINT': '/tmp/absent-a10-50k'}):
            return Config.fromfile(ROOT / 'configs/a10/a10_dual_inference.py')

    def test_backend_switch_preserves_control_and_needs_no_weights(self):
        from fluxvla.engines import build_runner_from_cfg
        native = self.config()
        for selected, protocol, port in [('fluxvla', 'fluxvla_zmq', 5556),
                                         ('openpi', 'openpi_ws', 8001),
                                         ('fluxvla', 'fluxvla_zmq', 5556)]:
            select_a10_backend(native, selected)
            self.assertEqual(native.inference.remote_inference.protocol, protocol)
            self.assertEqual(native.inference.remote_inference.server_port, port)
            self.assertEqual(native.inference.remote_inference.timeout_s, 3600.)
            self.assertIsNone(native.operator.joint_follower_velocity_percent)
            self.assertEqual(native.inference.mode, 'motion')
            self.assertEqual(native.inference.cycles, 1_000_000)
            self.assertEqual(native.inference.execute_horizon, 20)
            self.assertIsNone(native.inference.manual_speed_percent)
            self.assertEqual(native.grippers.left.drop_alarm_policy, 'warn')
            self.assertEqual(native.grippers.right.drop_alarm_policy, 'warn')
            self.assertEqual('inference_model' in native, selected == 'fluxvla')
            native.inference.cfg = native
            runner = build_runner_from_cfg(native.inference)
            runner.run_setup()
            del native.inference.cfg

    def test_environment_and_compatibility_entrypoints(self):
        with patch.dict(os.environ, {'A10_MODEL_BACKEND': 'openpi'}):
            cfg = Config.fromfile(ROOT / 'configs/a10/a10_dual_inference.py')
            select_a10_backend(cfg)
            self.assertEqual(cfg.inference.remote_inference.protocol, 'openpi_ws')
            self.assertNotIn('inference_model', cfg)
            self.assertNotIn('dataset', cfg.inference)
        with patch.dict(os.environ, {'A10_MODEL_BACKEND': 'fluxvla'}):
            cfg = Config.fromfile(ROOT / 'configs/a10/a10_dual_openpi_remote.py')
            select_a10_backend(cfg)
            self.assertEqual(cfg.inference.remote_inference.protocol, 'openpi_ws')
        with patch.dict(os.environ, {'A10_MODEL_BACKEND': 'openpi'}):
            cfg = Config.fromfile(ROOT / 'configs/a10/a10_dual_fold_inference.py')
            select_a10_backend(cfg)
            self.assertEqual(cfg.inference.remote_inference.protocol, 'fluxvla_zmq')
            self.assertEqual(cfg.inference.execute_horizon, 20)

    def test_checkpoint_and_endpoint_override(self):
        cfg = self.config()
        cfg.merge_from_dict({'model_checkpoint_dir': '/tmp/new-a10-bundle'})
        select_a10_backend(cfg)
        self.assertEqual(cfg.inference.dataset.norm_stats,
                         '/tmp/new-a10-bundle/dataset_statistics.json')
        self.assertEqual(cfg.inference.dataset.tokenizer_path,
                         '/tmp/new-a10-bundle/tokenizer')
        cfg.inference.dataset.norm_stats = '/tmp/custom-stats.json'
        cfg.model_checkpoint_dir = '/tmp/another-a10-bundle'
        select_a10_backend(cfg)
        self.assertEqual(cfg.inference.dataset.norm_stats, '/tmp/custom-stats.json')
        options = {'inference.remote_inference.server_host': '192.0.2.12',
                   'inference.remote_inference.server_port': 9001,
                   'inference.remote_inference.timeout_s': 500.}
        cfg.merge_from_dict(options)
        select_a10_backend(cfg, 'openpi', overrides=options)
        self.assertEqual(cfg.inference.remote_inference.server_host, '192.0.2.12')
        self.assertEqual(cfg.inference.remote_inference.server_port, 9001)
        self.assertEqual(cfg.inference.remote_inference.timeout_s, 500.)
        self.assertNotIn('dataset', cfg.inference)
        self.assertNotIn('denormalize_action', cfg.inference)

    def test_overrides_and_model_pipeline(self):
        cfg = self.config()
        cfg.merge_from_dict({'model_backend': 'openpi', 'inference.remote_inference.timeout_s': 500.})
        select_a10_backend(cfg)
        self.assertEqual(cfg.inference.remote_inference.timeout_s, 500.)
        select_a10_backend(cfg, 'fluxvla')
        self.assertEqual(cfg.inference.dataset.norm_type, 'quantile')
        self.assertEqual(cfg.inference.denormalize_action.relative_mask, ([True] * 6 + [False]) * 2)
        self.assertEqual(cfg.inference.prompt, 'fold the black cloth using both arms')
        self.assertEqual(cfg.inference_model.checkpoint_dir, '/tmp/absent-a10-50k')
        with self.assertRaises(ValueError):
            select_a10_backend(cfg, 'invalid')


if __name__ == '__main__':
    unittest.main()
