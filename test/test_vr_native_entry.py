"""Native config/CLI, input abstraction and diagnostic lifecycle contracts."""
import ast
import importlib.abc
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from mmengine import Config
from fluxvla.engines.operators.vr_operator import BaseTeleoperator, VROperator
from fluxvla.engines.operators.vr.runtime import build_runtime, VRTeleopRuntime
from test_vr_operator import FakeVR, FakeMapper, FakeGripper, action

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'configs/a10/a10_single_vr_base.py'


class NativeEntryTest(unittest.TestCase):
    def test_default_input_contract_is_robot_free(self):
        cfg = Config.fromfile(str(CONFIG)).teleop
        runtime = build_runtime(cfg)
        self.assertIsInstance(runtime.teleoperator, BaseTeleoperator)
        self.assertIsNone(runtime.robot)
        self.assertIsNone(runtime.gripper)
        self.assertFalse(runtime.vr.is_connected)
        self.assertTrue((Path(runtime.vr.config.xlevr_path) / 'web-ui/index.html').is_file())
        runtime.close()

    def test_native_operator_maps_and_forwards_events_and_hud(self):
        cfg = Config.fromfile(str(CONFIG)).teleop
        operator = build_runtime(cfg).teleoperator
        raw = {'xlevr.enabled': False, 'xlevr.reset_arm': False}
        with patch.object(operator.input, 'get_action', return_value=raw), \
             patch.object(operator.input, 'get_vr_events', return_value={'stop_recording': True}), \
             patch.object(operator.input, 'send_hud') as hud:
            result = operator.get_action()
            self.assertFalse(result['ee.enabled'])
            self.assertTrue(operator.get_events()['stop_recording'])
            operator.send_feedback({'type': 'hud', 'phase': 'teleop'})
            hud.assert_called_once()

    def test_unknown_mapping_and_wrong_input_arm_rejected(self):
        cfg = Config.fromfile(str(CONFIG)).teleop.input.to_dict()
        cfg.pop('type')
        with self.assertRaises(ValueError):
            VROperator(**dict(cfg, arm='head'))
        with self.assertRaises(ValueError):
            VROperator(**dict(cfg, mapping={'misspelled': 1}))

    def test_vr_only_diagnostics_written_and_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'diagnostics.jsonl'
            runtime = VRTeleopRuntime(FakeVR([{'mapped': action()}]), FakeMapper(),
                                     diagnostics_path=str(path))
            self.assertEqual(runtime.run(max_steps=1), 1)
            record = json.loads(path.read_text())
            self.assertEqual(record['control_frame_index'], 0)
            self.assertIsNone(record['sent_relative_target'])
            self.assertIsNone(record['error'])
            self.assertFalse(record['execute'])
            self.assertIsNone(runtime._diagnostics)

    def test_failure_is_recorded_and_cleanup_preserves_exception(self):
        from test_vr_operator import make_live
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'diagnostics.jsonl'
            runtime, robot = make_live([{'mapped': action(translation=(.03, 0, 0))}])
            runtime.diagnostics_path = str(path)
            with self.assertRaisesRegex(ValueError, 'step limit'):
                runtime.run(max_steps=1)
            record = json.loads(path.read_text())
            self.assertIn('step limit', record['error'])
            self.assertFalse(robot.targets)
            self.assertEqual(robot.events[-1], 'close')

    def test_live_config_uses_shared_usb_driver_without_opening_devices(self):
        cfg = Config.fromfile(str(CONFIG)).teleop
        cfg.vr_only = False
        cfg.operator.allow_motion = True
        cfg.operator.feedback_position_unit = 'mm'
        cfg.operator.gripper_transport = 'usb'
        cfg.operator.gripper = dict(port='/not/a/device', max_width_mm=85.)
        cfg.max_position_step_m = .01
        cfg.max_rotation_step_rad = .1
        runtime = build_runtime(cfg)
        self.assertEqual(runtime.gripper.max_width_mm, 85.)
        self.assertFalse(runtime.gripper.is_connected)
        self.assertFalse(runtime.robot.is_connected)
        self.assertIsNone(runtime.robot.gripper_operator)
        runtime.close()

    def test_cli_preview_with_projects_import_forbidden(self):
        # The new startup path must work even if every legacy project import fails.
        code = """
import importlib.abc, runpy, sys
class BlockProjects(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname == 'fluxvla.projects' or fullname.startswith('fluxvla.projects.'):
            raise AssertionError('Legacy project imported: ' + fullname)
sys.meta_path.insert(0, BlockProjects())
sys.argv = ['scripts/teleoperate.py', '--config', 'configs/a10/a10_single_vr_base.py',
            '--cfg-options', 'teleop.input.network.host_ip=127.0.0.1', '--preview']
runpy.run_path('scripts/teleoperate.py', run_name='__main__')
"""
        result = subprocess.run([sys.executable, '-c', code], cwd=ROOT,
                                text=True, capture_output=True, timeout=90)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('vr_only=True', result.stdout)

    def test_runtime_uses_input_contract_and_resets_reference(self):
        cfg = Config.fromfile(str(CONFIG)).teleop
        runtime = build_runtime(cfg)
        # Replace only hardware input and mapping; the native facade remains real.
        runtime.teleoperator.input = FakeVR([{'mapped': action(enabled=False)}])
        runtime.teleoperator.mapper = FakeMapper()
        runtime.vr = runtime.teleoperator.input
        runtime.mapper = runtime.teleoperator.mapper
        runtime.diagnostics_path = None
        with patch.object(runtime.teleoperator, 'get_action',
                          wraps=runtime.teleoperator.get_action) as read, \
             patch.object(runtime.teleoperator, 'get_events',
                          wraps=runtime.teleoperator.get_events) as events:
            self.assertEqual(runtime.run(max_steps=1), 1)
            read.assert_called_once()
            events.assert_called_once()
        self.assertFalse(runtime.teleoperator.is_connected)
        self.assertGreaterEqual(runtime.mapper.resets, 1)

    def test_setup_packages_new_web_assets_and_licenses(self):
        tree = ast.parse((ROOT / 'setup.py').read_text())
        call = next(n for n in tree.body if isinstance(n, ast.Expr)
                    and isinstance(n.value, ast.Call)
                    and getattr(n.value.func, 'id', '') == 'setup').value
        packages = ast.literal_eval(next(k.value for k in call.keywords if k.arg == 'package_data'))
        self.assertIn('fluxvla.engines.operators.vr', packages)
        self.assertNotIn('fluxvla.projects.a10.vr', packages)
        directory = ROOT / 'fluxvla/engines/operators/vr'
        for pattern in packages['fluxvla.engines.operators.vr']:
            self.assertTrue(list(directory.glob(pattern)), pattern)


if __name__ == '__main__':
    unittest.main()
