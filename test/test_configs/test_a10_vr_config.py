"""Check hardware reuse and command-line overrides through MMEngine's real loader."""
from pathlib import Path
import unittest

from mmengine import Config

ROOT = Path(__file__).resolve().parents[2]
CONFIGS = ROOT / 'configs/a10'


class A10VRConfigTest(unittest.TestCase):
    def test_vr_reuses_hardware_without_changing_other_consumers(self):
        robot = Config.fromfile(CONFIGS / 'a10_dual_arm_operator.py').operator
        devices = Config.fromfile(CONFIGS / 'a10_dual_devices.py')
        vr = Config.fromfile(CONFIGS / 'a10_dual_vr.py').teleop
        expected = robot.to_dict()
        expected.update(allow_motion=True, gripper_transport='controller')
        expected['shared_control']['require_feedback'] = False
        assert vr.operator == expected
        assert robot.allow_motion is False
        assert 'require_feedback' not in robot.shared_control
        for side in ('left', 'right'):
            expected_gripper = devices.grippers[side].to_dict()
            expected_gripper['speed'] = 50
            assert vr.zx_grippers.devices[side] == expected_gripper
            assert devices.grippers[side].speed == 5
        for entry in ('a10_dual_fold_inference.py', 'a10_dual_openpi_remote.py'):
            inference = Config.fromfile(CONFIGS / entry)
            assert inference.operator == robot
            assert inference.cameras == devices.cameras
            assert inference.inference.mode == 'preview'

    def test_vr_overrides_preserve_bindings_and_do_not_change_defaults(self):
        config = Config.fromfile(CONFIGS / 'a10_dual_vr.py')
        config.merge_from_dict({'teleop.operator.host': '127.0.0.42',
                                'teleop.input.mapping.pos_scale': 1.2})
        assert config.teleop.operator.host == '127.0.0.42'
        assert config.teleop.input.mapping.pos_scale == 1.2
        assert config.teleop.operator.arms.left.ee_id == 0
        assert config.teleop.input.hand_to_arm == dict(left='right', right='left')
        assert Config.fromfile(CONFIGS / 'a10_dual_vr.py').teleop.input.mapping.pos_scale == 1.8
