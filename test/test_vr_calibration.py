"""Pose calibration math, read-only IO, persistence and CLI; no hardware."""
import copy
import json
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace
import tempfile
import time
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation
from mmengine import Config

from fluxvla.engines.operators.vr import runtime
from fluxvla.engines.operators.vr.control import calibrate_reference_pose
from test_a10_operator import FakeController

ROOT = Path(__file__).resolve().parents[1]
TOOL = [np.sqrt(.5), np.sqrt(.5), 0., 0.]


class Clock:
    value = 100.

    def now(self):
        return self.value

    def sleep(self, seconds):
        self.value += seconds


class Input:
    def __init__(self, clock, *, mode=None):
        self.clock, self.mode, self.index = clock, mode, 0

    def get_action(self):
        self.index += 1
        seq = 0 if self.mode == 'duplicate' else self.index
        if self.mode == 'rollback' and self.index > 2:
            seq = 0
        age = 1. if self.mode == 'stale' else 0.
        return {'xlevr.enabled': True, 'xlevr.sample_sequence': seq,
                'xlevr.sample_receive_time_s': self.clock.now() - age,
                'xlevr.sample_age_s': age,
                'xlevr.target_position': [0., 0., 0.],
                'xlevr.orientation_quat': [0., 0., 0., 1.]}


class Robot:
    def __init__(self, clock):
        self.clock = clock
        self.moving = self.active = self.fault = self.slow = self.stale = False

    def get_state(self):
        if self.slow:
            self.clock.sleep(.3)
        return SimpleNamespace(
            received_monotonic_s=self.clock.now() - (1. if self.stale else 0.),
            has_error=self.fault, follower_active=self.active,
            raw={'ret_context': {'robot_msg': {'motion': 'Moving' if self.moving else 'Stopped'}}},
            tcp_pq_m=[.1, .2, .3, 0., 0., 0., 1.])


class CalibrationTest(unittest.TestCase):
    def config(self):
        cfg = Config.fromfile(ROOT / 'configs/a10/a10_single_vr_base.py').teleop
        cfg.operator.feedback_position_unit = 'mm'
        return cfg

    def report(self):
        hand = [0., 0., 0., 0., 0., 0., 1.]
        robot = [.1, .2, .3, 0., 0., 0., 1.]
        return calibrate_reference_pose([hand] * 12, [robot] * 12, TOOL)

    def test_noncommuting_fit_sign_equivalence_and_heldout_validation(self):
        base = Rotation.from_euler('xyz', [.3, -.7, .5])
        hand = Rotation.from_euler('xyz', [-.2, .4, .8])
        tool = Rotation.from_quat(TOOL)
        actual = base * hand * tool
        hands = [[0., 0., 0., *(hand.as_quat() * (-1 if i % 2 else 1))] for i in range(12)]
        robots = [[.1, .2, .3, *actual.as_quat()] for _ in range(12)]
        result = calibrate_reference_pose(hands, robots, TOOL)
        fitted = Rotation.from_quat(result['pose_calibration']['vr_to_robot_quat_xyzw'])
        self.assertLess((base.inv() * fitted).magnitude(), 1e-12)
        self.assertEqual(result['quality']['validation_samples'], 6)
        self.assertLess(result['quality']['max_validation_rotation_error_rad'], 1e-12)

    def test_moving_invalid_or_insufficient_poses_rejected(self):
        identity = [0., 0., 0., 0., 0., 0., 1.]
        for side in ('hand', 'robot'):
            hands, robots = np.tile(identity, (12, 1)), np.tile(identity, (12, 1))
            values = hands if side == 'hand' else robots
            values[-1, 0] = .1
            with self.assertRaisesRegex(ValueError, side):
                calibrate_reference_pose(hands, robots, TOOL)
            values[-1] = [0., 0., 0., *Rotation.from_euler('z', 20, degrees=True).as_quat()]
            with self.assertRaisesRegex(ValueError, side):
                calibrate_reference_pose(hands, robots, TOOL)
        with self.assertRaises(ValueError):
            calibrate_reference_pose([identity] * 9, [identity] * 9, TOOL)
        with self.assertRaises(ValueError):
            calibrate_reference_pose([identity] * 12, [identity] * 12, [0.] * 4)

    def test_collection_rejects_stale_rollback_duplicates_faults_and_slow_get(self):
        for mode, error in [('stale', ValueError), ('rollback', ValueError), ('duplicate', TimeoutError),
                            ('moving', ValueError), ('active', ValueError), ('fault', ValueError),
                            ('slow', ValueError), ('old_feedback', ValueError), (None, None)]:
            with self.subTest(mode=mode):
                clock = Clock()
                robot = Robot(clock)
                for attribute in ('moving', 'active', 'fault', 'slow'):
                    setattr(robot, attribute, mode == attribute)
                robot.stale = mode == 'old_feedback'
                with patch.object(runtime.time, 'monotonic', clock.now), patch.object(runtime.time, 'sleep', clock.sleep):
                    if error:
                        with self.assertRaises(error):
                            runtime.collect_pose_calibration(Input(clock, mode=mode), robot, TOOL,
                                                             duration_s=.3, timeout_s=.5)
                    else:
                        result = runtime.collect_pose_calibration(Input(clock), robot, TOOL)
                        self.assertGreaterEqual(result['sample_duration_s'], 1.)
                        self.assertGreaterEqual(result['quality']['sample_count'], 10)

    def test_readonly_loopback_save_reload_backup_and_failure_preserves_file(self):
        cfg = self.config()
        with FakeController() as server, tempfile.TemporaryDirectory() as temporary:
            cfg.operator.update(host='127.0.0.1', allow_motion=True,
                                status_ws_port=server.server.socket.getsockname()[1],
                                udp_port=server.udp.getsockname()[1],
                                joint_udp_port=server.joint_udp.getsockname()[1],
                                cameras={'must_not_open': {}},
                                gripper_transport='usb', gripper={'port': '/must/not/open'})
            path = Path(temporary) / 'calibration.json'
            fake_vr = SimpleNamespace(input=Input(SimpleNamespace(now=time.monotonic)),
                                      connect=lambda: None, close=lambda: None)
            with patch('fluxvla.engines.utils.builder.build_operator_from_cfg', return_value=fake_vr):
                report = runtime.calibrate_pose(cfg, output_path=path, countdown=0, prompt=lambda _: '')
                original = path.read_bytes()
                with patch.object(runtime, 'collect_pose_calibration', return_value=copy.deepcopy(report)):
                    runtime.calibrate_pose(cfg, output_path=path, countdown=0, prompt=lambda _: '')
            self.assertEqual(Path(str(path) + '.bak').read_bytes(), original)
            self.assertTrue(server.commands)
            self.assertEqual(set(server.commands), {'get'})
            with self.assertRaises(TimeoutError):
                server.udp.recv(4096)
            with self.assertRaises(TimeoutError):
                server.joint_udp.recv(4096)
            self.assertTrue(report['read_only'])
            # Load in a normal VR-only runtime; no camera/USB/robot connects.
            cfg.pose_calibration_path = str(path)
            cfg.update(control_mode='reference', pose_mapping='calibrated_absolute')
            loaded = runtime.build_runtime(cfg)
            self.assertFalse(loaded._connected)
            self.assertEqual(loaded.reference_control.pose_calibration, report['pose_calibration'])
            original = path.read_bytes()
            with patch('fluxvla.engines.utils.builder.build_operator_from_cfg', return_value=fake_vr), \
                 patch.object(runtime, 'collect_pose_calibration', side_effect=ValueError('unstable')):
                with self.assertRaisesRegex(ValueError, 'unstable'):
                    runtime.calibrate_pose(cfg, output_path=path, countdown=0, prompt=lambda _: '')
            self.assertEqual(path.read_bytes(), original)
            cfg.operator.host = 'another-controller'
            with self.assertRaisesRegex(ValueError, '地址'):
                runtime.build_runtime(cfg)
            cfg.pose_mapping = 'anchor_relative'
            runtime.build_runtime(cfg)  # Old mode ignores the file entirely.

    def test_cli_calibration_preview_never_connects_or_writes(self):
        from scripts.teleoperate import main
        with tempfile.TemporaryDirectory() as temporary, \
             patch('fluxvla.engines.operators.a10_operator.A10Operator.connect', side_effect=AssertionError('connected')):
            path = Path(temporary) / 'new.json'
            result = main(['--config', str(ROOT / 'configs/a10/a10_single_vr_base.py'),
                           '--calibrate-pose', '--preview', '--pose-calibration', str(path),
                           '--cfg-options', 'teleop.operator.feedback_position_unit=mm',
                           'teleop.pose_mapping=calibrated_absolute'])
            self.assertEqual(result, 0)
            self.assertFalse(path.exists())

    def test_load_rejects_wrong_arm_units_tool_and_malformed_data(self):
        cfg = self.config()
        cfg.update(control_mode='reference', pose_mapping='calibrated_absolute')
        report = dict(self.report(), schema_version=1, kind='vr_pose_calibration', read_only=True,
                      controller_host=cfg.operator.host, feedback_position_unit='mm', vr_arm='right')
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'saved.json'
            cfg.pose_calibration_path = str(path)
            for changes in ({'vr_arm': 'left'}, {'feedback_position_unit': 'm'}, {'kind': 'other'},
                            {'pose_calibration': dict(vr_to_robot_quat_xyzw=[0., 0., 0., 1.],
                                                      hand_to_tool_quat_xyzw=[0., 0., 0., 1.])}):
                path.write_text(json.dumps(dict(report, **changes)))
                with self.assertRaises(ValueError):
                    runtime.build_runtime(cfg)
            path.write_text('not-json')
            with self.assertRaises(ValueError):
                runtime.build_runtime(cfg)
            cfg.pose_calibration.vr_to_robot_quat_xyzw = [0., 0., 0., 1.]
            self.assertFalse(runtime.build_runtime(cfg)._connected)  # Explicit config wins.

    def test_replay_accepts_saved_calibration_report(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'saved.json'
            path.write_text(json.dumps(dict(self.report(), kind='vr_pose_calibration')))
            result = subprocess.run([sys.executable, str(ROOT / 'scripts/replay_vr_control.py'),
                                     '--demo', '--pose-mapping', 'calibrated_absolute',
                                     '--pose-calibration', str(path)], capture_output=True, text=True, check=True)
            report = json.loads(result.stdout)
            self.assertEqual(report['frames'], 120)
            self.assertFalse(report['transmitted'])


if __name__ == '__main__':
    unittest.main()
