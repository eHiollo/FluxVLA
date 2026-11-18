"""Read-only calibration UI backend/HTTP tests; all endpoints are loopback."""
import copy
import asyncio
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import urlopen

import numpy as np
from mmengine import Config
from scipy.spatial.transform import Rotation

from fluxvla.engines.operators.vr.calibration_view import CalibrationView, build_calibration_view
from fluxvla.engines.operators.vr.mapper import XLeVRDeltaEEMapper
from fluxvla.engines.operators.vr.monitor import ReuseHTTPServer, SimpleAPIHandler
from test_a10_operator import FakeController
from test_vr_calibration import Robot

ROOT = Path(__file__).resolve().parents[1]
IDENTITY = [0., 0., 0., 1.]
CALIBRATION = dict(vr_to_robot_quat_xyzw=IDENTITY, hand_to_tool_quat_xyzw=IDENTITY)


class VR:
    def __init__(self):
        self.input = SimpleNamespace(config=SimpleNamespace(server_settings=dict(host_ip='127.0.0.1', https_port=8443)))
        self.mapper = XLeVRDeltaEEMapper(pos_scale=1., vr_to_robot_scale=1., max_vr_speed_m_s=1e9)
        self.sequence = 0
        self.values = {'xlevr.target_position': [0., 0., 0.],
                       'xlevr.orientation_quat': IDENTITY, 'xlevr.enabled': False,
                       'xlevr.verification_tracking': True, 'xlevr.ray_direction_local': [0., 0., -1.]}
        self.age = 0.
        self.closed = False

    def connect(self):
        pass

    def close(self):
        self.closed = True

    def reset_reference(self):
        self.mapper.reset()

    def get_action(self):
        self.sequence += 1
        self.last_raw = dict(copy.deepcopy(self.values), **{
            'xlevr.sample_sequence': self.sequence,
            'xlevr.sample_receive_time_s': time.monotonic() - self.age,
            'xlevr.sample_age_s': self.age, 'xlevr.source_timestamp': float(self.sequence)})
        return self.mapper(self.last_raw)


class CalibrationViewTest(unittest.TestCase):
    def view(self, calibration=None):
        vr = VR()
        robot = Robot(SimpleNamespace(now=time.monotonic))
        return CalibrationView(vr, robot, calibration or CALIBRATION, {'test': True})

    def config(self):
        cfg = Config.fromfile(ROOT / 'configs/a10/a10_single_vr_base.py').teleop
        cfg.operator.feedback_position_unit = 'mm'
        cfg.pose_calibration = copy.deepcopy(CALIBRATION)
        return cfg

    def test_actual_target_ray_noncommuting_and_large_angle_no_governor(self):
        b, c, h = (Rotation.from_euler('xyz', xyz) for xyz in
                   ([.4, -.7, .2], [-.3, .8, .6], [2., -.9, 1.1]))
        view = self.view(dict(vr_to_robot_quat_xyzw=b.as_quat(), hand_to_tool_quat_xyzw=c.as_quat()))
        view.vr.values['xlevr.orientation_quat'] = h.as_quat()
        first = view.step()
        self.assertTrue(first['valid'])
        expected = b * h * c
        self.assertLess((Rotation.from_quat(first['target_pose'][3:]).inv() * expected).magnitude(), 1e-12)
        np.testing.assert_allclose(first['target_axes'], expected.apply(np.eye(3)))
        np.testing.assert_allclose(first['ray_base'], (b * h).apply([0., 0., -1.]))
        self.assertGreater(first['rotation_error_deg'], 20.)
        for _ in range(8):
            self.assertTrue(view.step()['valid'])
        view.vr.values['xlevr.orientation_quat'] = -h.as_quat()
        np.testing.assert_allclose(view.step()['target_axes'], first['target_axes'])
        self.assertFalse(first['transmitted'])

    def test_translation_anchor_release_and_reengage(self):
        view = self.view()
        view.vr.values['xlevr.enabled'] = True
        self.assertTrue(view.step()['translation_anchored'])
        view.vr.values['xlevr.target_position'] = [.01, .02, .03]
        sample = view.step()
        np.testing.assert_allclose(sample['displacement_mm'], [10., 20., 30.])
        view.vr.values['xlevr.enabled'] = False
        self.assertIsNone(view.step()['displacement_mm'])
        view.vr.values['xlevr.enabled'] = True
        np.testing.assert_allclose(view.step()['displacement_mm'], [0., 0., 0.])

    def test_errors_clear_current_data_and_recover_without_motion(self):
        for failure in ('stale', 'moving', 'active', 'fault', 'tracking', 'invalid_quaternion', 'missing_ray'):
            with self.subTest(failure=failure):
                view = self.view()
                self.assertTrue(view.step()['valid'])
                if failure == 'stale':
                    view.vr.age = 1.
                elif failure in ('moving', 'active', 'fault'):
                    setattr(view.robot, failure, True)
                elif failure == 'tracking':
                    view.vr.values['xlevr.verification_tracking'] = False
                elif failure == 'invalid_quaternion':
                    view.vr.values['xlevr.orientation_quat'] = [0.] * 4
                else:
                    view.vr.values['xlevr.ray_direction_local'] = None
                result = view.step()
                if failure == 'missing_ray':
                    self.assertTrue(result['valid'])
                    self.assertIsNone(result['ray_base'])
                else:
                    self.assertFalse(result['valid'])
                    self.assertIsNone(result['target_pose'])
                    self.assertIsNone(result['actual_pose'])

    def test_snapshot_ages_out_during_blocked_producer_and_is_copy(self):
        view = self.view()
        view.step()
        first = view.snapshot()
        first['metadata']['test'] = False
        self.assertTrue(view.snapshot()['metadata']['test'])
        with patch('fluxvla.engines.operators.vr.calibration_view.time.monotonic', return_value=time.monotonic() + 1.):
            expired = view.snapshot()
        self.assertFalse(expired['valid'])
        self.assertIsNone(expired['target_pose'])
        self.assertIsNone(expired['ray_base'])

    def test_rollback_rejected_and_duplicate_cannot_replace_accepted_hand(self):
        view = self.view()
        first = view.step()
        view.vr.sequence -= 1
        view.vr.values['xlevr.orientation_quat'] = Rotation.from_euler('z', 1.).as_quat()
        duplicate = view.step()
        np.testing.assert_allclose(duplicate['target_axes'], first['target_axes'])
        view.vr.sequence = -1
        self.assertFalse(view.step()['valid'])
        self.assertTrue(view.step()['valid'])

    def test_loopback_lifecycle_only_get_even_with_motion_and_accessories_configured(self):
        cfg, vr = self.config(), VR()
        with FakeController() as server, tempfile.TemporaryDirectory() as temporary:
            # Inline calibration takes precedence; no path file may be overwritten.
            path = Path(temporary) / 'calibration.json'
            path.write_text('original calibration', encoding='utf-8')
            before = hashlib.sha256(path.read_bytes()).hexdigest()
            cfg.pose_calibration_path = str(path)
            cfg.operator.update(host='127.0.0.1', allow_motion=True,
                                status_ws_port=server.server.socket.getsockname()[1],
                                udp_port=server.udp.getsockname()[1], joint_udp_port=server.joint_udp.getsockname()[1],
                                cameras={'do_not_open': {}}, gripper_transport='usb', gripper={'port': '/do/not/open'})
            with patch('fluxvla.engines.utils.builder.build_operator_from_cfg', return_value=vr):
                view = build_calibration_view(cfg)
            self.assertFalse(view.robot.allow_motion)
            self.assertIsNone(view.robot.camera_operator)
            self.assertIsNone(view.robot.gripper_operator)
            view.run(max_steps=3)
            self.assertTrue(vr.closed)
            self.assertTrue(server.commands)
            self.assertEqual(set(server.commands), {'get'})
            for udp in (server.udp, server.joint_udp):
                with self.assertRaises(TimeoutError):
                    udp.recv(4096)
            self.assertEqual(before, hashlib.sha256(path.read_bytes()).hexdigest())

    def test_http_observer_is_get_only_and_absent_in_normal_mode(self):
        view = self.view()
        view.step()
        server = ReuseHTTPServer(('127.0.0.1', 0), SimpleAPIHandler)
        server.web_root_path = str(ROOT / 'fluxvla/engines/operators/vr')
        server.config = SimpleNamespace(websocket_port=8442)
        server.calibration_view = view
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        base = f'http://127.0.0.1:{server.server_address[1]}'
        try:
            with urlopen(base + '/calibration.html') as response:
                self.assertIn('只读标定验证', response.read().decode())
                self.assertEqual(response.headers['Cache-Control'], 'no-store')
            with urlopen(base + '/api/calibration-view') as response:
                self.assertTrue(json.load(response)['read_only'])
            with urlopen(base + '/vr-config.js') as response:
                self.assertIn(b'FLUXVLA_VERIFY_ONLY=true', response.read())
            server.calibration_view = None
            for path in ('/api/calibration-view', '/calibration.html'):
                with self.assertRaises(HTTPError) as error:
                    urlopen(base + path)
                self.assertEqual(error.exception.code, 404)
        finally:
            server.shutdown()
            server.server_close()
            worker.join(timeout=2)

    def test_browser_ray_and_tracking_metadata_reaches_input(self):
        from fluxvla.engines.operators.vr.ws_server import VRWebSocketServer
        from fluxvla.engines.operators.vr.monitor import VRMonitorBridge
        from fluxvla.engines.operators.vr.input import VRInput
        async def receive():
            queue = asyncio.Queue()
            server = VRWebSocketServer(queue, SimpleNamespace(vr_to_robot_scale=1.))
            await server.process_single_controller('right', dict(
                position=dict(x=.1, y=.2, z=.3), quaternion=dict(x=0, y=0, z=0, w=1),
                rayDirectionLocal=[0., .6, -.8], verificationTracking=True))
            return await queue.get()
        goal = asyncio.run(receive())
        bridge = VRMonitorBridge(str(ROOT), {})
        bridge._record_goal(goal)
        vr = VRInput(SimpleNamespace(arm='right', fine_trigger_threshold=.5, require_squeeze_to_move=True))
        vr._vr_monitor = bridge
        vr._connected = True
        vr._vr_thread = SimpleNamespace(is_alive=lambda: True)
        result = vr.get_action()
        self.assertEqual(result['xlevr.ray_direction_local'], [0., .6, -.8])
        self.assertTrue(result['xlevr.verification_tracking'])

    def test_cli_preview_and_mode_exclusion(self):
        cmd = [sys.executable, str(ROOT / 'scripts/teleoperate.py'), '--config',
               str(ROOT / 'configs/a10/a10_single_vr_base.py'), '--verify-calibration', '--preview']
        options = ['--cfg-options', 'teleop.operator.host=192.0.2.1',
                   'teleop.operator.feedback_position_unit=mm',
                   'teleop.pose_calibration.vr_to_robot_quat_xyzw=[0,0,0,1]']
        result = subprocess.run(cmd + options, text=True, capture_output=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('未启动服务或连接设备', result.stdout)
        result = subprocess.run(cmd + ['--observe'], text=True, capture_output=True, timeout=30)
        self.assertNotEqual(result.returncode, 0)


if __name__ == '__main__':
    unittest.main()
