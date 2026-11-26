"""LongWAM wire, capture history, and common runner regressions without hardware."""

import base64
from collections import deque
import json
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
from urllib.error import HTTPError
from unittest.mock import Mock, patch

import cv2
import numpy as np
from mmengine import Config

from fluxvla.engines.operators.camera_operator import BufferedCameraOperator, CameraHistoryNotReady
from fluxvla.engines.runners.a10_inference_runner import A10DualInferenceRunner
from fluxvla.engines.utils.a10_inference_backends import select_a10_backend
from fluxvla.projects.a10.longwam import LongWamPolicy
from test_a10_framework_loop import components
from test_a10_dual_inference import live_device_fixture


class LongWamTests(unittest.TestCase):
    def setUp(self):
        self.image = np.full((480, 640, 3), [210, 80, 15], dtype=np.uint8)

    def observation(self):
        return {'observation/state': np.zeros(14),
                'image_history': {name: [self.image] * 13 for name in ('top', 'left', 'right')}}

    def response(self, obs_id):
        return dict(obs_id=obs_id, checkpoint='step_000600.pt', actions=np.zeros((32, 14)).tolist(),
                    step_hz=30, first_step_offset=0, server_ms=20., inference_ms=15.)

    def policy(self):
        with patch.object(LongWamPolicy, 'request', return_value=dict(ready=True, checkpoint='step_000600.pt')):
            return LongWamPolicy('127.0.0.1', 8765)

    def test_real_http_roundtrip_and_rgb_jpeg(self):
        captured = []
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                self.send_response(200)
                self.end_headers()
                self.wfile.write(json.dumps(dict(ready=True, checkpoint='step_000600.pt')).encode())

            def do_POST(self):
                captured.append(json.loads(self.rfile.read(int(self.headers['Content-Length']))))
                self.send_response(200)
                self.end_headers()
                self.wfile.write(json.dumps(outer.response(captured[-1]['obs_id'])).encode())

            def log_message(self, *args):
                pass

        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        policy = LongWamPolicy('127.0.0.1', server.server_port)
        try:
            result = policy.infer(self.observation())
            self.assertEqual(np.asarray(result['actions']).shape, (32, 14))
            self.assertEqual(set(captured[0]), {'obs_id', 'state', 'frames'})
            self.assertEqual([len(frames) for frames in captured[0]['frames'].values()], [13] * 3)
            decoded = cv2.imdecode(np.frombuffer(base64.b64decode(captured[0]['frames']['top'][0],
                                                                           validate=True), np.uint8), 1)
            self.assertEqual(decoded.shape, (480, 640, 3))
            np.testing.assert_allclose(decoded[100, 100, ::-1], self.image[100, 100], atol=3)
        finally:
            policy.close()
            server.shutdown()
            server.server_close()
            worker.join(2)

    def test_invalid_response_rejected(self):
        for key, value in [('obs_id', 'other'), ('checkpoint', 'other'), ('step_hz', 15),
                           ('first_step_offset', 1), ('server_ms', -1), ('inference_ms', float('nan')),
                           ('actions', np.zeros((30, 14)).tolist()),
                           ('actions', np.full((32, 14), float('nan')).tolist())]:
            with self.subTest(key=key, value=str(value)[:30]):
                policy = self.policy()
                def respond(path, body):
                    result = self.response(json.loads(body)['obs_id'])
                    result[key] = value
                    return result
                with patch.object(policy, 'request', side_effect=respond), self.assertRaises(ValueError):
                    policy.infer(self.observation())
                policy.close()

    def test_request_validation_and_size_limit(self):
        policy = self.policy()
        for observation in [dict(self.observation(), **{'observation/state': np.zeros(13)}),
                            dict(self.observation(), image_history={'top': []})]:
            with self.assertRaises(ValueError):
                policy.infer(observation)
        with patch.object(policy, 'encode_image', return_value='x' * 500_000), \
                patch.object(policy, 'request') as request, self.assertRaisesRegex(ValueError, '16 MiB'):
            policy.infer(self.observation())
        request.assert_not_called()
        with self.assertRaises(ValueError):
            policy.encode_image(np.zeros((224, 224, 3), np.uint8))
        policy.close()

    def test_unready_health_and_http_error(self):
        with patch.object(LongWamPolicy, 'request', return_value=dict(ready=False)), \
                self.assertRaisesRegex(RuntimeError, 'not ready'):
            LongWamPolicy('127.0.0.1', 8765)
        policy = self.policy()
        error = HTTPError(policy.url, 413, 'too large', {}, BytesIO(b'request too large'))
        with patch.object(policy.opener, 'open', side_effect=error), \
                self.assertRaisesRegex(RuntimeError, 'HTTP 413: request too large'):
            policy.request('/predict', b'{}')
        policy.close()

    def camera(self, count=64):
        camera = BufferedCameraOperator({name: dict(device=name) for name in
                                         ('base_0_rgb', 'left_wrist_0_rgb', 'right_wrist_0_rgb')},
                                        history_size=64, camera_factory=Mock())
        for frames in camera.histories.values():
            frames.extend(dict(sequence=index, received_monotonic_s=10 + index / 30,
                               image=self.image) for index in range(count))
        return camera

    def test_history_selects_original_offsets_and_rejects_gaps(self):
        camera = self.camera()
        window = camera.snapshot_window(12.1)
        for frames in window.values():
            self.assertEqual([frame['sequence'] for frame in frames], list(range(15, 64, 4)))
        with self.assertRaises(CameraHistoryNotReady):
            self.camera(48).snapshot_window(12.)
        with self.assertRaisesRegex(RuntimeError, 'aligned'):
            camera.snapshot_window(13.)
        camera.histories['left_wrist_0_rgb'][30]['received_monotonic_s'] += .2
        camera.histories['base_0_rgb'][31]['received_monotonic_s'] += .2
        with self.assertRaises(RuntimeError):
            camera.snapshot_window(12.1)

    def test_longwam_backend_defaults_to_motion_and_preserves_explicit_mode(self):
        cfg = Config.fromfile('configs/a10/a10_dual_inference.py')
        select_a10_backend(cfg, 'longwam')
        self.assertEqual(cfg.inference.mode, 'motion')
        self.assertEqual(cfg.inference.remote_inference.server_port, 8765)
        self.assertNotIn('inference_model', cfg)
        self.assertNotIn('dataset', cfg.inference)
        runner = A10DualInferenceRunner(**{key: value for key, value in cfg.inference.items() if key != 'type'})
        runner.run_setup()
        self.assertEqual(runner.action_chunk, 32)
        for mode in ('read_only', 'preview'):
            cfg.inference.mode = mode
            select_a10_backend(cfg, 'openpi')
            select_a10_backend(cfg, 'longwam')
            self.assertEqual(cfg.inference.mode, mode)
            select_a10_backend(cfg, 'longwam', {'inference.mode': mode})
            self.assertEqual(cfg.inference.mode, mode)
        select_a10_backend(cfg, 'longwam', {'inference.mode': 'motion'})
        self.assertEqual(cfg.inference.mode, 'motion')
        select_a10_backend(cfg, 'openpi')
        self.assertEqual(cfg.inference.remote_inference.protocol, 'openpi_ws')
        with patch.dict('os.environ', {'A10_MODEL_BACKEND': 'longwam'}):
            for path in ('configs/a10/a10_dual_inference.py',
                         'configs/a10/a10_dual_longwam_remote.py'):
                configured = Config.fromfile(path)
                select_a10_backend(configured)
                self.assertEqual(configured.inference.mode, 'motion')
                self.assertEqual(configured.inference.remote_inference.protocol, 'longwam_http')

    def test_serial_motion_starts_at_zero_regardless_of_inference_delay(self):
        runner = components(execute_horizon=20)
        runner.remote['protocol'] = 'longwam_http'
        frame = runner.ros_operator.get_frame()
        frame['robot_time'] = runner._clock.monotonic()
        runner.observation_window = deque([frame])
        runner._action_ctx = Mock(state=frame['qpos'])
        actions = np.tile(frame['qpos'], (32, 1))
        actions[:, 0] += np.arange(32) * .001
        for delay in (.1, 1.687, 30.):
            with self.subTest(delay=delay):
                runner._clock.sleep(delay)
                with patch.object(runner.ros_operator, 'execute_trajectory') as execute:
                    runner._execute_actions(actions)
                arguments = execute.call_args.kwargs
                np.testing.assert_array_equal(arguments['arm_trajectories']['left'], actions[:20, :6])
                np.testing.assert_array_equal(arguments['arm_trajectories']['right'], actions[:20, 7:13])
                np.testing.assert_array_equal(arguments['gripper_trajectories']['left'], actions[:20, 6])
                np.testing.assert_array_equal(arguments['gripper_trajectories']['right'], actions[:20, 13])
                self.assertEqual(arguments['dt'], 1 / 30)
                self.assertFalse(arguments['async_exec'])

    def test_read_only_framework_sends_history_without_motion(self):
        runner = components()
        runner.mode = 'read_only'
        runner.remote['protocol'] = 'longwam_http'
        sample = runner.ros_operator.devices.sample()
        sample['image_history'] = {name: [dict(image=self.image, sequence=index,
            received_monotonic_s=sample['robot_time'] + (index - 12) * 4 / 30)
            for index in range(13)] for name in runner.ros_operator.bridge.CAMERAS}
        runner._policy = Mock()
        runner._policy.infer.return_value = self.response('test-observation')
        with patch.object(runner.ros_operator.devices, 'sample', return_value=sample):
            actions, paused = runner._run_action_chunk(None)
        self.assertEqual(actions.shape, (32, 14))
        self.assertFalse(paused)
        wire = runner._policy.infer.call_args.args[0]
        self.assertEqual(set(wire['image_history']), {'top', 'left', 'right'})
        self.assertEqual([len(images) for images in wire['image_history'].values()], [13] * 3)
        self.assertEqual(runner.ros_operator.devices.sent, 0)
        self.assertFalse(runner.ros_operator.attempted)

    def test_execution_checks_do_not_require_model_history(self):
        devices, state = live_device_fixture()
        devices.longwam_history = True
        devices.cameras.snapshot_window.side_effect = RuntimeError('history alignment gap')
        devices.check()
        devices.cameras.snapshot_window.assert_not_called()
        devices.cameras.snapshot.assert_called_once()
        self.assertIs(devices._checked_state, state)
        with self.assertRaisesRegex(RuntimeError, 'history alignment gap'):
            devices.sample(include_history=True)

    def test_pre_execution_validation_uses_current_frame_only(self):
        devices, state = live_device_fixture()
        devices.longwam_history = True
        devices.cameras.snapshot_window.side_effect = RuntimeError('history alignment gap')
        runner = components(device=devices)
        runner.ros_operator.settings['longwam_history'] = True
        sample = devices.sample()
        actions = np.tile(runner.ros_operator.bridge.observation(sample, runner.settings)[
            'observation/state'], (32, 1))
        runner.ros_operator.validate_actions(actions)
        devices.cameras.snapshot_window.assert_not_called()
        with self.assertRaisesRegex(RuntimeError, 'history alignment gap'):
            runner.ros_operator.get_frame()


if __name__ == '__main__':
    unittest.main()
