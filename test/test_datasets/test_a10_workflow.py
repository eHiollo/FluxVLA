"""A10 integration tests that never connect to physical hardware."""

import copy
import io
import json
import socketserver
import threading
import unittest
from unittest.mock import patch

import numpy as np
import zmq

from fluxvla.engines.runners.serving.serializers import ObsSerializer
from fluxvla.engines.runners.serving.zmq_server import PolicyServer
from fluxvla.projects.a10 import bridge
from fluxvla.projects.a10.config import load_project, validate_project
from fluxvla.projects.a10.follower import A10LegacyFollower
from fluxvla.projects.a10.transforms import (A10CameraSlots,
                                             A10DenormalizeAction)


class Controller(socketserver.ThreadingTCPServer):
    allow_reuse_address = True


class ControllerHandler(socketserver.StreamRequestHandler):
    def handle(self):
        for line in self.rfile:
            command = line.decode().strip()
            if command == 'GET_POLICY_STATUS':
                result = {'idle': True, 'remaining': 0}
            elif command == 'GET_FOLLOWER_STATE':
                result = {'q': [0.0] * 7}
            elif command.startswith('SET_JOINTS_BATCH '):
                payload = json.loads(command.split(' ', 1)[1])
                self.server.actions.append(payload['actions'])
                result = {'accepted': True}
            elif command.startswith(('SET_EE_DELTA ', 'SET_EE_ANCHOR ')) or command == 'reset':
                self.server.commands.append(command)
                continue  # Legacy A10 does not acknowledge these commands.
            else:
                result = {'error': 'unknown'}
            self.wfile.write(json.dumps(result).encode() + b'\n')


class FakeCamera:
    def read_rgb(self):
        return np.zeros((8, 8, 3), dtype=np.uint8)

    def close(self):
        pass


class A10WorkflowTest(unittest.TestCase):
    def test_legacy_ee_delta_and_reset_wire_protocol(self):
        controller = Controller(('127.0.0.1', 0), ControllerHandler)
        controller.actions, controller.commands = [], []
        thread = threading.Thread(target=controller.serve_forever, daemon=True)
        thread.start()
        follower = A10LegacyFollower('127.0.0.1', controller.server_address[1])
        try:
            follower.send_ee_delta([.001, 0, 0, 0, 0, 0, -.5])
            np.testing.assert_array_equal(follower.state(), np.zeros(7))
            follower.reset()
            follower.state()
            payload = json.loads(controller.commands[0].split(' ', 1)[1])
            self.assertEqual(payload['actions'], [.001, 0, 0, 0, 0, 0, -.5])
            self.assertEqual(controller.commands[1], 'reset')
            with self.assertRaises(ValueError):
                follower.send_ee_delta([0] * 6 + [50])
        finally:
            follower.close()
            controller.shutdown()
            controller.server_close()
            thread.join(timeout=2)

    def test_camera_and_action_conventions(self):
        sample = {'images': [np.ones((3, 8, 8), dtype=np.uint8),
                             np.ones((3, 8, 8), dtype=np.uint8) * 2]}
        result = A10CameraSlots()(sample)
        self.assertEqual(result['img_masks'].tolist(), [True, False, True])
        self.assertEqual([int(x.sum()) for x in result['images']],
                         [192, 0, 384])
        stats = {'a10': {'action': {'q01': [-1.0] * 7,
                                   'q99': [1.0] * 7}}}
        action = A10DenormalizeAction(stats)(
            {'action': np.zeros((1, 2, 32), dtype=np.float32),
             'state': np.array([1, 2, 3, 4, 5, 6, 50], dtype=np.float32)})
        self.assertEqual(action.shape, (1, 2, 7))
        np.testing.assert_array_equal(action[0, 0], [1, 2, 3, 4, 5, 6, 0])
        cfg = copy.deepcopy(load_project())
        cfg['model']['action_source'] = 'action'
        with self.assertRaises(ValueError):
            validate_project(cfg)

    def test_one_closed_loop_cycle_against_fake_services(self):
        controller = Controller(('127.0.0.1', 0), ControllerHandler)
        controller.actions = []
        controller_thread = threading.Thread(target=controller.serve_forever,
                                             daemon=True)
        controller_thread.start()

        server = PolicyServer(host='127.0.0.1', port=0)
        endpoint = server.socket.getsockopt_string(zmq.LAST_ENDPOINT)
        policy_port = int(endpoint.rsplit(':', 1)[1])

        def predict(obs_data, unnorm_key):
            observation = ObsSerializer.from_bytes(obs_data)
            self.assertEqual(unnorm_key, 'a10')
            self.assertEqual(observation['qpos'].shape, (7,))
            self.assertEqual(observation['base_0_rgb'].shape, (8, 8, 3))
            actions = np.array([[[0.05] * 6 + [50.0]]],
                               dtype=np.float32)
            buffer = io.BytesIO()
            np.save(buffer, actions, allow_pickle=False)
            return {'action_data': buffer.getvalue(), 'infer_time': 0.001}

        server.register_endpoint('predict_action', predict)
        policy_thread = threading.Thread(target=server.run, daemon=True)
        policy_thread.start()
        cfg = copy.deepcopy(load_project())
        cfg['robot'].update(host='127.0.0.1', port=controller.server_address[1],
                            allow_motion=True, max_joint_step_rad=0.1,
                            gripper_mm_range=[0.0, 100.0],
                            post_idle_settle_s=0.0)
        cfg['inference'].update(host='127.0.0.1', port=policy_port,
                                timeout_s=2.0)
        try:
            with patch.object(bridge, '_cameras', return_value={
                'base_0_rgb': FakeCamera(),
                'right_wrist_0_rgb': FakeCamera()}):
                self.assertEqual(bridge.run(cfg, cycles=1), 0)
            self.assertEqual(len(controller.actions), 1)
            self.assertEqual(len(controller.actions[0]), 1)
            self.assertAlmostEqual(controller.actions[0][0][0], 0.05)
            with self.assertRaisesRegex(ValueError, 'joint step'):
                bridge.checked_actions(
                    np.array([[0.5] * 6 + [50.0]], dtype=np.float32),
                    np.zeros(7, dtype=np.float32), max_chunk=10,
                    max_step=0.1, gripper_range=[0, 100])
        finally:
            server.close()
            policy_thread.join(timeout=2)
            controller.shutdown()
            controller.server_close()
            controller_thread.join(timeout=2)


if __name__ == '__main__':
    unittest.main()
