"""Persistent evidence, partial failures and model-server correlation; no hardware."""
import json
from pathlib import Path
import tempfile
import unittest
import threading

import zmq
from unittest.mock import patch

import numpy as np
import torch

from fluxvla.engines.utils.a10_debug_log import A10DebugLog
from fluxvla.engines.runners.serving.zmq_server import create_server
import test_a10_inference_runner as runner_fixtures


class DebugLogTest(unittest.TestCase):
    def read_events(self, directory):
        return [json.loads(line) for line in (Path(directory) / 'events.jsonl').read_text().splitlines()]

    def test_recorder_copies_arrays_saves_rgb_and_drains(self):
        with tempfile.TemporaryDirectory() as root:
            log = A10DebugLog(role='test', root_dir=root, metadata={'api_key': 'hidden'})
            values = np.array([1., 2.])
            image = np.full((3, 4, 3), [255, 0, 0], dtype=np.uint8)
            log.record('observation', cycle=1, values=values, images={'base': image})
            values[:] = 99
            log.close()
            rows = self.read_events(log.path)
            self.assertEqual(rows[0]['values'], [1., 2.])
            from PIL import Image
            np.testing.assert_array_equal(np.asarray(Image.open(log.path / rows[0]['image_files']['base'])), image)
            summary = json.loads((log.path / 'summary.json').read_text())
            self.assertEqual(summary['dropped_events'], 0)
            self.assertEqual(summary['write_errors'], 0)
            from scripts.a10_log_summary import summarize
            report = summarize(log.path)
            self.assertEqual(report['sampled_images'], 1)
            self.assertEqual(report['log_health']['write_errors'], 0)
            self.assertNotIn('hidden', (log.path / 'metadata.json').read_text())

    def test_runner_persists_stop_reason_actions_and_dry_run(self):
        with tempfile.TemporaryDirectory() as root:
            runner, robot = runner_fixtures.A10RunnerTest().make_runner(debug_log=dict(root_dir=root, image_every_n_cycles=1))
            runner.run()
            events = self.read_events(runner._recorder.path)
            self.assertEqual(next(x for x in events if x['event'] == 'run_end')['reason'], 'max_publish_step')
            plans = [x for x in events if x['event'] == 'planned_actions']
            self.assertEqual([len(x['selected_actions']) for x in plans], [2, 1])
            self.assertIn('nominal_target_velocity_rad_s', plans[0])
            self.assertTrue([x for x in events if x['event'] == 'observation'][0]['image_files'])
            self.assertNotIn('execute', robot.events)
            self.assertEqual(events[-1]['event'], 'cleanup_complete')

    def test_runner_error_keeps_partial_send_and_stop_failure(self):
        with tempfile.TemporaryDirectory() as root:
            runner, robot = runner_fixtures.A10RunnerTest().make_runner(dry_run=False, debug_log=dict(root_dir=root))
            def fail(**kwargs):
                robot.last_trajectory = dict(sent=1, completed=False, trace=[dict(slot=0, joint_sent=True)])
                raise ConnectionError('injected UDP/USB failure')
            robot.execute_trajectory = fail
            robot.failure = 'stop'
            with self.assertRaises(ConnectionError):
                runner.run()
            events = self.read_events(runner._recorder.path)
            error = next(x for x in events if x['event'] == 'run_error')
            self.assertEqual(error['trajectory']['sent'], 1)
            self.assertIn('injected UDP/USB failure', error['traceback'])
            self.assertFalse(next(x for x in events if x['event'] == 'follower_stop')['confirmed'])

    def test_server_retains_raw_actions_and_request_correlation(self):
        class Policy(torch.nn.Module):
            def predict_action(self, **kwargs):
                return torch.ones(1, 3, 7)
        with tempfile.TemporaryDirectory() as root:
            server = create_server(Policy(), device='cpu', host='127.0.0.1', port=0,
                                   dataset=lambda obs: dict(states=torch.zeros(1, 7)),
                                   denormalize_action=lambda data: data['action'][0] + 2,
                                   debug_log=dict(root_dir=root),
                                   model_metadata=dict(checkpoint_dir='checkpoint_test'))
            from fluxvla.engines.runners.serving.serializers import MsgSerializer, ObsSerializer
            context = zmq.Context()
            client = context.socket(zmq.REQ)
            client.setsockopt(zmq.RCVTIMEO, 3000)
            client.setsockopt(zmq.LINGER, 0)
            client.connect(server.socket.getsockopt_string(zmq.LAST_ENDPOINT))
            worker = threading.Thread(target=server.run)
            worker.start()
            try:
                observation = dict(qpos=np.zeros(7), task_description='test', debug_request_id='client-run:4')
                client.send(MsgSerializer.to_bytes(dict(endpoint='predict_action', data=dict(
                    obs_data=ObsSerializer.to_bytes(observation), unnorm_key='a10'))))
                result = MsgSerializer.from_bytes(client.recv())
                self.assertNotIn('error', result)
                self.assertEqual(result['debug']['client_request_id'], 'client-run:4')
                self.assertEqual(result['debug']['model']['checkpoint_dir'], 'checkpoint_test')
            finally:
                server.close()
                worker.join(3)
                client.close()
                context.term()
            self.assertFalse(worker.is_alive())
            rows = self.read_events(server.debug_log.path)
            raw = next(x for x in rows if x['event'] == 'model_prediction')
            returned = next(x for x in rows if x['event'] == 'returned_actions')
            np.testing.assert_array_equal(raw['raw_normalized_actions'], np.ones((1, 3, 7)))
            np.testing.assert_array_equal(returned['absolute_actions_rad_mm'], np.full((3, 7), 3))


if __name__ == '__main__':
    unittest.main()
