"""Native platform camera, writer and VR service regression tests."""

import copy
import json
import ssl
import tempfile
import time
import unittest
import urllib.request
from pathlib import Path
from unittest.mock import patch

import numpy as np
from websockets.sync.client import connect

from fluxvla.datasets.parquet_dataset_v3 import ParquetDatasetV3
from fluxvla.projects.a10.cameras import RGBCamera
from fluxvla.projects.a10.config import load_project
from fluxvla.projects.a10.recording import EpisodeWriter
from fluxvla.projects.a10.vr.runtime import build_vr


class NativePlatformTest(unittest.TestCase):
    def test_camera_rgb_rotation(self):
        bgr = np.array([[[1, 2, 3], [4, 5, 6]],
                        [[7, 8, 9], [10, 11, 12]]], dtype=np.uint8)
        capture = unittest.mock.Mock()
        capture.read.return_value = (True, bgr)
        camera = RGBCamera({'device': 0, 'rotate_180': True})
        rgb = camera._read(capture)
        np.testing.assert_array_equal(rgb, bgr[::-1, ::-1, ::-1])

    def test_writer_resume_and_discard(self):
        cameras = {name: {'height': 16, 'width': 16}
                   for name in ('base_0_rgb', 'right_wrist_0_rgb')}
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / 'data'
            writer = EpisodeWriter(root, cameras)
            writer.start('first_task')
            for index in range(4):
                writer.append([index * .01] * 6 + [50.], [1.] + [0.] * 7,
                              {name: np.full((16, 16, 3), index * 20,
                                             dtype=np.uint8) for name in cameras})
            self.assertEqual(writer.save(), 0)
            writer = EpisodeWriter(root, cameras, resume=True)
            writer.start('discarded_task')
            writer.append([0.] * 7, [0.] * 8,
                          {name: np.zeros((16, 16, 3), dtype=np.uint8) for name in cameras})
            writer.discard()
            self.assertEqual(json.loads((root / 'meta/info.json').read_text())['total_frames'], 4)
            writer.start('second_task')
            for _ in range(3):
                writer.append([.1] * 6 + [60.], [0.] * 8,
                              {name: np.full((16, 16, 3), 100,
                                             dtype=np.uint8) for name in cameras})
            self.assertEqual(writer.save(), 1)
            dataset = ParquetDatasetV3(str(root), transforms=[], statistic_name='a10',
                                       action_window_size=30, window_start_idx=1,
                                       expose_index=True)
            sample = dataset.__getitem__(2, {'a10': dataset.stats[0]['stats']})
            self.assertEqual(sample['actions'].shape, (30, 7))
            self.assertEqual(sample['action_masks'].tolist(), [1.] + [0.] * 29)
            self.assertAlmostEqual(float(sample['actions'][-1, 0]), .03, places=6)
            self.assertEqual(dataset.tasks[0], {0: 'first_task', 1: 'second_task'})
            self.assertEqual(len(dataset), 7)

    def test_native_vr_https_websocket_and_cleanup(self):
        cfg = copy.deepcopy(load_project())
        with tempfile.TemporaryDirectory() as temporary:
            cfg['vr_server'].update(host_ip='127.0.0.1', https_port=0,
                                    websocket_port=0, tls_directory=temporary)
            vr, mapper = build_vr(cfg)
            vr.connect()
            try:
                monitor = vr._vr_monitor
                https_port = monitor.https_server.httpd.server_address[1]
                ws_port = monitor.vr_server.server.sockets[0].getsockname()[1]
                context = ssl._create_unverified_context()  # Local test certificate only.
                with urllib.request.urlopen(f'https://127.0.0.1:{https_port}/',
                                            context=context, timeout=2) as response:
                    self.assertIn(b'FluxVLA', response.read())
                packet = {'rightController': {'position': {'x': 0., 'y': 0., 'z': 0.},
                          'quaternion': {'x': 0., 'y': 0., 'z': 0., 'w': 1.},
                          'gripActive': True, 'trigger': 0., 'thumbstick': {}}}
                with connect(f'wss://127.0.0.1:{ws_port}', ssl=context,
                             proxy=None, open_timeout=2) as browser:
                    browser.send(json.dumps(packet))
                    deadline = time.monotonic() + 2
                    while vr.get_action()['xlevr.sample_sequence'] < 1 and time.monotonic() < deadline:
                        time.sleep(.01)
                    action = vr.get_action()
                    self.assertGreaterEqual(action['xlevr.sample_sequence'], 1)
                    self.assertTrue(action['xlevr.enabled'])
                    mapped = mapper(action)
                    self.assertTrue(all(mapped[key] == 0 for key in
                        ('ee.delta_x', 'ee.delta_y', 'ee.delta_z')))
                    stale = dict(action, **{'xlevr.sample_age_s': 1.0})
                    mapped = mapper(stale)
                    self.assertFalse(mapped['ee.enabled'])
            finally:
                vr.disconnect()
            self.assertFalse(vr._vr_thread.is_alive())
            self.assertFalse(monitor.servers_started)


if __name__ == '__main__':
    unittest.main()
