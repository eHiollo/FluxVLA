"""Onsite probe tests using local fake devices, without physical robot motion."""

import json
import socketserver
import subprocess
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import cv2
import numpy as np

from fluxvla.projects.a10 import hardware_check as check
from fluxvla.projects.a10.config import load_project


class ReadOnlyHandler(socketserver.StreamRequestHandler):
    def handle(self):
        for line in self.rfile:
            command = line.decode().strip()
            self.server.commands.append(command)
            if command == 'GET_POLICY_STATUS':
                reply = {'remaining': self.server.remaining,
                         'idle': self.server.remaining == 0}
            elif command == 'GET_FOLLOWER_STATE':
                reply = {'q': [0.] * 6 + [50.]}
            else:
                reply = {'error': 'unexpected command'}
            self.wfile.write(json.dumps(reply).encode() + b'\n')


class HardwareCheckTest(unittest.TestCase):
    def test_robot_probe_sends_only_queries_and_stops_when_busy(self):
        with socketserver.TCPServer(('127.0.0.1', 0), ReadOnlyHandler) as server:
            server.commands, server.remaining = [], 0
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            cfg = load_project()
            cfg['robot'].update(host='127.0.0.1', port=server.server_address[1])
            try:
                result = check.check_robot(cfg)
                self.assertEqual(result['status'], 'PASS')
                self.assertEqual(result['state'][-1], 50.)
                self.assertEqual(server.commands, ['GET_POLICY_STATUS',
                                                   'GET_FOLLOWER_STATE',
                                                   'GET_POLICY_STATUS'])
                server.commands, server.remaining = [], 10
                self.assertEqual(check.check_robot(cfg)['status'], 'FAIL')
                self.assertEqual(server.commands, ['GET_POLICY_STATUS'])
            finally:
                server.shutdown()
                thread.join(timeout=2)

    def _camera_config(self, output):
        cfg = load_project()
        for name, settings in cfg['cameras'].items():
            path = output / name
            path.touch()
            settings.update(device=str(path), width=8, height=6)
        return cfg

    def test_paired_capture_and_saved_rgb_snapshot(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            cfg = self._camera_config(output)
            rgb = np.full((6, 8, 3), [10, 30, 90], dtype=np.uint8)
            cameras = {name: Mock() for name in cfg['cameras']}
            for camera in cameras.values():
                camera.read_rgb.return_value = rgb
            with patch.object(check, 'build_cameras', return_value=cameras), \
                    patch.object(check.time, 'monotonic', side_effect=[1., 1.04, 1.08]):
                result = check.check_cameras(cfg, output, 3)
            self.assertEqual(result['status'], 'PASS')
            self.assertAlmostEqual(result['paired_capture_fps'], 25.)
            self.assertFalse(result['meets_90_percent_target'])
            for name, camera in cameras.items():
                self.assertEqual(camera.read_rgb.call_count, 5)
                camera.close.assert_called_once()
                image = cv2.imread(str(output / f'{name}.png'))
                np.testing.assert_array_equal(image[..., ::-1], rgb)

    def test_dimension_mismatch_releases_both_cameras(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            cfg = self._camera_config(output)
            cameras = {name: Mock() for name in cfg['cameras']}
            for camera in cameras.values():
                camera.read_rgb.return_value = np.zeros((2, 2, 3), dtype=np.uint8)
            with patch.object(check, 'build_cameras', return_value=cameras):
                with self.assertRaisesRegex(ValueError, 'expected uint8'):
                    check.check_cameras(cfg, output, 3)
            for camera in cameras.values():
                camera.close.assert_called_once()

    def test_missing_and_duplicate_devices_do_not_open_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            cfg = self._camera_config(output)
            paths = [Path(value['device']) for value in cfg['cameras'].values()]
            paths[1].unlink()
            paths[1].symlink_to(paths[0])
            with patch.object(check, 'build_cameras') as build:
                result = check.check_cameras(cfg, output, 3)
                self.assertIn('same device', result['error'])
                paths[0].unlink()
                result = check.check_cameras(cfg, output, 3)
                self.assertIn('missing', result['error'])
                build.assert_not_called()

    def test_timed_out_worker_does_not_report_success(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            # A stale result must never override a timed-out process.
            (output / 'cameras.json').write_text('{"status": "PASS"}')
            with patch.object(check.subprocess, 'run',
                              side_effect=subprocess.TimeoutExpired('worker', 1)):
                result = check.run_worker(load_project(), 'cameras', output, 3, 1)
            self.assertEqual(result['status'], 'FAIL')
            self.assertIn('timed out', result['error'])


if __name__ == '__main__':
    unittest.main()
