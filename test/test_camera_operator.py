"""USB camera contract tests with fake OpenCV devices; no hardware access."""

import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np


_spec = importlib.util.spec_from_file_location(
    '_standalone_camera', Path(__file__).resolve().parents[1]
    / 'fluxvla/engines/operators/camera_operator.py')
module = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = module
_spec.loader.exec_module(module)
CameraOperator = module.CameraOperator
RGBCamera = module.RGBCamera


class FakeCapture:
    def __init__(self, frame=None, *, opened=True, read_ok=True):
        self.frame = frame
        self.opened = opened
        self.read_ok = read_ok
        self.released = False
        self.grabs = 0
        self.settings = []

    def isOpened(self):
        return self.opened and not self.released

    def set(self, *args):
        self.settings.append(args)
        return True

    def grab(self):
        self.grabs += 1
        return True

    def read(self):
        return self.read_ok, self.frame

    def release(self):
        self.released = True


class CameraOperatorTest(unittest.TestCase):
    def setUp(self):
        self.bgr = np.array([[[1, 2, 3], [4, 5, 6]],
                             [[7, 8, 9], [10, 11, 12]]], dtype=np.uint8)

    def test_lazy_persistent_rgb_rotation_and_close(self):
        capture = FakeCapture(self.bgr)
        with patch.object(module.cv2, 'VideoCapture', return_value=capture) as open_camera:
            camera = RGBCamera(dict(device='2', width=640, height=480,
                                    fps=30, rotate_180=True))
            open_camera.assert_not_called()
            np.testing.assert_array_equal(camera.read_rgb(), self.bgr[::-1, ::-1, ::-1])
            camera.read_rgb()
            open_camera.assert_called_once_with(2)
            camera.close()
            self.assertTrue(capture.released)

    def test_exclusive_read_warms_up_and_releases_on_failure(self):
        good = FakeCapture(self.bgr)
        bad = FakeCapture(self.bgr, read_ok=False)
        with patch.object(module.cv2, 'VideoCapture', side_effect=[good, bad]):
            camera = RGBCamera(dict(device='/dev/video0', exclusive_per_read=True))
            np.testing.assert_array_equal(camera.read_rgb(), self.bgr[..., ::-1])
            self.assertEqual(good.grabs, 2)
            self.assertTrue(good.released)
            with self.assertRaisesRegex(RuntimeError, 'read failed'):
                camera.read_rgb()
            self.assertTrue(bad.released)

    def test_persistent_read_reopens_once(self):
        bad = FakeCapture(self.bgr, read_ok=False)
        good = FakeCapture(self.bgr)
        with patch.object(module.cv2, 'VideoCapture', side_effect=[bad, good]) as open_camera:
            camera = RGBCamera(dict(device=1))
            np.testing.assert_array_equal(camera.read_rgb(), self.bgr[..., ::-1])
            self.assertEqual(open_camera.call_count, 2)
            self.assertTrue(bad.released)
            self.assertEqual(good.grabs, 2)
            camera.close()

    def test_operator_paired_frame_and_cleanup(self):
        captures = [FakeCapture(self.bgr), FakeCapture(self.bgr)]
        settings = {'base_0_rgb': {'device': 0},
                    'right_wrist_0_rgb': {'device': 2, 'rotate_180': True}}
        with patch.object(module.cv2, 'VideoCapture', side_effect=captures):
            with CameraOperator(settings) as cameras:
                frame = cameras.get_frame()
            self.assertGreater(frame['camera_received_monotonic_s'], 0)
            np.testing.assert_array_equal(frame['base_0_rgb'], self.bgr[..., ::-1])
            np.testing.assert_array_equal(frame['right_wrist_0_rgb'],
                                          self.bgr[::-1, ::-1, ::-1])
            self.assertTrue(all(capture.released for capture in captures))

    def test_cleanup_continues_after_one_camera_failure(self):
        cameras = CameraOperator({'base': dict(device=0), 'wrist': dict(device=2)})
        with patch.object(cameras.cameras['base'], 'close', side_effect=OSError('release failed')), \
                patch.object(cameras.cameras['wrist'], 'close') as close_wrist:
            with self.assertRaisesRegex(RuntimeError, 'cleanup failed'):
                cameras.close()
            close_wrist.assert_called_once()

    def test_policy_resize_preserves_aspect_and_rejects_bad_frame(self):
        rgb = np.full((2, 4, 3), [11, 22, 33], dtype=np.uint8)
        result = RGBCamera.preprocess_to_policy_chw(rgb, size=4)
        self.assertEqual(result.shape, (3, 4, 4))
        self.assertTrue(np.all(result[:, 0] == 0))
        np.testing.assert_array_equal(result[:, 1, 0], [11, 22, 33])
        with self.assertRaises(ValueError):
            RGBCamera.preprocess_to_policy_chw(np.zeros((2, 4, 3), dtype=np.float32))


if __name__ == '__main__':
    unittest.main()
