"""Standalone USB RGB cameras for A10 recording and inference.

Capture is lazy: constructing an operator does not open a device. Frames are
HWC uint8 RGB; policy resizing belongs to the dataset transform, not capture.
"""

import threading
import time

import cv2
import numpy as np


# Some USB controllers cannot keep two video streams open simultaneously.
_USB_CAPTURE_LOCK = threading.Lock()


class RGBCamera:
    """One OpenCV camera, persistent or opened exclusively for each frame."""

    def __init__(self, settings):
        self.settings = dict(settings)
        if 'device' not in self.settings:
            raise ValueError('Camera device is required')
        for key in ('width', 'height', 'fps'):
            if key in self.settings and (type(self.settings[key]) not in (int, float)
                                         or self.settings[key] <= 0):
                raise ValueError(f'Camera {key} must be positive')
        self.capture = None
        self._lock = threading.RLock()

    def _open(self):
        source = self.settings['device']
        if isinstance(source, str) and source.isdigit():
            source = int(source)
        capture = cv2.VideoCapture(source)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f'Cannot open RGB camera: {source}')
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        for key, option in [('width', cv2.CAP_PROP_FRAME_WIDTH),
                            ('height', cv2.CAP_PROP_FRAME_HEIGHT),
                            ('fps', cv2.CAP_PROP_FPS)]:
            if key in self.settings:
                capture.set(option, self.settings[key])
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return capture

    def _read(self, capture, *, warmup=False):
        if warmup:
            capture.grab()
            capture.grab()
        ok, frame = capture.read()
        if (not ok or not isinstance(frame, np.ndarray) or frame.ndim != 3
                or frame.shape[2] != 3 or frame.dtype != np.uint8
                or not frame.shape[0] or not frame.shape[1]):
            raise RuntimeError(f'RGB camera read failed: {self.settings["device"]}')
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        if self.settings.get('rotate_180', False):
            rgb = cv2.rotate(rgb, cv2.ROTATE_180)
        return np.ascontiguousarray(rgb)

    def read_rgb(self):
        with self._lock:
            if self.settings.get('exclusive_per_read', False):
                with _USB_CAPTURE_LOCK:
                    capture = self._open()
                    try:
                        return self._read(capture, warmup=True)
                    finally:
                        capture.release()
            if self.capture is None or not self.capture.isOpened():
                self.close()
                self.capture = self._open()
            try:
                return self._read(self.capture)
            except RuntimeError:
                # Match A10_new's one-time reopen on a transient USB read error.
                self.close()
                self.capture = self._open()
                try:
                    return self._read(self.capture, warmup=True)
                except RuntimeError:
                    self.close()
                    raise

    @staticmethod
    def preprocess_to_policy_chw(frame_rgb, size=224):
        """A10_new bridge's aspect-preserving uint8 RGB policy input."""
        if type(size) is not int or size <= 0:
            raise ValueError('Policy image size must be a positive integer')
        frame = np.asarray(frame_rgb)
        if (frame.ndim != 3 or frame.shape[2] != 3 or frame.dtype != np.uint8
                or not frame.shape[0] or not frame.shape[1]):
            raise ValueError('Expected nonempty HWC uint8 RGB image')
        height, width = frame.shape[:2]
        scale = min(size / width, size / height)
        new_width = max(1, round(width * scale))
        new_height = max(1, round(height * scale))
        resized = cv2.resize(frame, (new_width, new_height),
                             interpolation=cv2.INTER_LINEAR)
        canvas = np.zeros((size, size, 3), dtype=np.uint8)
        y = (size - new_height) // 2
        x = (size - new_width) // 2
        canvas[y:y + new_height, x:x + new_width] = resized
        return np.ascontiguousarray(canvas.transpose(2, 0, 1))

    def close(self):
        with self._lock:
            if self.capture is not None:
                self.capture.release()
                self.capture = None


class CameraOperator:
    """Own named cameras and return a frame ready to merge with robot state.

    ``get_frame`` returns the configured RGB names plus a host monotonic
    timestamp. It does not imply hardware synchronization between cameras.
    """

    def __init__(self, cameras):
        if not isinstance(cameras, dict) or not cameras:
            raise ValueError('Configure at least one named camera')
        if 'camera_received_monotonic_s' in cameras:
            raise ValueError('Camera name conflicts with frame timestamp')
        self.cameras = {name: RGBCamera(settings) for name, settings in cameras.items()}

    def get_frame(self):
        frame = {name: camera.read_rgb() for name, camera in self.cameras.items()}
        frame['camera_received_monotonic_s'] = time.monotonic()
        return frame

    def close(self):
        errors = []
        for camera in self.cameras.values():
            try:
                camera.close()
            except Exception as exc:
                errors.append(exc)
        if errors:
            raise RuntimeError(f'Camera cleanup failed: {errors}') from errors[0]

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()
