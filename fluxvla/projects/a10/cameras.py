"""Platform-owned OpenCV RGB capture shared by recording and inference."""

import threading

import cv2
import numpy as np

_USB_LOCK = threading.Lock()


class RGBCamera:
    def __init__(self, settings: dict):
        self.settings = settings
        self.capture = None

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
            capture.set(option, self.settings[key])
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return capture

    def _read(self, capture):
        ok, frame = capture.read()
        if not ok or frame is None:
            raise RuntimeError(f'RGB camera read failed: {self.settings["device"]}')
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        if self.settings['rotate_180']:
            rgb = cv2.rotate(rgb, cv2.ROTATE_180)
        return np.ascontiguousarray(rgb)

    def read_rgb(self):
        if self.settings.get('exclusive_per_read', False):
            with _USB_LOCK:
                capture = self._open()
                try:
                    capture.grab()
                    capture.grab()
                    return self._read(capture)
                finally:
                    capture.release()
        if self.capture is None:
            self.capture = self._open()
        return self._read(self.capture)

    def close(self):
        if self.capture is not None:
            self.capture.release()
            self.capture = None


def build_cameras(cfg):
    return {name: RGBCamera(settings) for name, settings in cfg['cameras'].items()}
