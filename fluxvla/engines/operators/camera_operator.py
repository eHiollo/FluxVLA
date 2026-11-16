"""Standalone USB RGB cameras for A10 recording and inference.

Capture is lazy: constructing an operator does not open a device. Frames are
HWC uint8 RGB; policy resizing belongs to the dataset transform, not capture.
"""

import threading
import time
import json
import logging
from pathlib import Path

import cv2
import numpy as np


# Some USB controllers cannot keep two video streams open simultaneously.
_USB_CAPTURE_LOCK = threading.Lock()
_REALSENSE_SETTINGS_LOCK = threading.Lock()


def _realsense_device(settings, preset):
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise RuntimeError('RealSense JSON requires pyrealsense2 in the capture environment') from exc
    serial = settings.get('identity', {}).get('serial')
    if not serial:
        raise ValueError('RealSense JSON requires identity.serial')
    context = rs.context()
    # USB descriptors used by by-id may expose ASIC serials, while recent SDKs
    # expose the device serial as serial_number. Both must select one device.
    matches = [d for d in context.query_devices() if any(
        d.supports(key) and d.get_info(key) == serial for key in
        (rs.camera_info.serial_number, rs.camera_info.asic_serial_number))]
    if len(matches) != 1:
        raise RuntimeError(f'RealSense serial {serial}: expected exactly one device')
    device = matches[0]
    expected = preset.get('device', {}).get('name', '')
    actual = device.get_info(rs.camera_info.name)
    if not expected or not actual.casefold().endswith(expected.casefold()):
        raise ValueError(f'RealSense preset model mismatch: {expected!r} vs {actual!r}')
    return rs, context, device


def _apply_realsense_options(rs, device, preset):
    """Apply only RGB exposure, gain and white balance; ignore advanced presets."""
    values = preset['parameters']
    color = any(k.startswith('controls-color-') for k in values)
    prefix = 'color' if color else 'depth'
    sensor = device.first_color_sensor() if color else device.first_depth_sensor()
    exposure_auto = 'controls-color-autoexposure-auto' if color else 'controls-autoexposure-auto'
    wb_auto = f'controls-{prefix}-white-balance-auto'
    automatic = str(values.get(exposure_auto, 'False')).casefold() == 'true'
    automatic_wb = str(values.get(wb_auto, 'False')).casefold() == 'true'
    controls = [(exposure_auto, 'enable_auto_exposure'), (wb_auto, 'enable_auto_white_balance')]
    if not automatic:
        controls += [('controls-color-autoexposure-manual' if color else 'controls-autoexposure-manual', 'exposure'),
                     (f'controls-{prefix}-gain', 'gain')]
    if not automatic_wb:
        controls.append((f'controls-{prefix}-white-balance-manual', 'white_balance'))
    report = {}
    for key, name in controls:
        if key not in values:
            continue
        option = getattr(rs.option, name)
        if not sensor.supports(option):
            raise RuntimeError(f'RealSense does not support preset control {key}')
        value = str(values[key])
        requested = float(value.casefold() == 'true') if value.casefold() in ('true', 'false') else float(value)
        sensor.set_option(option, requested)
        actual = sensor.get_option(option)
        if abs(actual-requested) > max(1e-5, sensor.get_option_range(option).step / 2):
            raise RuntimeError(f'RealSense {key}: requested {requested}, read back {actual}')
        report[key] = dict(requested=requested, actual=actual)
    return report


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
        path = self.settings.get('realsense_json')
        if path:
            # Reload on every open/reopen, not on every frame. Serial binding
            # prevents a D405 preset from being applied to the other camera.
            with _REALSENSE_SETTINGS_LOCK:
                preset = json.loads(Path(path).read_text(encoding='utf-8'))
                rs, context, device = _realsense_device(self.settings, preset)
                capture = self._open_capture()
                try:
                    self.last_settings_report = _apply_realsense_options(rs, device, preset)
                except Exception:
                    capture.release()
                    raise
                logging.getLogger(__name__).info('RealSense %s loaded %s; verified=%s',
                    self.settings['identity']['serial'], path, self.last_settings_report)
                return capture
        return self._open_capture()

    def _open_capture(self):
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


class BufferedCameraOperator:
    """Bounded RGB histories with HOST receive times (not exposure timestamps).

    Each worker owns one camera. Snapshot never waits for USB; ordinary synchronous
    CameraOperator callers are unchanged. Constructing this class opens nothing.
    """
    def __init__(self, cameras, history_size=4, camera_factory=RGBCamera):
        from collections import deque
        if not cameras or type(history_size) is not int or history_size < 2:
            raise ValueError('Configure cameras and history_size >= 2')
        if any(c.get('exclusive_per_read') for c in cameras.values()):
            raise ValueError('Recording requires persistent simultaneous camera streams')
        self.cameras = {n: camera_factory(c) for n, c in cameras.items()}
        self.histories = {n: deque(maxlen=history_size) for n in cameras}
        self.errors = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._threads = []

    def start(self):
        if self._threads or self._stop.is_set():
            raise RuntimeError('Camera workers cannot be restarted')
        for name, camera in self.cameras.items():
            thread = threading.Thread(target=self._capture, args=(name, camera),
                                      name=f'record-camera-{name}', daemon=True)
            self._threads.append(thread); thread.start()

    def _capture(self, name, camera):
        sequence = 0
        try:
            while not self._stop.is_set():
                rgb = camera.read_rgb()
                stamp = time.monotonic()
                rgb.setflags(write=False)
                with self._lock:
                    self.histories[name].append(dict(image=rgb, received_monotonic_s=stamp,
                                                     sequence=sequence))
                sequence += 1
        except Exception as exc:
            with self._lock: self.errors[name] = str(exc)
        finally:
            try: camera.close()
            except Exception as exc:
                with self._lock: self.errors[name] = str(exc)

    def snapshot(self, reference_time):
        with self._lock:
            if self.errors:
                raise RuntimeError(f'Camera errors: {self.errors}')
            if any(not frames for frames in self.histories.values()):
                raise RuntimeError('Waiting for all three camera streams')
            return {name: dict(min(frames, key=lambda f: abs(f['received_monotonic_s']-reference_time)))
                    for name, frames in self.histories.items()}

    def close(self):
        self._stop.set()
        for thread in self._threads: thread.join(1.)
        if any(thread.is_alive() for thread in self._threads):
            raise RuntimeError('Camera capture is blocked; worker not stopped (robot cleanup already requested)')
