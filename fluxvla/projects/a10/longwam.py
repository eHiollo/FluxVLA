"""LongWAM JSON transport. Images are already oriented RGB from capture."""

import base64
import json
import math
import uuid
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener

import cv2
import numpy as np


class LongWamPolicy:
    def __init__(self, host, port, timeout_s=30., jpeg_quality=90):
        if type(jpeg_quality) is not int or not 1 <= jpeg_quality <= 100:
            raise ValueError('jpeg_quality must be 1..100')
        self.url = f'http://{host}:{port}'
        self.timeout = timeout_s
        self.jpeg_quality = jpeg_quality
        self.opener = build_opener(ProxyHandler({}))
        health = self.request('/health')
        if health.get('ready') is not True or not health.get('checkpoint'):
            raise RuntimeError(f'LongWAM is not ready: {health}')
        self.metadata = dict(protocol='longwam_http', model=health)

    def request(self, path, body=None):
        request = Request(self.url + path, data=body,
                          headers={'Content-Type': 'application/json'})
        try:
            with self.opener.open(request, timeout=self.timeout) as response:
                payload = response.read(16 * 1024 * 1024 + 1)
        except HTTPError as exc:
            detail = exc.read(2048).decode('utf-8', errors='replace')
            raise RuntimeError(f'LongWAM HTTP {exc.code}: {detail}') from exc
        if len(payload) > 16 * 1024 * 1024:
            raise ValueError('LongWAM response exceeds 16 MiB')
        result = json.loads(payload)
        if not isinstance(result, dict):
            raise ValueError('Expected LongWAM JSON object')
        return result

    def encode_image(self, image):
        image = np.asarray(image)
        if image.dtype != np.uint8 or image.shape != (480, 640, 3):
            raise ValueError('LongWAM requires 640x480 HWC uint8 RGB')
        success, jpeg = cv2.imencode('.jpg', cv2.cvtColor(image, cv2.COLOR_RGB2BGR),
                                    [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not success:
            raise RuntimeError('JPEG encoding failed')
        return base64.b64encode(jpeg).decode('ascii')

    def infer(self, observation):
        state = np.asarray(observation['observation/state'])
        if state.shape != (14,) or state.dtype.kind not in 'fiu' or not np.isfinite(state).all():
            raise ValueError('LongWAM state must be finite numeric [14]')
        if np.any((state[[6, 13]] < 0) | (state[[6, 13]] > 1)):
            raise ValueError('LongWAM grippers must be within 0..1')
        histories = observation['image_history']
        if set(histories) != {'top', 'left', 'right'} or any(len(frames) != 13 for frames in histories.values()):
            raise ValueError('LongWAM requires three 13-frame histories')
        obs_id = uuid.uuid4().hex
        body = json.dumps(dict(obs_id=obs_id, state=state.tolist(), frames={
            name: [self.encode_image(image) for image in frames]
            for name, frames in histories.items()}), allow_nan=False,
            separators=(',', ':')).encode('utf-8')
        if len(body) > 16 * 1024 * 1024:
            raise ValueError('LongWAM request exceeds 16 MiB; lower remote_inference.jpeg_quality')
        response = self.request('/predict', body)
        if response.get('obs_id') != obs_id:
            raise ValueError('LongWAM obs_id mismatch')
        if response.get('checkpoint') != self.metadata['model']['checkpoint']:
            raise ValueError('LongWAM checkpoint changed since health check')
        if type(response.get('step_hz')) not in (int, float) or response['step_hz'] != 30:
            raise ValueError('LongWAM step_hz must be 30')
        if type(response.get('first_step_offset')) is not int or response['first_step_offset'] != 0:
            raise ValueError('LongWAM first_step_offset must be 0')
        for key in ('server_ms', 'inference_ms'):
            value = response.get(key)
            if type(value) not in (float, int) or not math.isfinite(value) or value < 0:
                raise ValueError(f'Invalid LongWAM {key}')
        actions = np.asarray(response.get('actions'))
        if actions.shape != (32, 14) or actions.dtype.kind not in 'fiu' or not np.isfinite(actions).all():
            raise ValueError('LongWAM requires finite numeric actions [32,14]')
        return response

    def close(self):
        self.opener.close()
