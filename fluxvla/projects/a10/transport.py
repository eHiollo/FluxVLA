"""Small protocol adapters at the FluxVLA / A10 hardware boundary."""

from __future__ import annotations

import io
import json
import socket
import time

import numpy as np
import zmq

from fluxvla.engines.runners.serving.serializers import (MsgSerializer,
                                                        decode_predict_response,
                                                        encode_predict_request)


class PolicyClient:
    def __init__(self, host: str, port: int, timeout_s: float):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.LINGER, 0)
        timeout_ms = max(1, int(timeout_s * 1000))
        self.socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self.socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
        self.socket.connect(f'tcp://{host}:{port}')

    def ping(self) -> None:
        self.socket.send(MsgSerializer.to_bytes({'endpoint': 'ping'}))
        response = MsgSerializer.from_bytes(self.socket.recv())
        if response.get('status') != 'ok':
            raise RuntimeError(f'FluxVLA model service is not ready: {response}')

    def predict(self, observation: dict) -> np.ndarray:
        self.socket.send(encode_predict_request(observation, unnorm_key='a10'))
        response = decode_predict_response(self.socket.recv())
        if response.get('error'):
            raise RuntimeError(f'FluxVLA inference failed: {response["error"]}')
        raw = response.get('action_data')
        if not isinstance(raw, bytes):
            raise RuntimeError('FluxVLA response has no action_data')
        actions = np.load(io.BytesIO(raw), allow_pickle=False)
        return np.asarray(actions, dtype=np.float32)

    def close(self) -> None:
        self.socket.close(linger=0)
        self.context.term()


class A10TcpClient:
    """Bounded newline JSON client for the existing A10 controller protocol."""

    def __init__(self, host: str, port: int, timeout_s: float = 2.0):
        self.socket = socket.create_connection((host, int(port)), timeout_s)
        self.socket.settimeout(timeout_s)
        self.buffer = bytearray()

    def _receive_line(self) -> dict:
        while b'\n' not in self.buffer:
            chunk = self.socket.recv(4096)
            if not chunk:
                raise ConnectionError('A10 controller closed the connection')
            self.buffer.extend(chunk)
            if len(self.buffer) > 1024 * 1024:
                raise ValueError('A10 controller reply is too large')
        line, _, remaining = self.buffer.partition(b'\n')
        self.buffer = bytearray(remaining)
        return json.loads(line)

    def _request(self, line: str) -> dict:
        self.socket.sendall(line.encode('utf-8') + b'\n')
        response = self._receive_line()
        if not isinstance(response, dict):
            raise ValueError('A10 controller reply must be an object')
        return response

    def state(self) -> np.ndarray:
        value = np.asarray(self._request('GET_FOLLOWER_STATE').get('q'),
                           dtype=np.float32)
        if value.shape != (7,) or not np.isfinite(value).all():
            raise ValueError(f'A10 controller returned invalid state {value}')
        return value

    def status(self) -> dict:
        result = self._request('GET_POLICY_STATUS')
        remaining = result.get('remaining')
        if isinstance(remaining, bool) or not isinstance(remaining, int):
            raise ValueError(f'A10 controller returned invalid status {result}')
        return result

    def send_batch(self, actions: np.ndarray) -> None:
        values = np.asarray(actions, dtype=np.float32)
        if values.ndim != 2 or values.shape[1] != 7 or not np.isfinite(values).all():
            raise ValueError(f'Invalid A10 batch shape/value: {values.shape}')
        payload = json.dumps({'actions': values.tolist()}, separators=(',', ':'))
        ack = self._request('SET_JOINTS_BATCH ' + payload)
        if ack.get('accepted') is not True:
            raise RuntimeError(f'A10 controller rejected batch: {ack}')

    def wait_idle(self, poll_s: float, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.status()['remaining'] == 0:
                return
            time.sleep(poll_s)
        raise TimeoutError('A10 batch did not finish before idle timeout')

    def close(self) -> None:
        self.socket.close()
