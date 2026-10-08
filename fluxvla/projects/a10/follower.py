"""Robot backend boundary. A future follower replaces this registered operator."""

from typing import Protocol

import numpy as np

from fluxvla.engines import OPERATORS, build_operator_from_cfg
from .transport import A10TcpClient


class Follower(Protocol):
    capabilities: frozenset[str]

    def state(self) -> np.ndarray: ...
    def status(self) -> dict: ...
    def send_batch(self, actions: np.ndarray) -> None: ...
    def send_ee_delta(self, actions: np.ndarray) -> None: ...
    def reset(self) -> None: ...
    def send_anchor_diagnostics(self, command: dict) -> None: ...
    def wait_idle(self, poll_s: float, timeout_s: float) -> None: ...
    def close(self) -> None: ...


@OPERATORS.register_module()
class A10LegacyFollower(A10TcpClient):
    capabilities = frozenset({'joint_batch', 'ee_delta', 'reset', 'anchor_diagnostics'})

    def send_ee_delta(self, actions):
        values = np.asarray(actions, dtype=np.float64)
        if values.shape != (7,) or not np.isfinite(values).all():
            raise ValueError('A10 EE delta requires seven finite values')
        if abs(values[6]) > 1:
            raise ValueError('A10 VR gripper command must be within [-1,1]')
        self._send_command('SET_EE_DELTA', {'actions': values.tolist()})

    def _send_command(self, command, payload=None):
        import json
        line = command if payload is None else command + ' ' + json.dumps(
            payload, separators=(',', ':'), allow_nan=False)
        self.socket.sendall(line.encode('utf-8') + b'\n')

    def send_anchor_diagnostics(self, command):
        self._send_command('SET_EE_ANCHOR', command)

    def reset(self):
        self._send_command('reset')


def build_follower(cfg) -> Follower:
    settings = cfg['robot']
    return build_operator_from_cfg(dict(
        type=settings['backend'], host=settings['host'], port=settings['port'],
        timeout_s=min(float(settings['timeout_ms']) / 1000, 5.0)))
