"""VR input and mapping interface; robot orchestration lives in vr.runtime."""

from abc import ABC, abstractmethod
from dataclasses import fields
from pathlib import Path
from types import SimpleNamespace

from ..utils.root import OPERATORS


class BaseTeleoperator(ABC):
    """Small input-device contract, independent of any robot transport."""

    @abstractmethod
    def connect(self):
        """Open the input service."""

    @abstractmethod
    def get_action(self):
        """Read and map the latest input sample."""

    @abstractmethod
    def get_events(self):
        """Read pending operator events."""

    @abstractmethod
    def send_feedback(self, feedback):
        """Publish input-device feedback."""

    @abstractmethod
    def close(self):
        """Release input resources."""


@OPERATORS.register_module()
class VROperator(BaseTeleoperator):
    """Own WebXR input + mapper, never connect to or command a robot."""

    def __init__(self, *, network, mapping=None, arm='right',
                 connection_timeout=10., enable_left_events=True,
                 thumbstick_event_threshold=.7):
        from .vr.input import VRInput
        from .vr.mapper import XLeVRDeltaEEMapper
        mapping = dict(mapping or {})
        allowed = {f.name for f in fields(XLeVRDeltaEEMapper) if f.init}
        if set(mapping) - allowed:
            raise ValueError(f'Unknown VR mapping settings: {set(mapping) - allowed}')
        if arm not in ('left', 'right'):
            raise ValueError('VR arm must be left or right')
        self.mapper = XLeVRDeltaEEMapper(**mapping)
        settings = dict(network)
        directory = Path(settings.pop('tls_directory')).expanduser().resolve()
        settings.update(certfile=str(directory / 'cert.pem'),
                        keyfile=str(directory / 'key.pem'))
        self.input = VRInput(SimpleNamespace(
            arm=arm, connection_timeout=connection_timeout,
            enable_left_events=enable_left_events,
            thumbstick_event_threshold=thumbstick_event_threshold,
            fine_trigger_threshold=self.mapper.fine_trigger_threshold,
            require_squeeze_to_move=self.mapper.require_squeeze_to_move,
            xlevr_path=str(Path(__file__).with_name('vr')),
            server_settings=settings))
        self.last_raw = self.last_action = None

    @property
    def is_connected(self):
        return self.input.is_connected

    def connect(self):
        self.reset_reference()
        try:
            self.input.connect()
        except BaseException:
            self.close()
            raise
        return self

    def reset_reference(self):
        self.mapper.reset()

    def get_action(self):
        self.last_raw = self.input.get_action()
        self.last_action = self.mapper(self.last_raw)
        return self.last_action

    def get_events(self):
        return self.input.get_vr_events()

    def send_feedback(self, feedback):
        self.input.send_hud(feedback)

    def close(self):
        self.input.disconnect()
