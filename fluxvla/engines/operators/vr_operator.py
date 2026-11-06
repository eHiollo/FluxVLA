"""VR input and mapping interface; robot orchestration lives in vr.runtime."""

from abc import ABC, abstractmethod
from dataclasses import fields
from pathlib import Path
import math
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

    def __init__(self, *, network, mapping=None, mapping_by_hand=None, arm='right', hand_to_arm=None,
                 connection_timeout=10., enable_left_events=True,
                 thumbstick_event_threshold=.7):
        from .vr.input import VRInput
        from .vr.mapper import XLeVRDeltaEEMapper
        mapping = dict(mapping or {})
        allowed = {f.name for f in fields(XLeVRDeltaEEMapper) if f.init}
        if set(mapping) - allowed:
            raise ValueError(f'Unknown VR mapping settings: {set(mapping) - allowed}')
        if arm not in ('left', 'right', 'both'):
            raise ValueError('VR arm must be left, right or both')
        self.is_dual = arm == 'both'
        if self.is_dual:
            if (not isinstance(hand_to_arm, dict) or set(hand_to_arm) != {'left', 'right'}
                    or any(not isinstance(v, str) or not v for v in hand_to_arm.values())
                    or len(set(hand_to_arm.values())) != 2):
                raise ValueError('both hands require distinct explicit hand_to_arm bindings')
        elif hand_to_arm is not None:
            raise ValueError('hand_to_arm requires arm=both')
        self.hand_to_arm = dict(hand_to_arm or {})
        hands = ('left', 'right') if self.is_dual else (arm,)
        overrides = {} if mapping_by_hand is None else mapping_by_hand
        if not isinstance(overrides, dict) or set(overrides) - set(hands):
            raise ValueError('mapping_by_hand keys must name configured hands')
        if overrides and not self.is_dual:
            raise ValueError('mapping_by_hand requires arm=both; use mapping for single-hand input')
        self.mappers = {}
        for hand in hands:
            override = overrides.get(hand, {})
            if not isinstance(override, dict) or set(override) - allowed:
                raise ValueError(f'Unknown VR mapping settings for {hand}')
            options = dict(mapping, **override)
            if self.is_dual and (options.get('position_control_mode', 'safe_frame_delta') != 'safe_frame_delta'
                                or not options.get('require_squeeze_to_move', True)):
                raise ValueError('dual input requires safe_frame_delta and squeeze control')
            self.mappers[hand] = XLeVRDeltaEEMapper(**options)
        self.mapper = self.mappers[hands[0]]  # Preserve the single-input interface.
        settings = dict(network)
        directory = Path(settings.pop('tls_directory')).expanduser().resolve()
        settings.update(certfile=str(directory / 'cert.pem'),
                        keyfile=str(directory / 'key.pem'))
        self.input = VRInput(SimpleNamespace(
            arm=arm, connection_timeout=connection_timeout,
            enable_left_events=enable_left_events and not self.is_dual,
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
        for mapper in self.mappers.values():
            mapper.reset()

    def get_action(self):
        if self.is_dual:
            self.last_raw = self.input.get_hand_actions()
            self.last_action = {}
            for hand, raw in self.last_raw.items():
                # In dual mode only the side key engages an arm. The front
                # trigger changes the scale while squeezed, never takes over.
                squeezed = bool(raw.get('xlevr.grip_active') or
                                 raw.get('xlevr.buttons', {}).get('squeeze'))
                raw['xlevr.enabled'] = squeezed
                mapped = self.mappers[hand](raw)
                # Thumbsticks remain independent of the arm clutch. Each raw
                # sample keeps its own freshness; no arm-name-based USB routing.
                value = float(raw.get('xlevr.thumbstick', {}).get('x', 0.))
                if not math.isfinite(value) or abs(value) > 1:
                    raise ValueError('Dual VR thumbstick x must be finite within [-1, 1]')
                mapped['gripper.pos'] = value
                mapped['_xlevr.anchor_command']['gripper'] = value
                self.last_action[self.hand_to_arm[hand]] = mapped
            return self.last_action
        self.last_raw = self.input.get_action()
        self.last_action = self.mapper(self.last_raw)
        return self.last_action

    def get_events(self):
        return self.input.get_vr_events()

    def send_feedback(self, feedback):
        self.input.send_hud(feedback)

    def close(self):
        self.input.disconnect()
