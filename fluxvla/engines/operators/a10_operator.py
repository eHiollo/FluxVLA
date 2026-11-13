"""A10 single-controller operator for KAANH joint and Cartesian followers.

Wire layout follows the user-supplied robot.7z reference. Local initialization
repeats configured joint targets; motion planning stays on the controller.
This module does not open connections at
import/construction, automatically enable motors, interpret VR increments, or invent a
seventh joint. Legacy Cartesian packets retain their original encoding. Native
Tool follower integrates cumulative component differences in current tool axes;
use the opt-in SDK tool adapter for base-frame planned pose trajectories.
"""

from __future__ import annotations

import json
import logging
import math
import select
import socket
import struct
import threading
import time
import warnings
from collections import deque
from dataclasses import dataclass, field, replace
from types import MappingProxyType
from typing import Mapping

import numpy as np
from scipy.spatial.transform import Rotation

from ..utils.root import OPERATORS
from .base_operator import BaseOperator
from .camera_operator import CameraOperator
from .gripper_operator import GripperOperator, GripperError, GripperTimeoutError

__all__ = ['A10Operator', 'A10State', 'A10ArmState', 'A10ProtocolError', 'A10ControllerError']

_HEADER = struct.Struct('<IIQqqq')
_MAX_PAYLOAD = 1024 * 1024
_FOLLOWER_MODES = {'cartesian': ('follower_cart', 'Cart'),
                   'joint': ('follower_aj', 'Axis')}


class SDKToolCumulativeAdapter:
    """Geometric feedforward only; does not estimate SDK execution progress.

    Candidate generation is transactional: commit only after local UDP success.
    Euler increments approximate the SDK's finer-grained component integration.
    """

    def __init__(self, startup_pose):
        self.previous_pose = _pose(startup_pose).copy()
        self.cumulative_pe = np.zeros(6)

    def propose(self, target, *, max_position_step_m, max_rotation_step_rad):
        target = _pose(target)
        for limit in (max_position_step_m, max_rotation_step_rad):
            if limit is not None and (not math.isfinite(limit) or limit <= 0):
                raise ValueError('SDK adapter step limits must be None or positive and finite')
        previous_rotation = Rotation.from_quat(self.previous_pose[3:])
        translation = previous_rotation.inv().apply(target[:3] - self.previous_pose[:3])
        rotation = previous_rotation.inv() * Rotation.from_quat(target[3:])
        # Reference teleoperation limits speed/acceleration before this codec.
        # Explicit console callers can still request independent step bounds.
        if ((max_position_step_m is not None and np.linalg.norm(translation) > max_position_step_m)
                or (max_rotation_step_rad is not None and rotation.magnitude() > max_rotation_step_rad)):
            raise ValueError('SDK adapter target exceeds step limit')
        # This interface is for small continuous steps, never large Euler jumps.
        if rotation.magnitude() >= np.pi / 2:
            raise ValueError('SDK adapter rotation must be smaller than pi/2')
        incremental = np.r_[translation, rotation.as_euler('ZYX')]
        candidate = self.cumulative_pe + incremental
        if not np.isfinite(candidate).all():
            raise ValueError('SDK adapter cumulative target is not finite')
        return target.copy(), candidate

    def commit(self, target, cumulative_pe):
        self.previous_pose = target.copy()
        self.cumulative_pe = cumulative_pe.copy()


class A10ProtocolError(ValueError):
    """A malformed or incompatible controller message."""


class A10ControllerError(RuntimeError):
    """The controller rejected a command or reported a fault."""


def _pack(payload: bytes, *, command: bool) -> bytes:
    if len(payload) > _MAX_PAYLOAD:
        raise A10ProtocolError('Payload exceeds the protocol size limit')
    fields = (1, 0x1000, 0xA1B2C3D4, 0, 0) if command else (0, 0, 0, 0, 0)
    return _HEADER.pack(len(payload), *fields) + payload


def _unpack(packet: bytes) -> bytes:
    if not isinstance(packet, bytes) or len(packet) < _HEADER.size:
        raise A10ProtocolError('Expected a binary frame with a 40-byte header')
    size, *_ = _HEADER.unpack_from(packet)
    if size > _MAX_PAYLOAD or size != len(packet) - _HEADER.size:
        raise A10ProtocolError('Header payload length does not match frame')
    # Response IDs/reserved fields have no documented echo contract.
    return packet[_HEADER.size:]


def _reject_constant(value):
    raise A10ProtocolError(f'Non-finite JSON number: {value}')


def _json_object(payload):
    try:
        value = json.loads(payload, parse_constant=_reject_constant)
    except (ValueError, UnicodeDecodeError) as exc:
        raise A10ProtocolError('Invalid controller JSON') from exc
    if not isinstance(value, dict):
        raise A10ProtocolError('Controller reply must be a JSON object')
    return value


def _decode_reply(packet, *, raw=False):
    value = _json_object(_unpack(packet))
    code = value.get('ret_code')
    if type(code) is not int:
        raise A10ProtocolError('Reply must contain integer ret_code')
    if raw:
        return value
    if code != 0:
        raise A10ControllerError(f'Controller ret_code={code}: {value.get("ret_msg", "")}')
    context = value.get('ret_context')
    if isinstance(context, str) and context.strip():
        value['ret_context'] = _json_object(context)
    return value


def _vector(value, size, field):
    if (not isinstance(value, (list, tuple, np.ndarray))
            or np.asarray(value).shape != (size,)):
        raise A10ProtocolError(f'{field} must have exactly {size} values')
    if any(isinstance(x, (bool, np.bool_)) or not isinstance(
            x, (int, float, np.integer, np.floating)) for x in value):
        raise A10ProtocolError(f'{field} must contain numbers, not strings/bools')
    result = np.asarray(value, dtype=np.float64)
    if not np.isfinite(result).all():
        raise A10ProtocolError(f'{field} must contain finite values')
    return result.copy()


def _single_group(value, size, field):
    """Accept flat or one nested group; never silently truncate another robot."""
    if value is None:
        return None
    if isinstance(value, list) and len(value) == 1 and isinstance(value[0], list):
        value = value[0]
    return _vector(value, size, field)


def _pose(value):
    result = _vector(value, 7, 'relative_pose_m_xyzw')
    if not math.isclose(float(np.linalg.norm(result[3:])), 1.0, abs_tol=1e-3):
        raise A10ProtocolError('Quaternion xyzw must have unit length')
    return result


def _optional_bool(value, field):
    if value is not None and type(value) is not bool:
        raise A10ProtocolError(f'{field} must be a boolean')
    return value


@dataclass(frozen=True)
class A10ArmState:
    """One explicitly mapped six-axis model; global faults stay on A10State."""

    ee_id: int
    actual_joints_rad: np.ndarray | None
    target_joints_rad: np.ndarray | None
    tcp_pq_raw: np.ndarray | None
    tcp_pq_m: np.ndarray | None
    received_monotonic_s: float


@dataclass(frozen=True)
class A10State:
    """Parsed feedback. None means unavailable, never a synthetic zero.

    received_monotonic_s is client receive time, not controller sample time.
    tcp_pq_raw retains the controller's unconfirmed pose semantics/units.
    """

    actual_joints_rad: np.ndarray | None
    target_joints_rad: np.ndarray | None
    tcp_pq_raw: np.ndarray | None
    tcp_pq_m: np.ndarray | None
    gripper_mm: float | None
    follower_active: bool | None
    follower_mode: str | None
    has_error: bool
    received_monotonic_s: float
    raw: dict
    arms: Mapping[str, A10ArmState] = field(default_factory=dict)

    @property
    def qpos(self):
        if self.actual_joints_rad is None or self.gripper_mm is None:
            return None
        return np.concatenate((self.actual_joints_rad, [self.gripper_mm]))


def _normalize_arms(arms, arm_name, order):
    if arms is None:
        if order is not None:
            raise ValueError('cartesian_target_order requires explicit arms')
        return None, (arm_name,)
    if not isinstance(arms, Mapping) or len(arms) not in (1, 2):
        raise ValueError('arms must configure one or two A10 arms')
    normalized, ee_ids, tcp_indices, joints = {}, set(), set(), set()
    for name, spec in arms.items():
        if not isinstance(name, str) or not name or not isinstance(spec, Mapping):
            raise ValueError('Arm names must be nonempty and arm settings must be mappings')
        required = {'ee_id', 'joint_group', 'tcp_index'}
        if not required <= set(spec) or set(spec) - required - {'joint_indices'}:
            raise ValueError('Each arm requires ee_id, joint_group, tcp_index; optional joint_indices')
        values = {key: spec[key] for key in required}
        if any(type(v) is not int or v < 0 for v in values.values()):
            raise ValueError('Arm IDs and feedback indices must be nonnegative integers')
        indices = spec.get('joint_indices', range(6))
        if (not isinstance(indices, (list, tuple, range)) or len(indices) != 6
                or any(type(v) is not int or v < 0 for v in indices) or len(set(indices)) != 6):
            raise ValueError('joint_indices must contain six distinct nonnegative integers')
        joint_keys = {(values['joint_group'], index) for index in indices}
        if values['ee_id'] in ee_ids or values['tcp_index'] in tcp_indices or joints & joint_keys:
            raise ValueError('Duplicate ee_id, TCP mapping or overlapping joint feedback')
        ee_ids.add(values['ee_id'])
        tcp_indices.add(values['tcp_index'])
        joints.update(joint_keys)
        normalized[name] = MappingProxyType(dict(values, joint_indices=tuple(indices)))
    if order is None:
        if len(arms) != 1:
            raise ValueError('Dual arms require explicit cartesian_target_order matching controller XML')
        order = tuple(arms)
    if (not isinstance(order, (list, tuple)) or any(not isinstance(x, str) for x in order)
            or len(order) != len(arms) or set(order) != set(arms)):
        raise ValueError('cartesian_target_order must list every configured arm exactly once')
    return MappingProxyType(normalized), tuple(order)


def _feedback_group(value, index, field_name):
    if value is None:
        return None
    if not isinstance(value, list) or not value:
        raise A10ProtocolError(f'{field_name} must contain feedback groups')
    groups = value if isinstance(value[0], list) else [value]
    if index >= len(groups):
        raise A10ProtocolError(f'{field_name} has no configured feedback group {index}')
    group = groups[index]
    if not isinstance(group, list) or not group:
        raise A10ProtocolError(f'{field_name}[{index}] is not a numeric group')
    return _vector(group, len(group), f'{field_name}[{index}]')


def _parse_state(reply, *, joint_unit, position_unit, gripper_feedback_path,
                 arms=None, arm_name='right'):
    context = reply.get('ret_context')
    if not isinstance(context, dict) or not isinstance(context.get('motion_msg'), dict):
        raise A10ProtocolError('Status reply is missing motion_msg')
    if arms is not None:
        motion, parsed, stamp = context['motion_msg'], {}, time.monotonic()
        first = None
        for name, spec in arms.items():
            selected = dict(motion)
            indices = spec.get('joint_indices', range(6))
            for key in ('actual_pos', 'motor_pos'):
                values = _feedback_group(motion.get(key), spec['joint_group'], key)
                if values is not None and max(indices) >= len(values):
                    raise A10ProtocolError(f'{key} cannot supply six configured joints for {name}')
                selected[key] = None if values is None else values[list(indices)].tolist()
            values = _feedback_group(motion.get('pq'), spec['tcp_index'], 'pq')
            selected['pq'] = None if values is None else values.tolist()
            nested = dict(reply, ret_context=dict(context, motion_msg=selected))
            state = _parse_state(nested, joint_unit=joint_unit, position_unit=position_unit,
                                 gripper_feedback_path=gripper_feedback_path, arm_name=name)
            if first is None:
                first = state
            parsed[name] = A10ArmState(spec['ee_id'], state.actual_joints_rad, state.target_joints_rad,
                                       state.tcp_pq_raw, state.tcp_pq_m, stamp)
        single = first if len(parsed) == 1 else None
        return A10State(single.actual_joints_rad if single else None,
                        single.target_joints_rad if single else None,
                        single.tcp_pq_raw if single else None, single.tcp_pq_m if single else None,
                        single.gripper_mm if single else None, first.follower_active,
                        first.follower_mode, first.has_error, stamp, reply, MappingProxyType(parsed))
    motion = context['motion_msg']
    groups = {}
    for key in ('follower_msg', 'robot_msg', 'driver_msg', 'log_msg'):
        groups[key] = context.get(key, {})
        if not isinstance(groups[key], dict):
            raise A10ProtocolError(f'{key} must be an object')
    follower, robot, driver, log = (groups[k] for k in groups)
    actual = _single_group(motion.get('actual_pos'), 6, 'actual_pos')
    target = _single_group(motion.get('motor_pos'), 6, 'motor_pos')
    if joint_unit == 'deg':
        actual = None if actual is None else np.deg2rad(actual)
        target = None if target is None else np.deg2rad(target)
    pq = motion.get('pq')
    # The template allows [model TCP, model axis pose] even for a single arm.
    # Joint groups above still have to describe exactly one six-axis arm.
    if isinstance(pq, list) and len(pq) == 2 and all(isinstance(x, list) for x in pq):
        _vector(pq[1], 7, 'auxiliary axis pq')
        pq = pq[0]
    pose = _single_group(pq, 7, 'pq')
    converted = None
    if pose is not None:
        _pose(pose)
        if position_unit is not None:
            converted = pose.copy()
            if position_unit == 'mm':
                converted[:3] /= 1000.0
    gripper = None
    if gripper_feedback_path is not None:
        value = context
        for key in gripper_feedback_path:
            value = value.get(key) if isinstance(value, dict) else None
        if value is not None:
            gripper = float(_vector([value], 1, 'gripper feedback in mm')[0])
            if not 0 <= gripper <= 100:
                raise A10ProtocolError('A10 gripper feedback must be within 0..100 mm')
    codes = driver.get('driver_err_code', [])
    if not isinstance(codes, list) or any(type(x) is not int for x in codes):
        raise A10ProtocolError('driver_err_code must be an integer list')
    has_error = bool(_optional_bool(log.get('has_error'), 'has_error')) or any(codes)
    for group in (robot, log):
        for key in ('cs_error_code', 'error_code', 'err_code'):
            if key in group:
                if type(group[key]) is not int:
                    raise A10ProtocolError(f'{key} must be an integer')
                has_error |= group[key] != 0
    mode = follower.get('follower_mode')
    if mode is not None and not isinstance(mode, str):
        raise A10ProtocolError('follower_mode must be a string')
    stamp = time.monotonic()
    arm = A10ArmState(0, actual, target, pose, converted, stamp)
    return A10State(actual, target, pose, converted, gripper,
                    _optional_bool(follower.get('running_state'), 'running_state'),
                    mode, bool(has_error), stamp, reply, MappingProxyType({arm_name: arm}))


def _speed_override(state):
    context = state.raw['ret_context']
    mode = context.get('robot_msg', {}).get('op_mode')
    if mode == 'Manual':
        percent = context.get('motion_msg', {}).get('jog_vel')
    elif mode == 'Auto':
        percent = context.get('program_msg', {}).get('vel')
    else:
        raise A10ProtocolError(f'Unsupported speed override mode: {mode!r}')
    if type(percent) not in (int, float) or not math.isfinite(percent) or not 0 <= percent <= 100:
        raise A10ProtocolError('Controller speed percentage is missing or invalid')
    return dict(mode=mode, percent=percent)


def _validate_speed_override(percent, expected_mode):
    if type(percent) not in (int, float) or not math.isfinite(percent) or not 0 <= percent <= 100:
        raise ValueError('Speed percentage must be finite within 0..100')
    if expected_mode not in ('Manual', 'Auto'):
        raise ValueError('expected_mode must be Manual or Auto')


def _set_controller_speed(connection, get_state, percent, expected_mode, timeout_s, expected_percent=None):
    """Caller holds the direct/shared control lock for the whole transaction."""
    _validate_speed_override(percent, expected_mode)
    def check(state):
        info = state.raw['ret_context'].get('robot_msg', {})
        if (state.has_error or state.follower_active is not False
                or info.get('motion') != 'Stopped' or info.get('op_mode') != expected_mode):
            raise RuntimeError('Speed change requires healthy Stopped feedback, inactive follower and unchanged mode')
    initial = get_state(timeout_s=timeout_s)
    check(initial)
    if expected_percent is not None and _speed_override(initial)['percent'] != expected_percent:
        raise RuntimeError('Controller speed changed; preserving site settings')
    command = 'set_jog_vel' if expected_mode == 'Manual' else 'set_pgm_vel'
    connection.request(f'{command} --vel_percent={percent:g}')
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        state = get_state(timeout_s=max(.001, deadline - time.monotonic()))
        check(state)
        result = _speed_override(state)
        if result['percent'] == percent:
            return result
        time.sleep(min(.02, max(0., deadline - time.monotonic())))
    raise TimeoutError('Controller speed percentage readback did not match')


class _FollowerConnection:
    """Serialized WS transactions plus a separate UDP socket; no robot logic."""

    def __init__(self, host, ws_port, udp_port, timeout_s, joint_udp_port=9999):
        self.host, self.ws_port, self.udp_port = host, ws_port, udp_port
        self.timeout_s = timeout_s
        self.joint_udp_port = joint_udp_port
        self.ws = self.udp = self.joint_udp = None
        self._request_pending = threading.Event()
        self.command_cancelled = None
        self._ws_trace = deque(maxlen=32)
        self._ws_addresses = None
        self.last_request_failure = None

    def connect(self):
        from websockets.sync.client import connect
        try:
            self.ws = connect(f'ws://{self.host}:{self.ws_port}', proxy=None,
                              # KAANH command replies are our health check. Do
                              # not require an independently unverified Pong
                              # implementation during an idle terminal session.
                              ping_interval=None,
                              open_timeout=self.timeout_s, close_timeout=self.timeout_s,
                              max_size=_MAX_PAYLOAD + _HEADER.size)
            self._ws_trace.clear()
            self.last_request_failure = None
            self._ws_addresses = dict(local=list(self.ws.local_address),
                                      remote=list(self.ws.remote_address))
            self._ws_trace.append(dict(event='connected', wall_ns=time.time_ns(),
                                       addresses=self._ws_addresses))
            self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp.settimeout(self.timeout_s)
            self.udp.connect((self.host, self.udp_port))
            self.joint_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.joint_udp.settimeout(self.timeout_s)
            self.joint_udp.connect((self.host, self.joint_udp_port))
        except Exception:
            self.close()
            raise

    def send_command(self, command):
        if (self.command_cancelled is not None and self.command_cancelled.is_set()
                and command.split(maxsplit=1)[0] not in (
                    'get', 'stop_follower', 'manual_stop', 'set_jog_vel', 'set_pgm_vel')):
            raise InterruptedError('当前命令已取消，未发送控制器命令')
        if self.ws is None:
            raise ConnectionError('A10 operator is not connected')
        try:
            self.ws.send(_pack(command.encode('utf-8'), command=True))
            # Local send completion is not a controller execution acknowledgement.
            self._ws_trace.append(dict(event='sent', wall_ns=time.time_ns(),
                                       command=command.split(maxsplit=1)[0]))
        except Exception:
            self.close()
            raise

    def request(self, command, *, status=False, timeout_s=None, raw=False):
        budget = self.timeout_s if timeout_s is None else timeout_s
        deadline = time.monotonic() + budget
        self._request_pending.set()
        try:
            self.send_command(command)
            for _ in range(32):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('A10 reply deadline exceeded')
                reply = _decode_reply(self.ws.recv(timeout=remaining), raw=raw)
                context = reply.get('ret_context')
                kind = ('status' if isinstance(context, dict) and 'motion_msg' in context
                        else 'empty_ack' if context in ('', None, {}) and not reply.get('ret_msg')
                        else 'other')
                self._ws_trace.append(dict(event='received', wall_ns=time.time_ns(),
                                           kind=kind, ret_code=reply.get('ret_code')))
                if raw:
                    return reply
                is_status = isinstance(context, dict) and 'motion_msg' in context
                if status and is_status:
                    return reply
                if not status and not is_status:
                    return reply
                if status and (context not in ('', None, {}) or reply.get('ret_msg', '')):
                    raise A10ProtocolError('Unexpected response while waiting for status')
                # Bounded skipping of delayed empty ACKs, or old status replies.
            raise A10ProtocolError('Too many unrelated replies')
        except Exception as exc:
            self.last_request_failure = dict(
                host=self.host, port=self.ws_port, command=command.split(maxsplit=1)[0],
                addresses=self._ws_addresses,
                timeout_s=budget, failed_wall_ns=time.time_ns(),
                error=f'{type(exc).__name__}: {exc}', events=list(self._ws_trace))
            logging.getLogger(__name__).error(
                'A10 WS failure trace (sent means local send only; ACK has no command ID): %s',
                json.dumps(self.last_request_failure, ensure_ascii=False))
            # An uncertain transaction cannot be reused for a later command.
            self.close()
            if isinstance(exc, TimeoutError):
                raise TimeoutError(
                    f'A10 command {command!r} reply timed out after {budget:.3f}s') from exc
            raise
        finally:
            self._request_pending.clear()

    def interrupt(self):
        """Wake a blocked receive; the requesting thread closes the stale socket."""
        ws = self.ws
        if self._request_pending.is_set() and ws is not None:
            try:
                ws.socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    def close(self):
        ws, udp, joint_udp = self.ws, self.udp, self.joint_udp
        self.ws = self.udp = self.joint_udp = None
        try:
            if ws is not None:
                ws.close()
        finally:
            try:
                if udp is not None:
                    udp.close()
            finally:
                if joint_udp is not None:
                    joint_udp.close()


@OPERATORS.register_module()
class A10Operator(BaseOperator):
    """Six-axis A10 + gripper, using the reference KAANH follower protocol.

    Legacy ``send_eepose({'right': pose})`` accepts xyz metres
    + xyzw: T_relative = inverse(T_start) * T_target. Each follower start
    resets SDK cumulative progress. Tool packets are accumulated component
    coordinates, not exact fixed-anchor pose targets for mixed rotations.
    ``cartesian_wire_format='pe321'`` keeps this public input and encodes signed
    Euler ZYX radians in ``type=321, pe`` to avoid the SDK pq angle wrap.
    ``send_joints({'right': joints})`` expects six absolute joint angles in
    radians. They are converted to degrees in UDP ``jp --motor_pos=[...]``;
    the gripper remains a separate command. Only the selected follower mode
    may receive targets, after this session's start is confirmed in feedback.
    The opt-in SDK tool interface converts planned base poses separately. Joint feedback
    defaults to degrees on the wire per the template; set feedback_joint_unit
    to the verified controller convention. Position units are unknown by
    default. get_state() remains controller-only. get_frame() can compose
    cameras and an explicitly selected USB GripperOperator. Configure
    gripper_feedback_path only for a verified controller millimetre field.

    Motion commands require allow_motion=True. close() closes network resources
    only; call stop_follower() explicitly to request and confirm mode exit. Mode
    exit is not a physical emergency stop. No automatic motor enable or login.

    Explicit ``arms`` maps logical names to controller ee_id, joint_group,
    joint_indices (six entries) and tcp_index. ``cartesian_target_order`` is
    the XML control-group row order, not inferred from left/right or ee_id.
    Two arms share one connection/follower; all targets must be supplied in
    one call. Joint targets flatten in joint_target_order (default ascending
    ee_id). Dual gripper trajectories remain unsupported. The local m_init
    command repeats its configured absolute target through send_joints.
    A10State.arms / get_frame()['arms'] expose named model feedback. Legacy
    flat fields remain available only for a single configured arm.
    """

    capabilities = frozenset({'relative_cartesian_follower', 'absolute_joint_follower',
                              'gripper_position'})
    command_mode = 'cartesian'

    def __init__(self, host, ws_port=5999, udp_port=9998, timeout_s=2.0,
                 allow_motion=False, arm_name='right', feedback_joint_unit='deg',
                 feedback_position_unit=None, gripper_feedback_path=None,
                 joint_udp_port=9999, command_mode='cartesian',
                 joint_follower_velocity_percent=None,
                 cameras=None, gripper=None, gripper_transport='controller',
                 continue_on_gripper_timeout=False,
                 trajectory_feedback_interval_s=0.1,
                 trajectory_feedback_timeout_s=0.1,
                 trajectory_max_lateness_s=None, status_poll_interval_s=None,
                 status_max_age_s=0.5, status_ws_port=5888,
                 command_timeout_s=60.0, record_trajectory_trace=False, shared_control=None,
                 cartesian_wire_format='pq', arms=None, cartesian_target_order=None,
                 joint_target_order=None, init_position=None,
                 init_gripper_positions=None, init_motion=None):
        super().__init__(sync_warning_enabled=False)
        if not isinstance(host, str) or not host or any(c in host for c in '/?#@'):
            raise ValueError('host must be a controller hostname or IP')
        if any(type(p) is not int or not 1 <= p <= 65535
               for p in (ws_port, udp_port, joint_udp_port, status_ws_port)):
            raise ValueError('Ports must be integers within 1..65535')
        if joint_udp_port == udp_port:
            raise ValueError('Cartesian and joint UDP ports must differ')
        if (joint_follower_velocity_percent is not None
                and (type(joint_follower_velocity_percent) is not int
                     or not 1 <= joint_follower_velocity_percent <= 100)):
            raise ValueError('joint_follower_velocity_percent must be an integer within 1..100 or None')
        self.joint_follower_velocity_percent = joint_follower_velocity_percent
        if command_mode not in _FOLLOWER_MODES:
            raise ValueError('command_mode must be cartesian or joint')
        if cartesian_wire_format not in ('pq', 'pe321'):
            raise ValueError('cartesian_wire_format must be pq or pe321')
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('timeout_s must be positive and finite')
        if (isinstance(command_timeout_s, bool) or not math.isfinite(command_timeout_s)
                or command_timeout_s <= 0):
            raise ValueError('command_timeout_s must be positive and finite')
        self.command_timeout_s = float(command_timeout_s)
        if type(allow_motion) is not bool or not isinstance(arm_name, str) or not arm_name:
            raise ValueError('allow_motion must be bool and arm_name must be nonempty')
        self._arm_specs, self.cartesian_target_order = _normalize_arms(arms, arm_name, cartesian_target_order)
        self.arm_names = tuple(self._arm_specs) if self._arm_specs is not None else (arm_name,)
        self.is_multi_arm = len(self.arm_names) > 1
        arm_ids = ({name: spec['ee_id'] for name, spec in self._arm_specs.items()}
                   if self._arm_specs is not None else {arm_name: 0})
        order = (sorted(self.arm_names, key=arm_ids.__getitem__)
                 if joint_target_order is None else joint_target_order)
        if (not isinstance(order, (list, tuple)) or len(order) != len(self.arm_names)
                or any(not isinstance(name, str) for name in order)
                or set(order) != set(self.arm_names)):
            raise ValueError('joint_target_order must list every configured arm exactly once')
        self.joint_target_order = tuple(order)
        self.joint_dofs = {name: 6 for name in self.arm_names}
        self.init_position = None
        if init_position is not None:
            if not isinstance(init_position, Mapping) or set(init_position) != set(self.arm_names):
                raise ValueError('init_position must provide every configured arm')
            self.init_position = {name: _vector(init_position[name], 6, f'{name} init_position')
                                  for name in self.arm_names}
        self.init_gripper_positions = ({name: 0. for name in self.arm_names}
                                       if init_gripper_positions is None else dict(init_gripper_positions))
        self.init_motion = {} if init_motion is None else dict(init_motion)
        self.last_init_motion = None
        if self.is_multi_arm and (gripper is not None or gripper_transport != 'controller'
                                  or gripper_feedback_path is not None):
            raise NotImplementedError('Dual-arm gripper routing is not configured')
        if self.is_multi_arm:
            self.capabilities = frozenset({'relative_cartesian_follower', 'absolute_joint_follower'})
        if feedback_joint_unit not in ('deg', 'rad'):
            raise ValueError('feedback_joint_unit must be deg or rad')
        if feedback_position_unit not in (None, 'mm', 'm'):
            raise ValueError('feedback_position_unit must be None, mm or m')
        if gripper_feedback_path is not None and (
                not isinstance(gripper_feedback_path, (list, tuple))
                or not gripper_feedback_path
                or any(not isinstance(k, str) or not k for k in gripper_feedback_path)):
            raise ValueError('gripper_feedback_path must be a nonempty list of keys')
        if gripper_transport not in ('controller', 'usb'):
            raise ValueError('gripper_transport must be controller or usb')
        if gripper_transport == 'usb' and not isinstance(gripper, Mapping):
            raise ValueError('USB gripper requires explicit gripper settings')
        if gripper_transport == 'controller' and gripper is not None:
            raise ValueError('USB settings require gripper_transport="usb"')
        reserved = {'arms', 'qpos', 'joints', 'gripper_mm', 'tcp_pq_m', 'follower_active',
                    'follower_mode', 'has_error', 'received_monotonic_s',
                    'gripper_received_monotonic_s', 'camera_received_monotonic_s'}
        if cameras is not None and (not isinstance(cameras, dict)
                or any(not isinstance(k, str) or not k or k in reserved for k in cameras)):
            raise ValueError('Camera names must be nonempty and not overwrite robot fields')
        self.camera_operator = None if cameras is None else CameraOperator(cameras)
        self.continue_on_gripper_timeout = bool(continue_on_gripper_timeout)
        self._gripper_timed_out = False
        self.gripper_transport = gripper_transport
        self.gripper_operator = (GripperOperator(**dict(gripper))
                                 if gripper_transport == 'usb' else None)
        self.gripper_max_width_mm = (self.gripper_operator.max_width_mm
                                    if self.gripper_operator is not None else 100.)
        for name, value in [('trajectory_feedback_interval_s', trajectory_feedback_interval_s),
                            ('trajectory_feedback_timeout_s', trajectory_feedback_timeout_s),
                            ('trajectory_max_lateness_s', trajectory_max_lateness_s)]:
            if value is not None and (isinstance(value, bool)
                                      or not math.isfinite(value) or value <= 0):
                raise ValueError(f'{name} must be positive and finite')
        self.trajectory_feedback_interval_s = float(trajectory_feedback_interval_s)
        self.trajectory_feedback_timeout_s = float(trajectory_feedback_timeout_s)
        self.trajectory_max_lateness_s = trajectory_max_lateness_s
        if status_poll_interval_s is not None and (
                isinstance(status_poll_interval_s, bool) or not math.isfinite(status_poll_interval_s)
                or status_poll_interval_s <= 0):
            raise ValueError('status_poll_interval_s must be positive and finite')
        if isinstance(status_max_age_s, bool) or not math.isfinite(status_max_age_s) or status_max_age_s <= 0:
            raise ValueError('status_max_age_s must be positive and finite')
        self.status_poll_interval_s = status_poll_interval_s
        self.status_max_age_s = float(status_max_age_s)
        self.status_ws_port = status_ws_port
        self._trajectory_lock = threading.Lock()
        self._trajectory_owner = None
        self.last_trajectory = None
        self.record_trajectory_trace = bool(record_trajectory_trace)
        self._last_joint_send_monotonic_s = None
        self._trajectory_feedback_trace = []
        self.allow_motion = allow_motion
        self.arm_name = self.arm_names[0] if not self.is_multi_arm else None
        self.feedback_joint_unit = feedback_joint_unit
        self.feedback_position_unit = feedback_position_unit
        self.gripper_feedback_path = gripper_feedback_path
        self.timeout_s = float(timeout_s)
        self.command_mode = command_mode
        self.cartesian_wire_format = cartesian_wire_format
        self._sdk_tool_adapter = None
        self._sdk_tool_adapters = None
        self._cartesian_raw_used = False
        self._cartesian_euler = np.zeros(3)
        self._cartesian_eulers = {name: np.zeros(3) for name in self.arm_names}
        self.shared_control = dict(shared_control) if shared_control else None
        self._shared_control_options = dict(self.shared_control) if self.shared_control else None
        self._init_shared_reserved = False
        if self.shared_control and self.shared_control.get('mode', 'shared') not in ('shared', 'auto', 'managed'):
            raise ValueError('shared_control.mode must be shared, auto or managed')
        if self.shared_control:
            from ..communication.a10_shared_connection import SharedFollowerConnection
            self._connection = SharedFollowerConnection(host, ws_port, udp_port, self.timeout_s,
                                                        joint_udp_port, shared=self.shared_control)
        else:
            self._connection = _FollowerConnection(host, ws_port, udp_port, self.timeout_s,
                                                   joint_udp_port)
        self.last_stop_confirmed = False
        self._status_connection = None
        self._status_thread = None
        self._status_stop = threading.Event()
        self._feedback_reader = None
        self._feedback_failure = None
        self._latest_state = None
        self._status_error = None
        self._status_lock = threading.Lock()
        self._io_lock = threading.RLock()
        self._command_cancelled = threading.Event()
        self._confirmed_active = False
        self._started_here = False
        self._start_confirmed = False
        self._follower_mode = None

    @property
    def is_connected(self):
        return (self._connection.ws is not None and self._connection.udp is not None
                and self._connection.joint_udp is not None)

    @property
    def follower_stop_pending(self):
        """This session sent a start that hasn't been confirmed inactive yet."""
        return self._started_here

    @property
    def last_communication_failure(self):
        """Latest command/cache transport failure, retained through cleanup."""
        failures = [getattr(connection, 'last_request_failure', None)
                    for connection in (self._connection, self._feedback_reader)]
        failures.append(self._feedback_failure)
        return max((failure for failure in failures if failure is not None),
                   key=lambda failure: failure.get('failed_wall_ns', 0), default=None)

    def _parse_feedback(self, reply):
        return _parse_state(reply, joint_unit=self.feedback_joint_unit,
                            position_unit=self.feedback_position_unit,
                            gripper_feedback_path=self.gripper_feedback_path,
                            arms=self._arm_specs, arm_name=self.arm_name)

    def _reset_cartesian_encoding(self):
        self._sdk_tool_adapter = None
        self._sdk_tool_adapters = None
        self._cartesian_raw_used = False
        self._cartesian_euler = np.zeros(3)
        self._cartesian_eulers = {name: np.zeros(3) for name in self.arm_names}

    def _require_single_arm(self, operation):
        if self.is_multi_arm:
            raise NotImplementedError(f'Dual-arm {operation} is not supported; no command was sent')

    def _has_model_feedback(self, state, mode):
        return (set(state.arms) == set(self.arm_names) and
                all(arm.actual_joints_rad is not None and
                    (mode != 'Cart' or arm.tcp_pq_raw is not None) for arm in state.arms.values()))

    def connect(self):
        with self._io_lock:
            if not self.is_connected:
                with self._status_lock:
                    self._latest_state = None
                    self._status_error = None
                self._confirmed_active = self._started_here = False
                self._start_confirmed = False
                self._follower_mode = None
                try:
                    self._connect_robot_transport()
                    if self.gripper_operator is not None:
                        self._gripper_timed_out = False
                        self.gripper_operator.connect()
                    if self.status_poll_interval_s is not None:
                        if self.shared_control:
                            from ..communication.a10_shared_connection import SharedFollowerConnection
                            self._status_connection = SharedFollowerConnection(
                                self._connection.host, self.status_ws_port,
                                self._connection.udp_port, self.timeout_s,
                                self._connection.joint_udp_port, shared=self.shared_control,
                                session_id=self._connection.session_id, status_channel=True)
                        else:
                            self._status_connection = _FollowerConnection(
                                self._connection.host, self.status_ws_port,
                                self._connection.udp_port, self.timeout_s,
                                self._connection.joint_udp_port)
                        self._status_connection.connect()
                        # Bootstrap on the monitor port too. Control-port GET
                        # must not prevent the status monitor from starting.
                        reply = self._status_connection.request('get', status=True)
                        self._latest_state = self._parse_feedback(reply)
                        self._status_stop.clear()
                        self._status_thread = threading.Thread(
                            target=self._poll_status, name='a10-status', daemon=True)
                        self._status_thread.start()
                except BaseException:
                    self.close()
                    raise

    def _connect_robot_transport(self):
        options = self._shared_control_options
        if not options or options.get('mode') != 'auto':
            self._connection.connect()
            return
        from ..communication.a10_shared_connection import (
            SharedFollowerConnection, SharedServiceUnavailable)
        previous = self._connection
        previous.close()
        self.shared_control = dict(options)
        self._connection = SharedFollowerConnection(
            previous.host, previous.ws_port, previous.udp_port, self.timeout_s,
            previous.joint_udp_port, shared=options)
        try:
            self._connection.connect()
        except SharedServiceUnavailable:
            # Only an initial local refusal means "service absent". A bad
            # hello, timeout or broken service must not silently go direct.
            self.shared_control = None
            self._connection = _FollowerConnection(
                previous.host, previous.ws_port, previous.udp_port, self.timeout_s,
                previous.joint_udp_port)
            self._connection.connect()
            logging.getLogger(__name__).info(
                'A10 connection: direct to %s:%s (no local shared service)',
                previous.host, previous.ws_port)
        else:
            logging.getLogger(__name__).info(
                'A10 connection: reused shared service %s:%s -> %s:%s',
                options.get('host', '127.0.0.1'), options.get('port', 5870),
                previous.host, previous.ws_port)

    def close(self):
        feedback_errors = []
        try:
            self.stop_feedback_polling()
        except Exception as exc:
            feedback_errors.append(exc)
        self._traj_stop_event.set()
        self._status_stop.set()
        if self._status_connection is not None:
            self._status_connection.close()
        thread, self._status_thread = self._status_thread, None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=self.timeout_s + .2)
        with self._io_lock:
            self._confirmed_active = self._started_here = False
            self._start_confirmed = False
            self._follower_mode = None
            self._reset_cartesian_encoding()
            errors = feedback_errors
            closers = [self._connection.close]
            if self.camera_operator is not None:
                closers.append(self.camera_operator.close)
            if self.gripper_operator is not None:
                closers.append(self.gripper_operator.disconnect)
            for closer in closers:
                try:
                    closer()
                except Exception as exc:
                    errors.append(exc)
            if errors:
                raise RuntimeError(f'A10 resource cleanup failed: {errors}') from errors[0]

    def _poll_status(self):
        while not self._status_stop.is_set():
            try:
                reply = self._status_connection.request('get', status=True)
                state = self._parse_feedback(reply)
                with self._status_lock:
                    if (self._latest_state is None or state.received_monotonic_s
                            >= self._latest_state.received_monotonic_s):
                        self._latest_state = state
            except Exception as exc:
                with self._status_lock:
                    self._status_error = exc
                self._status_stop.set()
                return
            self._status_stop.wait(self.status_poll_interval_s)

    def _recent_state(self):
        with self._status_lock:
            state, error = self._latest_state, self._status_error
        if error is not None:
            raise ConnectionError(f'A10 status polling failed: {error}') from error
        if state is None or time.monotonic() - state.received_monotonic_s > self.status_max_age_s:
            raise TimeoutError(f'A10 status feedback older than {self.status_max_age_s:.3f}s')
        return state

    def _require_motion(self, *, allow_cancelled=False):
        if not self.allow_motion:
            raise PermissionError('A10 allow_motion is false')
        if not allow_cancelled and self._command_cancelled.is_set():
            raise InterruptedError('当前命令已取消，禁止继续下发动作')
        if not self.is_connected:
            raise ConnectionError('A10 operator is not connected')

    def _runtime_recording_request(self, command):
        """Recorder commands share the control WS transaction lock, not motion ownership."""
        with self._io_lock:
            if self.status_poll_interval_s is not None:
                raise ValueError('Runtime recording requires one WebSocket transaction stream without status polling')
            if not self.is_connected:
                raise ConnectionError('A10 operator is not connected')
            reply = self._connection.request(command)
            state = reply.get('ret_context')
            if (not isinstance(state, dict)
                    or any(type(state.get(k)) is not bool for k in ('connected', 'recording', 'closing'))
                    or any(not isinstance(state.get(k), str) for k in ('session_id', 'file', 'stop_reason', 'error'))
                    or any(type(state.get(k)) is not int or state[k] < 0
                           for k in ('attempted', 'written', 'synced', 'last_synced_cycle', 'dropped'))
                    or type(state.get('heartbeat_timeout_ms')) is not int
                    or state['heartbeat_timeout_ms'] <= 0):
                raise A10ProtocolError('Invalid runtime recorder status reply')
            return state

    def start_runtime_recording(self):
        """Explicitly start controller logging; never enable or start the robot."""
        state = self._runtime_recording_request('record_start')
        if (not state['connected'] or not state['recording'] or state['closing']
                or not state['session_id'] or not state['file'] or state['error']):
            raise RuntimeError(f'Controller recording did not start: {state}')
        return state

    def get_runtime_recording_status(self):
        return self._runtime_recording_request('record_status')

    def heartbeat_runtime_recording(self):
        return self._runtime_recording_request('record_heartbeat')

    def begin_console_command(self):
        """Only a newly submitted console command clears the cancellation latch."""
        self._command_cancelled.clear()
        self._connection.command_cancelled = self._command_cancelled

    def cancel_current_command(self):
        """Latch cancellation without waiting for the command's I/O lock."""
        self._command_cancelled.set()
        self._connection.command_cancelled = self._command_cancelled
        self._traj_stop_event.set()
        self._connection.interrupt()

    def stop_all_actions(self):
        """Cancel sends, request follower exit AND controller motion stop.

        stop_follower only clears the SDK's running flag. A running real-time
        plan can remain active, so explicit console stop also sends manual_stop.
        Only fresh inactive + Stopped feedback confirms completion.
        """
        self.cancel_current_command()
        if not self.allow_motion:
            raise PermissionError('A10 allow_motion is false')
        with self._io_lock:
            self.last_stop_confirmed = False
            if not self.is_connected:
                # Recover only the transport needed to send stop. Never replay
                # a command, enable a motor or reopen accessories here.
                self._connection.connect()
            self._confirmed_active = self._start_confirmed = False
            if self.shared_control:
                self._connection.call('stop')
            else:
                self._connection.send_command('stop_follower')
                self._connection.send_command('manual_stop')
            state = self.wait_stopped()
            self._started_here = False
            self._follower_mode = None
            self._reset_cartesian_encoding()
            self.last_stop_confirmed = True
            return state

    def get_state(self, *, timeout_s=None):
        with self._io_lock:
            try:
                reply = self._connection.request('get', status=True, timeout_s=timeout_s)
                state = self._parse_feedback(reply)
                self._confirmed_active = (self._started_here
                                          and self._start_confirmed
                                          and state.follower_active is True
                                          and state.follower_mode == self._follower_mode
                                          and state.raw['ret_context'].get('robot_msg', {}).get('activate') == 'Enabled'
                                          and not state.has_error
                                          and self._has_model_feedback(state, self._follower_mode))
                with self._status_lock:
                    if (self._latest_state is None or state.received_monotonic_s
                            >= self._latest_state.received_monotonic_s):
                        self._latest_state = state
                return state
            except Exception:
                self._confirmed_active = False
                self._connection.close()
                raise

    def send_controller_command(self, command):
        """Send controller text verbatim and return its complete reply.

        This is a control operation, including for unknown command names. A
        reply is not motion confirmation. Unknown commands invalidate local
        target permission; only the normal start path can grant it again.
        """
        if not isinstance(command, str) or not command.strip():
            raise ValueError('Controller command must be nonempty text')
        if any(c in command for c in ('\r', '\n', '\x00')):
            raise ValueError('Send one controller command per line')
        with self._io_lock:
            self._require_motion()
            self._require_target_owner()
            state = self.get_state()
            self._confirmed_active = self._start_confirmed = False
            # Only a known follower start creates a cleanup obligation before
            # sending. m_init and arbitrary commands are not follower starts.
            was_inactive = state.follower_active is False
            command_name = command.split(maxsplit=1)[0]
            if was_inactive and command_name in ('follower_cart', 'follower_aj', 'follower_aj_tg'):
                self._started_here = True
            try:
                reply = self._connection.request(
                    command, raw=True, timeout_s=self.command_timeout_s)
            except TimeoutError as exc:
                raise TimeoutError(
                    f'控制器指令 {command!r} 在发送后 {self.command_timeout_s:g} 秒内未收到回复；'
                    '执行结果未知，不表示指令未执行或机器人已停止。'
                    '为避免迟到回复串线，已关闭本地控制连接；输入 reconnect 重新查询状态。'
                    '不会自动重发；可在启动控制台时用 --command-timeout 增加回复等待时间。') from exc
            try:
                state = self.get_state()
            except Exception as exc:
                raise A10ProtocolError(
                    'Controller reply: ' + json.dumps(reply, ensure_ascii=False)
                    + f'; subsequent state query failed: {exc}') from exc
            if state.follower_active is False:
                self._started_here = False
                self._follower_mode = None
            elif was_inactive and state.follower_active is True:
                # A completed transaction followed by an observed transition
                # provides evidence missing from an arbitrary-command timeout.
                self._started_here = True
            return reply

    def start_feedback_polling(self, hz=30.):
        """Subscribe to the shared service's single controller GET worker.

        The independent local cache reader never sends GET or takes _io_lock.
        Explicit command confirmations continue to use fresh synchronous GET.
        """
        if (isinstance(hz, bool) or not isinstance(hz, (int, float))
                or not math.isfinite(hz) or not 0 < hz <= 100):
            raise ValueError('feedback poll hz must be within (0, 100]')
        from ..communication.a10_shared_connection import SharedFollowerConnection
        if not isinstance(self._connection, SharedFollowerConnection):
            raise ValueError('Independent feedback polling requires shared_control')
        if self.status_poll_interval_s is not None:
            raise ValueError('Do not combine independent feedback polling with status_poll_interval_s')
        if not self.is_connected:
            raise ConnectionError("A10 operator is not connected")
        metrics = self._connection.call('feedback_poll', hz=hz)
        if self._feedback_reader is None:
            try:
                self._feedback_reader = self.open_feedback_reader()
                self._feedback_failure = None
            except BaseException:
                self._connection.call('feedback_poll', hz=None)
                raise
        return metrics

    def stop_feedback_polling(self):
        reader, self._feedback_reader = self._feedback_reader, None
        if reader is None:
            return
        self._feedback_failure = getattr(reader, 'last_request_failure', None)
        try:
            if self.is_connected:
                self._connection.call('feedback_poll', hz=None)
        finally:
            reader.close()

    def get_cached_frame(self):
        """Measured controller cache with original time/sequence; caller checks age."""
        reader = self._feedback_reader
        if reader is None:
            raise RuntimeError('Call start_feedback_polling before reading cached frames')
        return self.get_feedback_snapshot(reader)

    def open_feedback_reader(self):
        """Optional local cache channel; never opens another robot socket."""
        from ..communication.a10_shared_connection import SharedFollowerConnection
        if not isinstance(self._connection, SharedFollowerConnection):
            return None  # Direct transport publishes every GET in _latest_state.
        reader = SharedFollowerConnection(
            self._connection.host, self.status_ws_port, self._connection.udp_port,
            self.timeout_s, self._connection.joint_udp_port,
            shared=self.shared_control, session_id=self._connection.session_id, status_channel=True)
        reader.connect()
        try:
            reader.call('feedback_snapshot')  # Fail clearly if an old relay is still running.
        except BaseException:
            reader.close()
            raise RuntimeError('共享服务尚不支持录制反馈快照；请退出旧控制台和VR窗口，'
                               '等待5秒，再按原命令启动控制台和录制') from None
        return reader

    @staticmethod
    def _feedback_frame(state):
        return dict(joints=state.actual_joints_rad, tcp_pq_m=state.tcp_pq_m,
                    follower_active=state.follower_active, follower_mode=state.follower_mode,
                    has_error=state.has_error, received_monotonic_s=state.received_monotonic_s,
                    arms={name: dict(ee_id=arm.ee_id, joints=arm.actual_joints_rad,
                                     target_joints=arm.target_joints_rad, tcp_pq_m=arm.tcp_pq_m,
                                     received_monotonic_s=arm.received_monotonic_s)
                          for name, arm in state.arms.items()})

    def get_feedback_snapshot(self, reader=None):
        """Read measured controller feedback without the motion/transaction lock.

        No USB/camera reads, new robot GET or invented receive timestamp.
        Callers must check freshness against their own acquisition limits.
        """
        if reader is not None:
            snapshot = reader.call('feedback_snapshot')
            if not isinstance(snapshot, dict):
                raise RuntimeError('Waiting for measured robot feedback')
            stamp = snapshot.get('received_monotonic_s')
            if (isinstance(stamp, bool) or not isinstance(stamp, (int, float))
                    or not math.isfinite(stamp) or not 0 <= stamp <= time.monotonic()):
                raise A10ProtocolError('Invalid measured feedback timestamp')
            state = self._parse_feedback(snapshot['reply'])
            state = replace(state, received_monotonic_s=stamp,
                            arms={name: replace(arm, received_monotonic_s=stamp)
                                  for name, arm in state.arms.items()})
        else:
            with self._status_lock:
                state = self._latest_state
            if state is None:
                raise RuntimeError('Waiting for measured robot feedback')
        frame = self._feedback_frame(state)
        if reader is not None:
            frame['controller_feedback_sequence'] = snapshot['sequence']
            frame['feedback_polling'] = snapshot.get('polling', {})
        return frame

    def get_frame(self, slop=0.7):
        """Actual robot + optional USB gripper + RGB frames, with receive times.

        These sequential reads are not synchronized sensor samples. Missing
        configured hardware raises; targets are never substituted for feedback.
        Opt-in gripper timeout continuation retains the last measured width and
        its timestamp, marks it stale, and skips USB traffic until reconnect.
        get_state() remains the unmodified controller-only status interface.
        """
        del slop
        with self._io_lock:
            try:
                state = (self._recent_state() if self.status_poll_interval_s is not None
                         else self.get_state())
                width = state.gripper_mm
                frame = self._feedback_frame(state)
                if self.gripper_operator is not None:
                    grip = self._read_gripper_state()
                    frame['gripper_feedback_stale'] = self._gripper_timed_out
                    width = grip.position_mm
                    frame['gripper_received_monotonic_s'] = grip.timestamp
                frame['gripper_mm'] = width
                frame['qpos'] = (None if state.actual_joints_rad is None or width is None
                                 else np.concatenate((state.actual_joints_rad, [width])))
                if self.camera_operator is not None:
                    frame.update(self.camera_operator.get_frame())
                return frame
            except BaseException:
                # Keep a usable control connection for the caller's explicit
                # stop in finally if a camera/servo read fails mid-run.
                self._confirmed_active = self._start_confirmed = False
                if not self._started_here:
                    self.close()
                raise

    def wait_stopped(self, *, timeout_s=None):
        """Confirm reported inactive follower + Stopped via fresh GET replies.

        Does not issue any command or claim a hardware emergency stop. It uses
        explicit queries rather than the monitor cache. A timeout requires
        onsite inspection; it must never trigger automatic enable or restart.
        """
        budget = self.timeout_s if timeout_s is None else timeout_s
        if isinstance(budget, bool) or not math.isfinite(budget) or budget <= 0:
            raise ValueError('timeout_s must be positive and finite')
        deadline = time.monotonic() + budget
        state = None
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if state is not None and remaining < min(.02, budget / 10):
                break
            state = self.get_state(timeout_s=max(.001, remaining))
            if state.has_error:
                raise A10ControllerError('Controller reports a fault while confirming stop')
            info = state.raw['ret_context'].get('robot_msg', {})
            if state.follower_active is False and info.get('motion') == 'Stopped':
                return state
            time.sleep(min(.02, max(0., deadline - time.monotonic())))
        info = {} if state is None else state.raw['ret_context'].get('robot_msg', {})
        raise TimeoutError(
            'Inactive follower and Stopped feedback were not both confirmed; '
            f'last follower_active={None if state is None else state.follower_active}, '
            f'motion={info.get("motion")}, activate={info.get("activate")}. '
            'Confirm stopping on the controller; do not restart motion.')

    def _wait_mode(self, active):
        deadline = time.monotonic() + self.timeout_s
        state = None
        while time.monotonic() < deadline:
            if active:
                self._require_motion()
            remaining = deadline - time.monotonic()
            if state is not None and remaining < min(.02, self.timeout_s / 10):
                break
            state = self.get_state(timeout_s=max(.001, remaining))
            stopped = (active or not self.is_multi_arm or
                       state.raw['ret_context'].get('robot_msg', {}).get('motion') == 'Stopped')
            if (state.follower_active is active and stopped
                    and (not active or state.follower_mode == self._follower_mode)):
                if active and state.has_error:
                    raise A10ControllerError('Controller reports a fault')
                if active and state.raw['ret_context'].get('robot_msg', {}).get('activate') != 'Enabled':
                    raise A10ControllerError('Follower running but motion permission is not confirmed')
                if active and not self._has_model_feedback(state, self._follower_mode):
                    raise A10ProtocolError('Follower running but configured model feedback is missing')
                if active:
                    self._start_confirmed = self._confirmed_active = True
                return state
            if state.has_error and active:
                raise A10ControllerError('Controller reports a fault')
            time.sleep(min(.02, max(0., deadline - time.monotonic())))
        self._confirmed_active = False
        info = {} if state is None else state.raw['ret_context'].get('robot_msg', {})
        detail = (f'; last follower_active={None if state is None else state.follower_active}, '
                  f'mode={None if state is None else state.follower_mode}, '
                  f'motion={info.get("motion")}, activate={info.get("activate")}')
        if not active and self.is_multi_arm:
            raise TimeoutError('Dual follower stop requires inactive + Stopped feedback' + detail)
        raise TimeoutError('Follower mode change was not confirmed' + detail)




    def enable(self):
        """Request manual_en in Manual mode and confirm Enabled feedback."""
        return self._set_manual_enable(True)

    def disable(self):
        """Request manual_ds in Manual mode; this is not an emergency stop."""
        return self._set_manual_enable(False)

    def _set_manual_enable(self, enabled):
        with self._io_lock:
            self._require_motion()
            self._prepare_shared_console_command()
            state = self.get_state()
            info = state.raw['ret_context'].get('robot_msg', {})
            if info.get('op_mode') != 'Manual':
                raise RuntimeError('Manual enable/disable requires Manual mode; no automatic mode switch')
            if enabled:
                if state.has_error:
                    raise A10ControllerError('Controller reports a fault')
                if info.get('motion') != 'Stopped' or state.follower_active is not False:
                    raise RuntimeError('Enable requires Stopped feedback and inactive follower')
            self._confirmed_active = False
            self._connection.request('manual_en' if enabled else 'manual_ds')
            expected = 'Enabled' if enabled else 'Disabled'
            deadline = time.monotonic() + self.timeout_s
            while time.monotonic() < deadline:
                state = self.get_state(timeout_s=max(.001, deadline - time.monotonic()))
                info = state.raw['ret_context'].get('robot_msg', {})
                if info.get('op_mode') != 'Manual':
                    raise RuntimeError('Controller left Manual mode during enable/disable')
                if enabled and state.has_error:
                    raise A10ControllerError('Controller reports a fault')
                if info.get('activate') == expected:
                    return state
                time.sleep(min(.02, max(0., deadline - time.monotonic())))
            raise TimeoutError(f'{expected} feedback was not confirmed')

    def get_speed_override(self):
        """Read the current Manual/Auto controller speed percentage."""
        return _speed_override(self.get_state())

    def set_speed_override(self, percent, *, expected_mode, expected_percent=None,
                           allow_cancelled=False):
        """Set speed while stopped; cleanup may restore it without clearing cancellation."""
        _validate_speed_override(percent, expected_mode)
        with self._io_lock:
            self._require_motion(allow_cancelled=allow_cancelled)
            from ..communication.a10_shared_connection import SharedFollowerConnection
            if isinstance(self._connection, SharedFollowerConnection):
                return self._connection.call('speed_override', percent=percent,
                                             expected_mode=expected_mode, expected_percent=expected_percent)
            return _set_controller_speed(self._connection, self.get_state, percent,
                                         expected_mode, self.timeout_s, expected_percent)

    def set_tool_coordinate(self):
        """Explicitly select Tool jog coordinates and verify feedback; no target sent."""
        return self.set_jog_coordinate('Tool')

    def get_shared_control_status(self):
        """Read local ownership without touching robot motion or another owner."""
        if not self.shared_control:
            return None
        with self._io_lock:
            status = self._connection.lease('status')
            if status.get('preempted'):
                self._started_here = self._confirmed_active = self._start_confirmed = False
                self._follower_mode = None
                self._reset_cartesian_encoding()
            return status

    def acknowledge_shared_takeover(self):
        with self._io_lock:
            return self._connection.lease('acknowledge')

    def begin_shared_console_operation(self):
        if self.shared_control and self.shared_control.get('role') == 'console':
            self._connection.lease('begin_console')

    def end_shared_console_operation(self):
        if self.shared_control and self.shared_control.get('role') == 'console' and self.is_connected:
            self._connection.lease('end_console')

    def _prepare_shared_console_command(self):
        if self.shared_control and self.shared_control.get('role') == 'console':
            self._connection.lease('prepare_command')
            self._started_here = self._confirmed_active = self._start_confirmed = False
            self._follower_mode = None

    def set_jog_coordinate(self, coordinate):
        """Select controller jog coordinates and verify feedback while stopped."""
        if coordinate not in ('Axis', 'Tool', 'Base', 'Wobj'):
            raise ValueError('coordinate must be Axis, Tool, Base or Wobj')
        with self._io_lock:
            self._require_motion()
            self._prepare_shared_console_command()
            state = self.get_state()
            if state.has_error:
                raise A10ControllerError('Controller reports a fault')
            if state.follower_active is not False:
                raise RuntimeError('Stop the follower before changing jog coordinates')
            if state.raw['ret_context'].get('robot_msg', {}).get('motion') != 'Stopped':
                raise RuntimeError('Coordinate change requires confirmed Stopped feedback')
            self._connection.request(f'set_jog_coordinate --{coordinate.lower()}')
            deadline = time.monotonic() + self.timeout_s
            while time.monotonic() < deadline:
                state = self.get_state(timeout_s=max(.001, deadline - time.monotonic()))
                if state.has_error or state.follower_active is not False:
                    raise A10ControllerError('Controller fault or follower state changed during coordinate selection')
                if state.raw['ret_context']['motion_msg'].get('jog_coordinate') == coordinate:
                    return state
                time.sleep(min(.02, max(0., deadline - time.monotonic())))
            raise TimeoutError(f'{coordinate} jog coordinate was not confirmed')

    def start_follower(self, mode=None):
        """Start cartesian (default) or joint follower; never send an initial target."""
        mode = self.command_mode if mode is None else mode
        if mode not in _FOLLOWER_MODES:
            raise ValueError('mode must be cartesian or joint')
        command, feedback_mode = _FOLLOWER_MODES[mode]
        if mode == 'joint' and self.joint_follower_velocity_percent is not None:
            command += f' --vel_percent={self.joint_follower_velocity_percent}'
        with self._io_lock:
            self._require_motion()
            if self._started_here:
                state = self.get_state()
                owns = not self.shared_control or self._connection.lease('status')['owns']
                if owns and self._confirmed_active and state.follower_mode == feedback_mode:
                    self.command_mode = mode
                    return state  # Already in this session's confirmed mode; no duplicate start.
                raise RuntimeError('This session has a pending follower; stop_follower before switching modes')
            if self.shared_control:
                self._connection.lease('acquire')
            self.last_stop_confirmed = False
            state = self.get_state()
            if state.has_error:
                raise A10ControllerError('Controller reports a fault')
            if state.follower_active is not False:
                raise RuntimeError('Follower is active or its state is unknown; refusing takeover')
            if not self._has_model_feedback(state, feedback_mode):
                raise A10ProtocolError('Follower start requires joint and TCP feedback')
            info = state.raw['ret_context'].get('robot_msg', {})
            if info.get('activate') != 'Enabled' or info.get('motion') != 'Stopped':
                raise RuntimeError(
                    'Follower start requires Enabled and Stopped feedback; '
                    f'activate={info.get("activate")}, motion={info.get("motion")}, '
                    f'follower_active={state.follower_active}. '
                    'Confirm Enabled and Stopped on the pendant or A10 console before VR takeover. '
                    '若 motion=Running，请先输入 stop 结束控制器当前命令并确认 Stopped')
            coordinate = state.raw['ret_context']['motion_msg'].get('jog_coordinate')
            if mode == 'cartesian' and coordinate not in ('Base', 'Tool', 'Wobj'):
                raise RuntimeError(
                    f'Cartesian follower requires Cartesian jog coordinates, got {coordinate!r}; '
                    'explicitly call set_tool_coordinate() before starting')
            # Record the attempt before send: Ctrl+C can arrive after bytes
            # leave the socket but before send() returns to Python.
            self._started_here = True
            self._start_confirmed = False
            self._confirmed_active = False
            self._follower_mode = feedback_mode
            self.command_mode = mode
            self._reset_cartesian_encoding()
            try:
                if mode == 'joint' and not self.shared_control:
                    # Enable the follower module before entering the joint plan.
                    # Shared mode performs both steps under its service control lock.
                    self._connection.request('start_follower')
                self._connection.send_command(command)
                return self._wait_mode(True)
            except A10ControllerError:
                # A rejection may precede the outstanding get reply. Never reuse
                # that queue or retry the start: inspect state on a fresh socket.
                # Only confirmed inactivity clears this attempt's cleanup flag.
                if not self.is_connected:
                    try:
                        self._connection.connect()
                        recovered = self.get_state()
                        if recovered.follower_active is False:
                            self._started_here = False
                            self._follower_mode = None
                    except Exception:
                        self._connection.close()
                self._confirmed_active = False
                raise

    def stop_follower(self):
        self._traj_stop_event.set()
        with self._io_lock:
            self._sdk_tool_adapter = None
            self._sdk_tool_adapters = None
            self._require_motion(allow_cancelled=True)
            if self.shared_control:
                lease = self._connection.lease('status')
                if (self.shared_control.get('role') != 'console'
                        and lease['stopped_by_service'] and not lease['owns']):
                    self._started_here = self._confirmed_active = self._start_confirmed = False
                    self._follower_mode = None
                    self.last_stop_confirmed = True
                    return self.get_state()
                if self.shared_control.get('role') == 'console':
                    self._connection.send_command('stop_follower')
                    state = self._wait_mode(False)
                    self._started_here = self._confirmed_active = self._start_confirmed = False
                    self._follower_mode = None
                    self.last_stop_confirmed = True
                    return state
            if not self._started_here:
                raise RuntimeError('This connection did not start the follower')
            self._confirmed_active = False
            self._start_confirmed = False
            self._connection.send_command('stop_follower')
            state = self._wait_mode(False)
            self._started_here = False
            self._follower_mode = None
            self.last_stop_confirmed = True
            return state

    def begin_sdk_tool_trajectory(self, startup_pose):
        """Initialize after a fresh Tool start; dual input is a complete pose map."""
        with self._io_lock:
            self._require_target_owner()
            self._require_motion()
            if (not self._started_here or not self._confirmed_active
                    or self._follower_mode != 'Cart' or self._sdk_tool_adapter is not None
                    or self._sdk_tool_adapters is not None or self._cartesian_raw_used):
                raise RuntimeError('SDK adapter requires a fresh confirmed Cartesian follower')
            if self.cartesian_wire_format != 'pe321':
                raise ValueError('SDK adapter requires cartesian_wire_format=pe321')
            state = self.get_state()
            if state.has_error or state.follower_active is not True or state.raw[
                    'ret_context']['motion_msg'].get('jog_coordinate') != 'Tool':
                raise RuntimeError('SDK adapter requires healthy active Tool feedback')
            if self.is_multi_arm:
                if not isinstance(startup_pose, Mapping) or set(startup_pose) != set(self.arm_names):
                    raise ValueError('SDK adapter requires the complete arm group')
                self._sdk_tool_adapters = {
                    name: SDKToolCumulativeAdapter(startup_pose[name])
                    for name in self.cartesian_target_order}
            else:
                self._sdk_tool_adapter = SDKToolCumulativeAdapter(startup_pose)

    def send_sdk_tool_pose(self, target, *, max_position_step_m, max_rotation_step_rad):
        """Send a base-frame planned pose as cumulative local 321 components.

        No actual-error integral or application ACK is implied by this method.
        """
        with self._io_lock:
            if self.is_multi_arm:
                return self._send_sdk_tool_poses(target, max_position_step_m, max_rotation_step_rad)
            adapter = self._sdk_tool_adapter
            if adapter is None:
                raise RuntimeError('SDK tool adapter is not initialized')
            target, cumulative = adapter.propose(
                target, max_position_step_m=max_position_step_m,
                max_rotation_step_rad=max_rotation_step_rad)
            payload = json.dumps(dict(type='321', pe=[cumulative.tolist()]),
                                 separators=(',', ':'), allow_nan=False).encode('utf-8')
            try:
                self._send_follower_target(_pack(payload, command=False), 'Cart')
            except BaseException:
                # Delivery may be uncertain. Never resume the cumulative stream.
                self._sdk_tool_adapter = None
                self._confirmed_active = False
                raise
            adapter.commit(target, cumulative)
            return dict(cumulative_pe321=cumulative.tolist(),
                        planned_pose_m=target.tolist(), local_send_success=True,
                        execution_confirmed=False)

    def _send_sdk_tool_poses(self, targets, position_limit, rotation_limit):
        adapters = self._sdk_tool_adapters
        if adapters is None:
            raise RuntimeError('SDK tool adapters are not initialized')
        if not isinstance(targets, Mapping) or set(targets) != set(self.arm_names):
            raise ValueError('SDK adapter requires the complete arm group')
        candidates = {name: adapters[name].propose(
            targets[name], max_position_step_m=position_limit,
            max_rotation_step_rad=rotation_limit) for name in self.cartesian_target_order}
        payload = json.dumps(dict(type='321', pe=[candidate[1].tolist()
                             for candidate in candidates.values()]),
                             separators=(',', ':'), allow_nan=False).encode('utf-8')
        try:
            self._send_follower_target(_pack(payload, command=False), 'Cart')
        except BaseException:
            self._sdk_tool_adapters = None
            self._confirmed_active = False
            self._cartesian_raw_used = True  # Block reinitialization until stop/start.
            raise
        for name, (pose, cumulative) in candidates.items():
            adapters[name].commit(pose, cumulative)
        return dict(arms={name: dict(planned_pose_m=pose.tolist(),
                                    cumulative_pe321=cumulative.tolist())
                          for name, (pose, cumulative) in candidates.items()},
                    local_send_success=True, execution_confirmed=False)

    def jog_tool(self, arm_deltas):
        """Bounded dual-arm console move in each arm's command-start tool axes.

        Deltas are [dx,dy,dz,rx,ry,rz] (metres / rotation-vector radians).
        Requires an explicitly started follower. Unselected arms hold their
        previous planned pose. No automatic enable, restart, or error integral.
        GET and UDP are serialized for this low-speed diagnostic operation.
        """
        if not self.is_multi_arm:
            raise ValueError('jog_tool requires a dual-arm configuration')
        if (not isinstance(arm_deltas, Mapping) or not arm_deltas
                or not set(arm_deltas).issubset(self.arm_names)):
            raise ValueError(f'Expected selected arms from {self.arm_names}')
        deltas = {name: _vector(delta, 6, 'tool delta') for name, delta in arm_deltas.items()}
        if any(np.linalg.norm(d[:3]) > .01 or np.linalg.norm(d[3:]) > math.radians(5)
               for d in deltas.values()):
            raise ValueError('Each jog is limited to 10 mm translation / 5 degrees rotation')
        self._require_motion()
        if not self._started_here or not self._start_confirmed or self._follower_mode != 'Cart':
            raise RuntimeError('Start this console\'s Cartesian follower before jogging')
        if not self._trajectory_lock.acquire(blocking=False):
            raise RuntimeError('A trajectory already owns the target stream')
        self._trajectory_owner = threading.get_ident()
        self._traj_stop_event.clear()
        try:
            state, actual = self._jog_feedback()
            if self._sdk_tool_adapters is None:
                self.begin_sdk_tool_trajectory(actual)
            starts = {name: adapter.previous_pose.copy()
                      for name, adapter in self._sdk_tool_adapters.items()}
            # Never rebase the cumulative stream to delayed feedback between commands.
            if not self._jog_pose_matches(actual, starts, .002, .03):
                raise RuntimeError('Previous jog has not reached its target; refusing a new jog')
            goals = {name: pose.copy() for name, pose in starts.items()}
            duration = 1.
            for name, delta in deltas.items():
                rotation = Rotation.from_quat(starts[name][3:])
                goals[name][:3] += rotation.apply(delta[:3])
                goals[name][3:] = (rotation * Rotation.from_rotvec(delta[3:])).as_quat()
                duration = max(duration, 2 * np.linalg.norm(delta[:3]) / .005,
                               2 * np.linalg.norm(delta[3:]) / .05)
            count = math.ceil(duration * 30)
            previous = starts
            for index in range(count + 1):
                tick = time.monotonic()
                _, actual = self._jog_feedback()
                if not self._jog_pose_matches(actual, previous, .02, .15):
                    raise RuntimeError('Jog tracking error exceeds 20 mm / 0.15 rad')
                u = index / count
                blend = u**3 * (10 - 15*u + 6*u*u)
                targets = {name: pose.copy() for name, pose in starts.items()}
                for name, delta in deltas.items():
                    targets[name][:3] += blend * (goals[name][:3] - starts[name][:3])
                    targets[name][3:] = (Rotation.from_quat(starts[name][3:]) *
                                        Rotation.from_rotvec(blend * delta[3:])).as_quat()
                self.send_sdk_tool_pose(targets, max_position_step_m=.001,
                                        max_rotation_step_rad=.01)
                previous = targets
                # Slow feedback stretches the move; never send a catch-up burst.
                time.sleep(max(0., 1/30 - (time.monotonic() - tick)))
            deadline, settled = time.monotonic() + 5., 0
            while time.monotonic() < deadline:
                state, actual = self._jog_feedback()
                if not self._jog_pose_matches(actual, goals, .02, .15):
                    raise RuntimeError('Jog tracking error exceeds 20 mm / 0.15 rad')
                settled = settled + 1 if self._jog_pose_matches(actual, goals, .001, .02) else 0
                # Repeat the final cumulative value, never accumulate the jog twice.
                self.send_sdk_tool_pose(goals, max_position_step_m=.001,
                                        max_rotation_step_rad=.01)
                if settled >= 3:
                    return state
                time.sleep(.05)
            raise TimeoutError('Jog arrival not confirmed within 5 s (1 mm / 0.02 rad)')
        except BaseException:
            if self.follower_stop_pending:
                try:
                    self.stop_follower()
                except BaseException as stop_error:
                    warnings.warn(f'Jog stop unconfirmed: {stop_error}; check the controller', RuntimeWarning)
            raise
        finally:
            self._trajectory_owner = None
            self._trajectory_lock.release()

    def _jog_feedback(self):
        self._require_target_owner()
        state = self.get_state()
        context = state.raw['ret_context']
        if (state.has_error or not self._confirmed_active or state.follower_mode != 'Cart'
                or context['motion_msg'].get('jog_coordinate') != 'Tool'
                or context.get('robot_msg', {}).get('activate') != 'Enabled'):
            raise RuntimeError('Jog requires healthy, Enabled, active Tool follower feedback')
        poses = {name: _pose(arm.tcp_pq_m) for name, arm in state.arms.items()}
        if set(poses) != set(self.arm_names):
            raise A10ProtocolError('Jog requires actual poses for the complete arm group')
        return state, poses

    @staticmethod
    def _jog_pose_matches(actual, targets, position_tolerance, rotation_tolerance):
        return all(np.linalg.norm(actual[name][:3] - target[:3]) <= position_tolerance
                   and (Rotation.from_quat(actual[name][3:]).inv() *
                        Rotation.from_quat(target[3:])).magnitude() <= rotation_tolerance
                   for name, target in targets.items())

    def send_eepose(self, arm_targets):
        """Accept metres/xyzw; optionally encode signed, continuous Euler 321.

        KAANH 2.5.7's pq receiver maps negative yaw/roll to near 2*pi before
        scalar filtering/planning. The 321/pe receiver preserves signed angles.
        """
        if not isinstance(arm_targets, Mapping) or set(arm_targets) != set(self.arm_names):
            raise ValueError(f'Expected the complete arm group: {self.cartesian_target_order}')
        poses = {name: _pose(arm_targets[name]) for name in self.cartesian_target_order}
        with self._io_lock:
            if self._sdk_tool_adapter is not None or self._sdk_tool_adapters is not None:
                raise RuntimeError("Cannot mix legacy packets with SDK tool adapter")
            eulers = {}
            data = {'type': 'pq', 'pq': [value.tolist() for value in poses.values()]}
            if self.cartesian_wire_format == 'pe321':
                rows = []
                for name, value in poses.items():
                    with warnings.catch_warnings():
                        warnings.filterwarnings('error', message='Gimbal lock detected.*', category=UserWarning)
                        euler = Rotation.from_quat(value[3:]).as_euler('ZYX')
                    previous = self._cartesian_eulers[name]
                    delta = (euler - previous + np.pi) % (2 * np.pi) - np.pi
                    if np.max(np.abs(delta)) > np.pi / 2:
                        raise ValueError(f'Cartesian Euler branch jump exceeds pi/2 for {name}; refusing target')
                    eulers[name] = previous + delta
                    rows.append(np.r_[value[:3], eulers[name]].tolist())
                data = {'type': '321', 'pe': rows}
            payload = json.dumps(data, separators=(',', ':'), allow_nan=False).encode('utf-8')
            self._cartesian_raw_used = True  # Includes uncertain sends; no mid-stream switch.
            self._send_follower_target(_pack(payload, command=False), 'Cart')
            if eulers:
                self._cartesian_eulers.update(eulers)
                if not self.is_multi_arm:
                    self._cartesian_euler = eulers[self.arm_name].copy()

    def _send_follower_target(self, packet, mode):
        with self._io_lock:
            self._require_target_owner()
            self._require_motion()
            if (not self._started_here or not self._confirmed_active
                    or self._follower_mode != mode):
                raise RuntimeError(f'{mode} follower start has not been confirmed')
            udp = self._connection.udp if mode == 'Cart' else self._connection.joint_udp
            try:
                if mode == 'Axis':
                    self._check_joint_follower_reply()
                if udp.send(packet) != len(packet):
                    raise ConnectionError('Incomplete UDP datagram send')
            except OSError:
                self._confirmed_active = False
                raise
        # UDP has no application ACK. This return does not mean execution.

    def _check_joint_follower_reply(self):
        """Surface direct SDK rejection datagrams; silence is not acceptance.

        FollowerAjParam replies on a dimension mismatch, but sends no success
        ACK. Read only already available data, without delaying the stream.
        Shared transport uses a proxy socket and is not inspected here.
        """
        if self.shared_control or self._connection.joint_udp is None:
            return
        udp = self._connection.joint_udp
        if select.select([udp], [], [], 0)[0]:
            packet = udp.recv(65535)
            self._confirmed_active = False
            detail = _unpack(packet).decode('utf-8', errors='replace')[:500]
            raise A10ProtocolError(f'关节 follower UDP 返回：{detail}；已停止继续下发目标')

    def send_gripper(self, gripper_targets, wait=False):
        """Explicit USB or controller route; only USB can wait for actual arrival."""
        self._require_single_arm('gripper commands')
        if not isinstance(gripper_targets, Mapping) or set(gripper_targets) != {self.arm_name}:
            raise ValueError(f'Expected exactly one gripper: {self.arm_name}')
        position = float(_vector([gripper_targets[self.arm_name]], 1, 'gripper mm')[0])
        if not 0 <= position <= self.gripper_max_width_mm:
            raise ValueError(f'Gripper target must be within 0..{self.gripper_max_width_mm} mm')
        if wait and self.gripper_transport != 'usb':
            raise NotImplementedError('Controller gripper arrival feedback is not verified')
        return self._send_gripper_target(position, wait=wait)

    def _gripper_call(self, operation):
        if self._gripper_timed_out:
            return None
        try:
            return operation()
        except GripperError as exc:
            from serial import SerialTimeoutException
            timeout = isinstance(exc, GripperTimeoutError) or isinstance(
                exc.__cause__, SerialTimeoutException)
            if (not self.continue_on_gripper_timeout or not timeout
                    or self.gripper_operator.last_state is None):
                raise
            self._gripper_timed_out = True
            logging.getLogger(__name__).warning(
                '夹爪超时，继续推理：沿用最后实测开度，本次会话暂停夹爪通信。%s', exc)
            return None

    def _read_gripper_state(self, **kwargs):
        state = self._gripper_call(lambda: self.gripper_operator.read_state(**kwargs))
        return self.gripper_operator.last_state if state is None else state

    def _send_gripper_target(self, position, *, wait=False, check_state=True):
        self._require_single_arm('gripper commands')
        with self._io_lock:
            self._require_target_owner()
            self._require_motion()
            if check_state and self.get_state().has_error:
                raise A10ControllerError('Controller reports a fault')
            self._require_motion()
            if self.gripper_operator is not None:
                return self._gripper_call(
                    lambda: self.gripper_operator.set_position_mm(position, wait=wait))
            return self._connection.request(f'g_move --pos={position:.12g}')

    def _require_target_owner(self):
        if self._trajectory_owner is not None:
            if self._trajectory_owner != threading.get_ident():
                raise RuntimeError('A trajectory already owns the target stream')
            if self._traj_stop_event.is_set():
                raise InterruptedError('A10 trajectory was cancelled')

    @staticmethod
    def _feedback_trace(state, include_raw=False):
        if state is None:
            return None
        result = dict(received_monotonic_s=state.received_monotonic_s,
                      age_s=time.monotonic() - state.received_monotonic_s,
                      actual_joints_rad=(None if state.actual_joints_rad is None
                                         else state.actual_joints_rad.tolist()),
                      gripper_mm=state.gripper_mm, follower_active=state.follower_active,
                      follower_mode=state.follower_mode, has_error=state.has_error)
        if include_raw:
            result['raw'] = state.raw
        return result

    def _check_trajectory_feedback(self, timeout_s, *, check_gripper):
        state = (self._recent_state() if self.status_poll_interval_s is not None
                 else self.get_state(timeout_s=timeout_s))
        self._last_trajectory_feedback = state
        if self.record_trajectory_trace:
            self._trajectory_feedback_trace.append(self._feedback_trace(state, include_raw=True))
        if state.has_error:
            raise A10ControllerError('Controller fault during trajectory')
        if (not self._started_here or not self._start_confirmed
                or state.follower_active is not True or state.follower_mode != 'Axis'
                or state.raw['ret_context'].get('robot_msg', {}).get('activate') != 'Enabled'
                or not self._confirmed_active or self._follower_mode != 'Axis'):
            raise RuntimeError('Joint follower permission lost during trajectory')
        if any(state.arms[name].actual_joints_rad is None for name in self.arm_names):
            raise A10ProtocolError('Trajectory requires actual joint feedback')
        if check_gripper:
            self._read_gripper_state(timeout=timeout_s)

    def execute_trajectory(self, arm_trajectories, gripper_trajectories=None,
                           head_trajectory=None, dt=0.1, async_exec=False,
                           joint_target_repeat=1):
        """Send absolute joint rad / gripper mm rows on a monotonic clock.

        Synchronous only. Validate the WHOLE chunk before any target. First
        send is immediate after bounded preflight checks. Feedback queries
        pause the schedule; sending takes at least T*dt, plus feedback time.
        Return means sending ended, not physical arrival or UDP receipt.
        Periodic status/servo checks interrupt a faulted stream. Overdue slots
        abort, with no retry, interpolation, catch-up burst or ROS dependency.
        Repeat each joint target at dt/joint_target_repeat intervals. Model
        target duration remains dt; USB gripper is sent only on the first copy.
        Caller owns follower start/stop (the runner stops in its finally).
        """
        if async_exec is not False:
            raise NotImplementedError('A10 trajectories support async_exec=False only')
        if head_trajectory is not None:
            raise ValueError('A10 has no head trajectory')
        if isinstance(dt, bool) or not math.isfinite(dt) or dt <= 0:
            raise ValueError('dt must be positive and finite')
        if type(joint_target_repeat) is not int or joint_target_repeat <= 0:
            raise ValueError('joint_target_repeat must be a positive integer')
        send_dt = dt / joint_target_repeat
        if not math.isfinite(send_dt) or send_dt <= 0:
            raise ValueError('joint send interval must be positive and finite')
        if not isinstance(arm_trajectories, Mapping) or set(arm_trajectories) != set(self.arm_names):
            raise ValueError(f'Expected exactly these arms: {self.arm_names}')
        joints = {}
        for name in self.arm_names:
            raw = np.asarray(arm_trajectories[name])
            if raw.ndim != 2 or raw.shape[1] != self.joint_dofs[name] or not len(raw) or raw.dtype.kind not in 'iuf':
                raise ValueError(f'{name} trajectory must be numeric nonempty [T,{self.joint_dofs[name]}] radians')
            joints[name] = np.array(raw, dtype=np.float64, copy=True)
            with np.errstate(over='ignore'):
                if not np.isfinite(joints[name]).all() or not np.isfinite(np.rad2deg(joints[name])).all():
                    raise ValueError('Arm trajectory must remain finite after degree conversion')
        row_count = len(joints[self.arm_names[0]])
        if any(len(q) != row_count for q in joints.values()):
            raise ValueError('All arm trajectories must have the same number of rows')
        if not math.isfinite(row_count * dt):
            raise ValueError('Trajectory duration must be finite')
        widths = None
        if gripper_trajectories is not None and not (isinstance(gripper_trajectories, Mapping) and not gripper_trajectories):
            self._require_single_arm('timed gripper trajectories')
            if (not isinstance(gripper_trajectories, Mapping)
                    or set(gripper_trajectories) != {self.arm_name}):
                raise ValueError(f'Expected exactly one gripper: {self.arm_name}')
            raw = np.asarray(gripper_trajectories[self.arm_name])
            if raw.shape != (row_count,) or raw.dtype.kind not in 'iuf':
                raise ValueError('Gripper trajectory must be numeric [T] matching arm length')
            widths = np.array(raw, dtype=np.float64, copy=True)
            if (not np.isfinite(widths).all() or np.any(widths < 0)
                    or np.any(widths > self.gripper_max_width_mm)):
                raise ValueError('Gripper trajectory exceeds the configured working range')
            if self.gripper_transport != 'usb':
                raise NotImplementedError('Timed gripper trajectories require the USB driver')
        lateness = send_dt * .5 if self.trajectory_max_lateness_s is None else self.trajectory_max_lateness_s
        if lateness >= send_dt:
            raise ValueError('trajectory_max_lateness_s must be smaller than the joint send interval')
        if not self._trajectory_lock.acquire(blocking=False):
            raise RuntimeError('A trajectory is already running')
        sent = 0
        joint_packets_sent = 0
        gripper_targets_sent = 0
        started = None
        wall_started = None
        completed = False
        phase = 'preflight'
        trace = []
        self._trajectory_feedback_trace = []
        failure = None
        try:
            with self._io_lock:
                self._require_motion()
                if (not self._started_here or not self._start_confirmed
                        or not self._confirmed_active or self._follower_mode != 'Axis'):
                    raise RuntimeError('Confirmed owned joint follower required for trajectory')
                self._trajectory_owner = threading.get_ident()
                self._traj_stop_event.clear()
                self._require_target_owner()
                feedback_timeout = self.trajectory_feedback_timeout_s
                # Between chunks no send deadline is running. Use the normal
                # command budget, including after a long model inference.
                self._check_trajectory_feedback(self.timeout_s, check_gripper=widths is not None)
            started = last_feedback = time.monotonic()
            wall_started = started
            phase = 'sending'
            for slot in range(row_count * joint_target_repeat):
                index, repeat_index = divmod(slot, joint_target_repeat)
                target = {name: q[index] for name, q in joints.items()}
                deadline = started + slot * send_dt
                self._traj_stop_event.wait(max(0., deadline - time.monotonic()))
                with self._io_lock:
                    self._require_target_owner()
                    now = time.monotonic()
                    if now - deadline > lateness:
                        raise TimeoutError(f'Trajectory slot {slot} missed; no catch-up sends')
                    if now - last_feedback >= self.trajectory_feedback_interval_s:
                        feedback_started = time.monotonic()
                        self._check_trajectory_feedback(feedback_timeout, check_gripper=widths is not None)
                        last_feedback = time.monotonic()
                        # WS/USB feedback is not a real-time operation. Hold the
                        # last target while checking, then resume the schedule
                        # without skipping or bursting overdue model rows.
                        started += last_feedback - feedback_started
                        deadline = started + slot * send_dt
                        self._require_target_owner()
                    if time.monotonic() - deadline > lateness:
                        raise TimeoutError(f'Trajectory slot {slot} missed; no catch-up sends')
                    row = None
                    if self.record_trajectory_trace:
                        state = (self._latest_state if self.status_poll_interval_s is not None
                                 else getattr(self, '_last_trajectory_feedback', None))
                        row = dict(slot=slot, action_index=index, repeat_index=repeat_index,
                                   scheduled_monotonic_s=deadline,
                                   target_joints_rad=({name: q.tolist() for name, q in target.items()}
                                                     if self.is_multi_arm else target[self.arm_name].tolist()),
                                   target_gripper_mm=None if widths is None else float(widths[index]),
                                   joint_sent=False, gripper_sent=False,
                                   feedback=self._feedback_trace(state))
                        trace.append(row)
                    send_started = time.monotonic()
                    if row is not None:
                        row['joint_send_started_s'] = send_started
                        row['lateness_s'] = send_started - deadline
                        row['joint_send_gap_s'] = (None if self._last_joint_send_monotonic_s is None
                                                  else send_started - self._last_joint_send_monotonic_s)
                    self.send_joints(target)
                    joint_packets_sent += 1
                    self._last_joint_send_monotonic_s = send_started
                    if row is not None:
                        row['joint_sent'] = True
                        row['joint_send_finished_s'] = time.monotonic()
                    if widths is not None and repeat_index == 0:
                        grip_started = time.monotonic()
                        self._send_gripper_target(float(widths[index]), check_state=False)
                        if self._gripper_timed_out:
                            # Resume after the failed USB transaction without a
                            # catch-up burst or a secondary slot-budget failure.
                            delay = time.monotonic() - grip_started
                            started += delay
                            deadline += delay
                        else:
                            gripper_targets_sent += 1
                        if row is not None:
                            row['gripper_sent'] = not self._gripper_timed_out
                            row['gripper_send_finished_s'] = time.monotonic()
                    if repeat_index == joint_target_repeat - 1:
                        sent += 1
                    if time.monotonic() > deadline + send_dt:
                        raise TimeoutError(f'Trajectory slot {slot} exceeded its time budget')
            self._traj_stop_event.wait(max(0., started + row_count * dt - time.monotonic()))
            with self._io_lock:
                self._require_target_owner()
                # A final check also catches faults in shorter-than-monitor chunks.
                phase = 'final_feedback'
                self._check_trajectory_feedback(self.timeout_s, check_gripper=widths is not None)
            completed = True
        except BaseException as exc:
            failure = repr(exc)
            with self._io_lock:
                self._confirmed_active = self._start_confirmed = False
                self._traj_stop_event.set()
            raise
        finally:
            with self._io_lock:
                self.last_trajectory = dict(selected=row_count, sent=sent,
                    joint_packets_sent=joint_packets_sent, gripper_targets_sent=gripper_targets_sent,
                    joint_target_repeat=joint_target_repeat, joint_send_dt_s=send_dt,
                    joint_publish_hz=1. / send_dt, model_action_dt_s=dt,
                    completed=completed, phase=phase, physical_arrival_confirmed=False,
                    elapsed_s=None if wall_started is None else time.monotonic() - wall_started,
                    error=failure, trace=trace, feedback_checks=self._trajectory_feedback_trace)
                self._trajectory_owner = None
            self._trajectory_lock.release()

    def stop_trajectory(self):
        """Cancel pending sends and explicitly stop this session's follower."""
        self._traj_stop_event.set()
        if self.follower_stop_pending:
            return self.stop_follower()
        return None

    def is_trajectory_running(self):
        """Whether this operator is currently scheduling a chunk, not follower mode."""
        return self._trajectory_owner is not None

    def send_joints(self, arm_targets):
        """Send a complete joint group via follower UDP (wire degrees).

        Flatten arms in joint_target_order, with six joints per A10 arm.
        No gripper, implicit hold, interpolation, retry, or arrival ACK.
        """
        if not isinstance(arm_targets, Mapping) or set(arm_targets) != set(self.arm_names):
            raise ValueError(f'Expected exactly these arms: {self.arm_names}')
        targets = {name: _vector(arm_targets[name], self.joint_dofs[name],
                                 f'{name} joint targets in radians') for name in self.arm_names}
        radians = np.concatenate([targets[name] for name in self.joint_target_order])
        with np.errstate(over='ignore'):
            degrees = np.rad2deg(radians)
        if not np.isfinite(degrees).all():
            raise A10ProtocolError('Joint conversion to degrees must remain finite')
        values = ','.join(format(float(v), '.17g') for v in degrees)
        payload = f'jp --motor_pos=[{values}]'.encode('utf-8')
        self._send_follower_target(_pack(payload, command=False), 'Axis')

    def _claim_init_motion(self):
        with self._io_lock:
            if not self._trajectory_lock.acquire(blocking=False):
                raise RuntimeError('A trajectory is already running')
            self._trajectory_owner = threading.get_ident()

    def _release_init_motion(self):
        with self._io_lock:
            self._trajectory_owner = None
        try:
            if self._init_shared_reserved:
                self._init_shared_reserved = False
                self.end_shared_console_operation()
        finally:
            self._trajectory_lock.release()

    def _prepare_init_motion(self):
        with self._io_lock:
            self._require_motion()
            state = self.get_state()
            if state.follower_active not in (True, False):
                raise RuntimeError('Initialization requires known follower state')
            snapshot = dict(follower_active=state.follower_active,
                            follower_mode=state.follower_mode,
                            jog_coordinate=state.raw['ret_context']['motion_msg'].get('jog_coordinate'))
            if state.follower_active and state.follower_mode not in ('Axis', 'Cart'):
                raise RuntimeError('Cannot restore an unknown follower mode')
            # Reserve only after capturing the original mode/activation, so
            # console m_init retains its existing restoration semantics.
            if self.shared_control and self.shared_control.get('role') == 'console':
                self.begin_shared_console_operation()
                self._init_shared_reserved = True
            self._prepare_shared_console_command()
            if state.follower_active and not self.shared_control and not self._started_here:
                # Explicit m_init temporarily borrows the controller's existing follower.
                # Stop it before setting speed or entering a different plan.
                self._started_here = True
                self._follower_mode = state.follower_mode
            if self.follower_stop_pending:
                self.stop_follower()
            self._check_init_speed(self.get_state())
            return snapshot

    def _restore_init_control_state(self, snapshot):
        """Restore mode/activation after arrival, without replaying any old target."""
        with self._io_lock:
            self._require_motion()
            state = self.get_state()
            coordinate = snapshot['jog_coordinate']
            mode = snapshot['follower_mode']
            select_plan = mode in ('Axis', 'Cart') and (
                snapshot['follower_active'] or state.follower_mode != mode)
            plan_coordinate = coordinate
            if select_plan and mode == 'Cart' and coordinate == 'Axis' and not snapshot['follower_active']:
                plan_coordinate = 'Tool'  # Temporarily select Cart, then restore inactive Axis coordinates.
            if (plan_coordinate is not None and
                    state.raw['ret_context']['motion_msg'].get('jog_coordinate') != plan_coordinate):
                self.set_jog_coordinate(plan_coordinate)
            if select_plan:
                # Even an inactive controller retains its last Axis/Cart selection.
                # Select the original plan without sending a UDP target, then
                # stop it again if the follower was originally inactive.
                self._traj_stop_event.clear()
                self.start_follower('joint' if mode == 'Axis' else 'cartesian')
                if not snapshot['follower_active']:
                    self.stop_follower()
            self._require_motion()
            state = self.get_state()
            if (not snapshot['follower_active'] and coordinate is not None and
                    state.raw['ret_context']['motion_msg'].get('jog_coordinate') != coordinate):
                self.set_jog_coordinate(coordinate)
                state = self.get_state()
            if (state.follower_active != snapshot['follower_active']
                    or (mode in ('Axis', 'Cart') and state.follower_mode != mode)
                    or (coordinate is not None and
                        state.raw['ret_context']['motion_msg'].get('jog_coordinate') != coordinate)):
                raise RuntimeError('Original follower state or coordinate was not restored')
            # Cartesian encoding must be anchored anew at the post-init actual pose.
            self._reset_cartesian_encoding()
            return state

    @staticmethod
    def _check_init_speed(state):
        context = state.raw['ret_context']
        mode = context.get('robot_msg', {}).get('op_mode')
        percent = (context.get('motion_msg', {}).get('jog_vel') if mode == 'Manual'
                   else context.get('program_msg', {}).get('vel') if mode == 'Auto' else None)
        if type(percent) in (int, float) and percent == 0:
            raise RuntimeError(f'控制器 {mode} 速度倍率为 0%，无法回位；请在控制器设置合适速度后重试')

    def _read_init_joints(self):
        self._check_joint_follower_reply()
        state = self.get_state()
        self._check_init_speed(state)
        if state.has_error or state.raw['ret_context'].get('robot_msg', {}).get('activate') != 'Enabled':
            raise A10ControllerError('Initialization requires healthy Enabled feedback')
        if self._started_here and not self._confirmed_active:
            raise A10ControllerError('Initialization joint follower is no longer active')
        return {name: arm.actual_joints_rad for name, arm in state.arms.items()}

    def gohome(self):
        raise NotImplementedError('No verified home command for this follower adapter')
