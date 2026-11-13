"""Offline Cartesian reference generation; no sockets, SDK, or motor commands.

Anchor composition and the speed-only governor follow A10_new 97feb31,
``a10_anchor_protocol.cpp``. Velocity clamp/slew follow ``a10_vr_vel_plan.cpp``.
The optional acceleration stage is a frontend reference experiment, not the
controller's 500 Hz feedback/IK loop. Hardware IO stays in vr.runtime (stage 3).
All poses: base-frame xyz metres + quaternion xyzw; offsets: tool xyz + rotvec.
"""

from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial.transform import Rotation


def vector(value, size, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f'{name} must contain {size} finite numbers')
    return result.copy()


def pose(value):
    result = vector(value, 7, 'pose')
    if not np.isclose(np.linalg.norm(result[3:]), 1., atol=1e-6):
        raise ValueError('pose quaternion must be unit xyzw')
    return result


def calibrate_reference_pose(hand_poses, robot_poses, hand_to_tool_quat_xyzw):
    """Fit a fixed base/VR rotation from stationary, paired metre/xyzw poses.

    The existing local-axis convention supplies R_hand_tool. Even samples fit
    R_base_vr; odd samples check reconstruction. This checks consistency, not
    whether the operator chose the intended physical forward/up directions.
    """
    if len(hand_poses) != len(robot_poses) or len(hand_poses) < 10:
        raise ValueError('标定至少需要 10 对有效手柄和机械臂姿态')
    hands = np.array([pose(value) for value in hand_poses])
    robots = np.array([pose(value) for value in robot_poses])
    tool_quat = vector(hand_to_tool_quat_xyzw, 4, 'hand_to_tool_quat_xyzw')
    tool = Rotation.from_quat(pose(np.r_[np.zeros(3), tool_quat])[3:])
    hand_rotation, robot_rotation = Rotation.from_quat(hands[:, 3:]), Rotation.from_quat(robots[:, 3:])
    quality = {}
    for label, values, rotations, max_position, max_angle in (
            ('hand', hands, hand_rotation, .015, math.radians(3.)),
            ('robot', robots, robot_rotation, .002, math.radians(1.))):
        distance = float(np.linalg.norm(values[:, :3] - values[0, :3], axis=1).max())
        angle = float((rotations[0].inv() * rotations).magnitude().max())
        quality[f'max_{label}_position_change_m'] = distance
        quality[f'max_{label}_rotation_change_rad'] = angle
        if distance > max_position or angle > max_angle:
            raise ValueError(f'{label} 标定期间移动或转动过大，请保持静止后重试')
    candidates = robot_rotation * tool.inv() * hand_rotation.inv()
    base_from_vr = candidates[::2].mean()
    validation_error = ((base_from_vr * hand_rotation[1::2] * tool).inv()
                        * robot_rotation[1::2]).magnitude()
    quality.update(sample_count=len(hands), validation_samples=len(validation_error),
                   max_validation_rotation_error_rad=float(validation_error.max()))
    if validation_error.max() > math.radians(3.):
        raise ValueError('留出采样的姿态重建误差过大，请保持静止后重试')
    calibration = dict(vr_to_robot_quat_xyzw=base_from_vr.as_quat().tolist(),
                       hand_to_tool_quat_xyzw=tool.as_quat().tolist())
    return dict(pose_calibration=calibration, quality=quality,
                reference_hand_pose_vr=np.r_[hands[:, :3].mean(axis=0), hand_rotation.mean().as_quat()].tolist(),
                reference_robot_pose_m=np.r_[robots[:, :3].mean(axis=0), robot_rotation.mean().as_quat()].tolist())


def compose_offset(anchor, offset):
    """T_target = T_robot_anchor * T_offset, matching EeAnchorShadow."""
    anchor, offset = pose(anchor), vector(offset, 6, 'offset')
    rotation = Rotation.from_quat(anchor[3:])
    return np.r_[anchor[:3] + rotation.apply(offset[:3]),
                 (rotation * Rotation.from_rotvec(offset[3:])).as_quat()]


def follower_relative(startup_anchor, target):
    """Fixed follower anchor is independent of each VR clutch/re-anchor."""
    anchor, target = pose(startup_anchor), pose(target)
    inverse = Rotation.from_quat(anchor[3:]).inv()
    return np.r_[inverse.apply(target[:3] - anchor[:3]),
                 (inverse * Rotation.from_quat(target[3:])).as_quat()]


def clamp_vector(value, maximum):
    value = vector(value, 3, 'velocity')
    if not math.isfinite(maximum) or maximum < 0:
        raise ValueError('maximum must be finite and nonnegative')
    norm = np.linalg.norm(value)
    return value * (maximum / norm) if norm > maximum else value


def slew_vector(current, target, maximum_delta):
    """C++ slew_vec3 limits EACH component, not the vector norm."""
    current, target = vector(current, 3, 'current'), vector(target, 3, 'target')
    if not math.isfinite(maximum_delta) or maximum_delta < 0:
        raise ValueError('maximum_delta must be finite and nonnegative')
    return current + np.clip(target - current, -maximum_delta, maximum_delta)


@dataclass(frozen=True)
class ReferenceConfig:
    max_linear_speed_m_s: float = .12
    max_angular_speed_rad_s: float = .32
    max_tracking_error_m: float = .05
    max_tracking_error_rad: float = .35
    # False keeps measured errors as diagnostics; motion/time limits still apply.
    enforce_tracking_error_limits: bool = True
    # C++ default 50 * 2 ms. Use elapsed time when frontend rate changes.
    tracking_fault_after_s: float = .1
    max_input_age_s: float = .25
    max_feedback_age_s: float = .25
    max_dt_s: float = .1
    limit_acceleration: bool = True
    max_linear_accel_m_s2: float = .4
    max_angular_accel_rad_s2: float = .96
    position_gain: float = 6.
    rotation_gain: float = 4.8
    # Incremental mode follows current hand motion and drops angular catch-up.
    rotation_follow_mode: str = 'anchor'
    rotation_velocity_deadzone_rad_s: float = .03
    max_angular_brake_rad_s2: float | None = None
    # None preserves the original C++ parity path. Config opts into soft lead limits.
    max_reference_lead_m: float | None = None
    max_reference_lead_rad: float | None = None
    # Retained for configuration compatibility; dual release uses stability plus
    # the hard tracking caps, not precision arrival at the frontend endpoint.
    release_position_tolerance_m: float = .002
    release_rotation_tolerance_rad: float = .02
    release_settle_s: float = .1
    release_timeout_s: float = 2.

    def __post_init__(self):
        if self.rotation_follow_mode not in ('anchor', 'incremental_velocity'):
            raise ValueError('rotation_follow_mode must be anchor or incremental_velocity')
        if self.rotation_follow_mode == 'incremental_velocity' and not self.limit_acceleration:
            raise ValueError('incremental_velocity requires limit_acceleration=True')
        for name, value in vars(self).items():
            if name == 'rotation_follow_mode':
                continue
            if name == 'max_angular_brake_rad_s2' and value is None:
                continue
            if name == 'rotation_velocity_deadzone_rad_s':
                if isinstance(value, bool) or not math.isfinite(value) or value < 0:
                    raise ValueError(f'{name} must be finite and nonnegative')
                continue
            if name in ('max_reference_lead_m', 'max_reference_lead_rad') and value is None:
                continue
            if name in ('limit_acceleration', 'enforce_tracking_error_limits'):
                if type(value) is not bool:
                    raise ValueError(f'{name} must be bool')
            elif isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be positive and finite')
        for lead, hard in ((self.max_reference_lead_m, self.max_tracking_error_m),
                           (self.max_reference_lead_rad, self.max_tracking_error_rad)):
            if self.enforce_tracking_error_limits and lead is not None and lead >= hard:
                raise ValueError('Reference lead limit must be smaller than the tracking fault limit')


class AnchorState:
    """Validated EeAnchorShadow equivalent. No accumulation of duplicate offsets."""

    def __init__(self):
        self.active = False
        self.key = None
        self.sequence = -1
        self.robot_anchor = self.target = None

    def update(self, command, actual):
        actual = pose(actual)
        if not isinstance(command, dict) or type(command.get('active')) is not bool:
            raise ValueError('anchor command requires boolean active')
        session = command.get('session_id')
        if not isinstance(session, str) or not session:
            raise ValueError('session_id must be a nonempty string')
        for field in ('anchor_id', 'sample_sequence'):
            if type(command.get(field)) is not int or command[field] < 0:
                raise ValueError(f'{field} must be a nonnegative integer')
        offset = vector(command.get('offset'), 6, 'offset')
        gripper = command.get('gripper')
        if isinstance(gripper, bool) or not isinstance(gripper, (float, int)) or not math.isfinite(gripper) or abs(gripper) > 1:
            raise ValueError('anchor gripper must be finite and within [-1, 1]')
        key = (session, command['anchor_id'])
        sequence = command['sample_sequence']
        # Match the source shadow's inactive path. VRPoseController adds a
        # stricter ordering check before using commands to drive its reference.
        if not command['active']:
            self.active, self.key, self.sequence = False, key, sequence
            return 'inactive'
        new_session = not self.active or self.key[0] != session
        if not new_session and key[1] < self.key[1]:
            return 'stale'
        new_anchor = new_session or key[1] > self.key[1]
        if not new_anchor and sequence <= self.sequence:
            return 'duplicate' if sequence == self.sequence else 'stale'
        target = self._make_target(command, actual, offset, new_anchor)
        if new_anchor:
            self.robot_anchor = actual.copy()
        self.active, self.key, self.sequence = True, key, sequence
        self.target = target
        return 'anchored' if new_anchor else 'updated'

    def _make_target(self, command, actual, offset, new_anchor):
        return compose_offset(actual if new_anchor else self.robot_anchor, offset)


class CalibratedAnchorState(AnchorState):
    """Fixed-frame translation and calibrated absolute hand orientation.

    R_base_tool = R_base_vr * R_vr_hand * R_hand_tool. The second
    calibration expresses tool axes in hand coordinates, not its inverse.
    Only translation anchors/scale are captured again on clutch engagement.
    Input hand poses must have passed the mapper's freshness/spike guards.
    """

    def __init__(self, calibration):
        super().__init__()
        required = {'vr_to_robot_quat_xyzw', 'hand_to_tool_quat_xyzw'}
        if not isinstance(calibration, dict) or set(calibration) != required:
            raise ValueError('pose_calibration requires vr_to_robot_quat_xyzw and hand_to_tool_quat_xyzw')
        if any(calibration[name] is None for name in required):
            raise ValueError('pose_calibration has missing quaternions; no identity fallback')
        rotations = []
        for name in ('vr_to_robot_quat_xyzw', 'hand_to_tool_quat_xyzw'):
            quat = vector(calibration[name], 4, name)
            if not np.isclose(np.linalg.norm(quat), 1., atol=1e-6):
                raise ValueError(f'{name} must be a unit xyzw quaternion')
            rotations.append(Rotation.from_quat(quat))
        self.base_from_vr, self.hand_from_tool = rotations
        self.hand_anchor = None
        self.position_scale = None

    def _make_target(self, command, actual, offset, new_anchor):
        # Never use raw runtime input: mapper provides the accepted/cached pose.
        hand = pose(command.get('hand_pose_vr'))
        scale = command.get('position_scale')
        if isinstance(scale, bool) or not isinstance(scale, (float, int)) or not math.isfinite(scale) or scale <= 0:
            raise ValueError('position_scale must be positive and finite')
        if new_anchor:
            self.hand_anchor = hand[:3].copy()
            self.position_scale = scale
        robot_anchor = actual if new_anchor else self.robot_anchor
        translation = self.base_from_vr.apply((hand[:3] - self.hand_anchor) * self.position_scale)
        rotation = self.base_from_vr * Rotation.from_quat(hand[3:]) * self.hand_from_tool
        return np.r_[robot_anchor[:3] + translation, rotation.as_quat()]


class ReferenceGovernor:
    """Port of bounded reference + tracking freeze/fault, with time-based dwell.

    This class generates a reference, not a motor command. release/force_fault
    reset a reference to actual; their output must not be treated as a smooth
    braking trajectory. The outer controller withholds UDP targets in these states.
    """

    def __init__(self, config=None):
        self.config = config or ReferenceConfig()
        self.state = 'inactive'
        self.reference = None
        self.frozen_s = 0.
        self.tracking_error_m = self.tracking_error_rad = 0.
        self.position_progress_scale = self.rotation_progress_scale = 1.

    @staticmethod
    def _progress_scale(distance, limit):
        """Taper outward progress in the outer half of the lead budget.

        Search along the proposed continuous step, never snap to feedback.
        If feedback moved outside the budget, inward steps remain available.
        """
        if limit is None:
            return 1.
        start, end = distance(0.), distance(1.)
        if end <= start:
            return 1.
        scale = float(np.clip(2. * (limit - start) / limit, 0., 1.))
        if distance(scale) <= limit:
            return scale
        low, high = 0., scale
        for _ in range(24):
            mid = (low + high) / 2.
            if distance(mid) <= limit:
                low = mid
            else:
                high = mid
        return low

    def engage(self, actual):
        self.reference = pose(actual)
        self.state, self.frozen_s = 'tracking', 0.
        self.tracking_error_m = self.tracking_error_rad = 0.
        self.position_progress_scale = self.rotation_progress_scale = 1.

    def release(self, actual):
        self.engage(actual)
        self.state = 'inactive'

    def force_fault(self, actual):
        self.reference = pose(actual)
        self.state, self.frozen_s = 'fault', 0.

    def step(self, target, actual, dt_s, *, hold_reference=False):
        target, actual = pose(target), pose(actual)
        if not math.isfinite(dt_s) or dt_s <= 0:
            raise ValueError('dt_s must be positive and finite')
        if self.state not in ('tracking', 'frozen'):
            return self.state
        cfg, ref = self.config, self.reference
        self.position_progress_scale = self.rotation_progress_scale = 0.
        rotation = Rotation.from_quat(ref[3:])
        self.tracking_error_m = float(np.linalg.norm(ref[:3] - actual[:3]))
        self.tracking_error_rad = float((rotation.inv() * Rotation.from_quat(actual[3:])).magnitude())
        if cfg.enforce_tracking_error_limits and (
                self.tracking_error_m > cfg.max_tracking_error_m
                or self.tracking_error_rad > cfg.max_tracking_error_rad):
            self.frozen_s += dt_s
            self.state = 'fault' if self.frozen_s + 1e-12 >= cfg.tracking_fault_after_s else 'frozen'
            if self.state == 'fault':
                self.reference = actual.copy()
            return self.state
        self.frozen_s = 0.
        if hold_reference:
            self.state = 'frozen'
            return self.state
        translation = clamp_vector(target[:3] - ref[:3], cfg.max_linear_speed_m_s * dt_s)
        rotvec = (rotation.inv() * Rotation.from_quat(target[3:])).as_rotvec()
        rotvec = clamp_vector(rotvec, cfg.max_angular_speed_rad_s * dt_s)
        self.position_progress_scale = self._progress_scale(
            lambda s: np.linalg.norm(ref[:3] + s * translation - actual[:3]),
            cfg.max_reference_lead_m)
        actual_rotation = Rotation.from_quat(actual[3:])
        self.rotation_progress_scale = self._progress_scale(
            lambda s: (actual_rotation.inv() * rotation * Rotation.from_rotvec(s * rotvec)).magnitude(),
            cfg.max_reference_lead_rad)
        translation *= self.position_progress_scale
        rotvec *= self.rotation_progress_scale
        self.reference = np.r_[ref[:3] + translation,
                               (rotation * Rotation.from_rotvec(rotvec)).as_quat()]
        self.state = 'tracking'
        return self.state


class VRPoseController:
    """Pure frontend generator consuming mapper's _xlevr.anchor_command.

    Additional acceleration limits act on reference velocities, not servo
    feedback. Gains shape command/reference separation, not a motor feedback
    loop. No input low-pass, IK or joint control is copied.
    Fault/release results request a confirmed downstream stop; they do not
    promise physical braking. A fault stays latched until reset/start().
    """

    def __init__(self, config=None, *, pose_mapping='anchor_relative', pose_calibration=None):
        self.config = config or ReferenceConfig()
        if pose_mapping not in ('anchor_relative', 'calibrated_absolute'):
            raise ValueError('pose_mapping must be anchor_relative or calibrated_absolute')
        if pose_calibration is not None and not isinstance(pose_calibration, dict):
            raise ValueError('pose_calibration must be a dictionary')
        if pose_mapping == 'calibrated_absolute' and pose_calibration is None:
            raise ValueError('calibrated_absolute requires explicit pose_calibration')
        if self.config.rotation_follow_mode == 'incremental_velocity' and pose_mapping != 'anchor_relative':
            raise ValueError('incremental_velocity requires anchor_relative pose mapping')
        self.pose_mapping = pose_mapping
        # Own calibration data so caller edits cannot change a running mapping.
        self.pose_calibration = (None if pose_calibration is None else
                                 {key: None if value is None else list(value)
                                  for key, value in pose_calibration.items()})
        self.anchor = self._new_anchor()
        self.governor = ReferenceGovernor(self.config)
        self.startup_anchor = self.command_pose = None
        self.linear_velocity = np.zeros(3)
        self.angular_velocity = np.zeros(3)
        self.input_age_s = math.inf
        self.fault_reason = None
        self._release_elapsed_s = self._release_settled_s = 0.
        self._release_previous_actual = None
        self._release_previous_feedback_time_s = None
        self._reanchor_translation = np.zeros(3)
        self._reanchor_rotation = Rotation.identity()
        self._rotation_previous_target = None
        self._rotation_sample_elapsed_s = 0.
        self.rotation_input_base_rad_s = np.zeros(3)
        self.rotation_backlog_discarded_rad = 0.
        self._command_tracking_limit_exceeded = False

    def _new_anchor(self):
        return (CalibratedAnchorState(self.pose_calibration)
                if self.pose_mapping == 'calibrated_absolute' else AnchorState())

    def validate_command(self, command, actual):
        """Validate using the selected mapping before starting any follower."""
        self._new_anchor().update(command, actual)

    def start(self, startup_anchor):
        """Reset after confirmed follower start (or an explicitly labeled shadow anchor)."""
        self.__init__(self.config, pose_mapping=self.pose_mapping,
                      pose_calibration=self.pose_calibration)
        self.startup_anchor = pose(startup_anchor)
        self.command_pose = self.startup_anchor.copy()

    def _result(self, status, *, stop=False, packet=None):
        active = status in ('tracking', 'frozen') and not stop
        return dict(state=status, packet_status=packet, stop_requested=stop,
                    pose_mapping=self.pose_mapping,
                    fault_reason=self.fault_reason,
                    user_target=None if self.anchor.target is None else self.anchor.target.tolist(),
                    reference_pose=None if self.governor.reference is None else self.governor.reference.tolist(),
                    command_pose=None if self.command_pose is None else self.command_pose.tolist(),
                    relative_target=follower_relative(self.startup_anchor, self.command_pose).tolist() if active else None,
                    tracking_error_m=self.governor.tracking_error_m,
                    tracking_error_rad=self.governor.tracking_error_rad,
                    enforce_tracking_error_limits=self.config.enforce_tracking_error_limits,
                    tracking_limit_exceeded=bool(
                        self._command_tracking_limit_exceeded
                        or self.governor.tracking_error_m > self.config.max_tracking_error_m
                        or self.governor.tracking_error_rad > self.config.max_tracking_error_rad),
                    frozen_s=self.governor.frozen_s,
                    reference_progress_scale=dict(position=self.governor.position_progress_scale,
                                                  rotation=self.governor.rotation_progress_scale),
                    linear_velocity_m_s=self.linear_velocity.tolist(),
                    angular_velocity_rad_s=self.angular_velocity.tolist(),
                    rotation_follow_mode=self.config.rotation_follow_mode,
                    rotation_input_base_rad_s=self.rotation_input_base_rad_s.tolist(),
                    rotation_input_stationary=(None if self.config.rotation_follow_mode == 'anchor' else
                                               bool(np.linalg.norm(self.rotation_input_base_rad_s) == 0.)),
                    rotation_backlog_discarded_rad=self.rotation_backlog_discarded_rad)

    def _incremental_rotation(self, packet, tracking_target, dt_s):
        """Consume each accepted orientation once, in base axes, without a queue.

        Speed limiting intentionally discards unexecuted angular travel. This is
        motion following, not eventual arrival at the absolute anchor attitude.
        """
        self._rotation_sample_elapsed_s += dt_s
        self.rotation_input_base_rad_s.fill(0.)
        current = Rotation.from_quat(tracking_target[3:])
        if packet in ('anchored', 'updated'):
            if packet == 'updated' and self._rotation_previous_target is not None:
                self.rotation_input_base_rad_s = ((current * self._rotation_previous_target.inv()).as_rotvec()
                                                  / self._rotation_sample_elapsed_s)
                if np.linalg.norm(self.rotation_input_base_rad_s) < self.config.rotation_velocity_deadzone_rad_s:
                    self.rotation_input_base_rad_s.fill(0.)
            self._rotation_previous_target = current
            self._rotation_sample_elapsed_s = 0.
        rotation = Rotation.from_quat(self.command_pose[3:])
        angular = clamp_vector(rotation.inv().apply(self.rotation_input_base_rad_s),
                               self.config.max_angular_speed_rad_s)
        self.rotation_backlog_discarded_rad = float((rotation.inv() * current).magnitude())
        # Keep the position governor unchanged. Angular reference starts at the
        # continuous command, never jumps to feedback or the old hand target.
        self.governor.reference[3:] = self.command_pose[3:]
        target = tracking_target.copy()
        target[3:] = (rotation * Rotation.from_rotvec(angular * dt_s)).as_quat()
        return angular, target

    def _angular_acceleration_limit(self, desired):
        if (self.config.max_angular_brake_rad_s2 is not None
                and np.linalg.norm(desired) < np.linalg.norm(self.angular_velocity)
                and np.dot(desired - self.angular_velocity, self.angular_velocity) < 0.):
            return self.config.max_angular_brake_rad_s2
        return self.config.max_angular_accel_rad_s2

    def step(self, command, actual, dt_s, *, sample_age_s=0., feedback_age_s=0., has_error=False,
             continuous_reanchor=False):
        if self.startup_anchor is None:
            raise RuntimeError('Call start() with an explicit follower startup pose')
        actual = pose(actual)
        for name, value in [('dt_s', dt_s), ('sample_age_s', sample_age_s), ('feedback_age_s', feedback_age_s)]:
            if not math.isfinite(value) or value < 0 or (name == 'dt_s' and value == 0):
                raise ValueError(f'{name} must be finite and nonnegative (dt_s positive)')
        if type(has_error) is not bool:
            raise ValueError('has_error must be bool')
        if self.fault_reason:
            return self._result('fault', stop=True)
        reason = ('controller_fault' if has_error else
                  'control_gap' if dt_s > self.config.max_dt_s else
                  'feedback_stale' if feedback_age_s > self.config.max_feedback_age_s else None)
        packet = None
        self.input_age_s += dt_s
        if command is not None and reason is None:
            # Reject older packets. Accept an equal-sequence release: existing
            # mapper logs reuse the final active sequence on release, and a
            # release must never be mistaken for another active duplicate.
            key = (command.get('session_id'), command.get('anchor_id')) if isinstance(command, dict) else None
            seq = command.get('sample_sequence') if isinstance(command, dict) else None
            if (key is not None and self.anchor.key is not None and key[0] == self.anchor.key[0]
                    and type(key[1]) is int and type(seq) is int
                    and (key[1] < self.anchor.key[1] or (key == self.anchor.key and
                         (seq < self.anchor.sequence or (seq == self.anchor.sequence and command.get('active') is not False))))):
                packet = 'duplicate' if key == self.anchor.key and seq == self.anchor.sequence else 'stale'
            else:
                packet = self.anchor.update(command, actual)
                self.input_age_s = sample_age_s
                if packet == 'anchored':
                    # Keep the measured clutch anchor, but carry its existing
                    # command/feedback offset into this segment. A stationary
                    # hand must not trigger catch-up or rewind the UDP stream.
                    self._reanchor_translation = (self.command_pose[:3] - actual[:3]
                                                  if continuous_reanchor else np.zeros(3))
                    self._reanchor_rotation = (
                        Rotation.from_quat(self.command_pose[3:]) * Rotation.from_quat(actual[3:]).inv()
                        if continuous_reanchor else Rotation.identity())
                    self.governor.engage(self.command_pose if continuous_reanchor else actual)
                    if not continuous_reanchor:
                        self.command_pose = actual.copy()
                        self.linear_velocity.fill(0.)
                        self.angular_velocity.fill(0.)
                    self._release_elapsed_s = self._release_settled_s = 0.
                    self._release_previous_actual = None
                    self._release_previous_feedback_time_s = None
                elif packet == 'inactive':
                    self.governor.release(actual)
                    self.linear_velocity.fill(0.)
                    self.angular_velocity.fill(0.)
                    return self._result('inactive', stop=True, packet=packet)
        if self.anchor.active and self.input_age_s > self.config.max_input_age_s:
            reason = reason or 'input_stale'
        if reason is not None:
            self.fault_reason = reason
            self.governor.force_fault(actual)
            return self._result('fault', stop=True, packet=packet)
        if not self.anchor.active:
            return self._result('inactive', packet=packet)
        tracking_target = np.r_[self.anchor.target[:3] + self._reanchor_translation,
                                (self._reanchor_rotation * Rotation.from_quat(self.anchor.target[3:])).as_quat()]
        angular_input = None
        governor_target = tracking_target
        if self.config.rotation_follow_mode == 'incremental_velocity':
            angular_input, governor_target = self._incremental_rotation(packet, tracking_target, dt_s)
        state = self.governor.step(governor_target, actual, dt_s)
        if state == 'fault':
            self.fault_reason = 'tracking_error'
            return self._result('fault', stop=True, packet=packet)
        reference = self.governor.reference
        if self.config.limit_acceleration:
            # Frontend reference extension. Translation velocity uses base axes;
            # angular velocity uses command-local axes, unlike vr_vel's full loop.
            rotation = Rotation.from_quat(self.command_pose[3:])
            linear = clamp_vector((reference[:3] - self.command_pose[:3]) * self.config.position_gain,
                                  self.config.max_linear_speed_m_s)
            angular = clamp_vector((rotation.inv() * Rotation.from_quat(reference[3:])).as_rotvec() * self.config.rotation_gain,
                                   self.config.max_angular_speed_rad_s)
            if angular_input is not None:
                angular = angular_input * self.governor.rotation_progress_scale
            if state == 'frozen':
                linear.fill(0.)
                angular.fill(0.)
            # Conservative frontend adaptation: norm-bounded velocity changes.
            # C++ slew_vector above is component-wise; applying that directly
            # can temporarily exceed a Cartesian vector speed cap on turns.
            self.linear_velocity += clamp_vector(linear - self.linear_velocity,
                                                 self.config.max_linear_accel_m_s2 * dt_s)
            self.angular_velocity += clamp_vector(angular - self.angular_velocity,
                                                  self._angular_acceleration_limit(angular) * dt_s)
            self.command_pose = np.r_[self.command_pose[:3] + self.linear_velocity * dt_s,
                                      (rotation * Rotation.from_rotvec(self.angular_velocity * dt_s)).as_quat()]
        else:
            self.command_pose = reference.copy()
        # Acceleration shaping adds lag and may overshoot a reference briefly.
        # It must never turn that into an unbounded command/feedback separation.
        error_m = np.linalg.norm(self.command_pose[:3] - actual[:3])
        error_rad = (Rotation.from_quat(actual[3:]).inv() * Rotation.from_quat(self.command_pose[3:])).magnitude()
        self._command_tracking_limit_exceeded = bool(
            error_m > self.config.max_tracking_error_m or error_rad > self.config.max_tracking_error_rad)
        if self.config.enforce_tracking_error_limits and self._command_tracking_limit_exceeded:
            self.fault_reason = 'command_tracking_error'
            self.governor.force_fault(actual)
            result = self._result('fault', stop=True, packet=packet)
            result.update(command_tracking_error_m=float(error_m), command_tracking_error_rad=float(error_rad))
            return result
        result = self._result(state, packet=packet)
        result['tracking_target'] = tracking_target.tolist()
        result.update(command_tracking_error_m=float(error_m), command_tracking_error_rad=float(error_rad))
        return result

    def step_released(self, actual, dt_s, *, feedback_age_s=0., has_error=False, feedback_time_s=None):
        """Bound command deceleration, then hold its endpoint (dual clutch).

        Discards the old user/reference target, not the continuous command pose.
        This shapes frontend setpoints, not actual motor braking. The other arm
        can keep tracking; only faults request the shared follower to stop.
        """
        if self.startup_anchor is None:
            raise RuntimeError('Call start() before releasing an arm')
        actual = pose(actual)
        if (not math.isfinite(dt_s) or dt_s <= 0 or not math.isfinite(feedback_age_s)
                or feedback_age_s < 0 or type(has_error) is not bool):
            raise ValueError('Invalid release timing or fault flag')
        reason = (self.fault_reason or ('controller_fault' if has_error else
                  'control_gap' if dt_s > self.config.max_dt_s else
                  'feedback_stale' if feedback_age_s > self.config.max_feedback_age_s else None))
        if reason:
            self.fault_reason = reason
            return self._result('fault', stop=True)
        if feedback_time_s is not None and (
                isinstance(feedback_time_s, bool) or not isinstance(feedback_time_s, (float, int))
                or not math.isfinite(feedback_time_s) or feedback_time_s < 0):
            raise ValueError('feedback_time_s must be a finite nonnegative timestamp')
        previous_time = self._release_previous_feedback_time_s
        if feedback_time_s is not None and previous_time is not None and feedback_time_s < previous_time:
            self.fault_reason = 'feedback_timestamp_regression'
            return self._result('fault', stop=True)
        self.anchor.active = False
        self.anchor.target = None
        self._rotation_previous_target = None
        self._rotation_sample_elapsed_s = 0.
        self.rotation_input_base_rad_s.fill(0.)
        self.governor.reference = None
        self._release_elapsed_s += dt_s
        self.linear_velocity -= clamp_vector(self.linear_velocity,
                                             self.config.max_linear_accel_m_s2 * dt_s)
        self.angular_velocity -= clamp_vector(self.angular_velocity,
                                              self._angular_acceleration_limit(np.zeros(3)) * dt_s)
        rotation = Rotation.from_quat(self.command_pose[3:])
        self.command_pose = np.r_[self.command_pose[:3] + self.linear_velocity * dt_s,
                                  (rotation * Rotation.from_rotvec(self.angular_velocity * dt_s)).as_quat()]
        error_m = float(np.linalg.norm(self.command_pose[:3] - actual[:3]))
        error_rad = float((Rotation.from_quat(actual[3:]).inv()
                           * Rotation.from_quat(self.command_pose[3:])).magnitude())
        self.governor.tracking_error_m, self.governor.tracking_error_rad = error_m, error_rad
        self._command_tracking_limit_exceeded = bool(
            error_m > self.config.max_tracking_error_m or error_rad > self.config.max_tracking_error_rad)
        if self.config.enforce_tracking_error_limits and self._command_tracking_limit_exceeded:
            self.fault_reason = 'release_tracking_error'
        stopped_command = (np.linalg.norm(self.linear_velocity) < 1e-10
                           and np.linalg.norm(self.angular_velocity) < 1e-10)
        quiet = False
        new_feedback = (feedback_time_s is None or self._release_previous_feedback_time_s is None
                        or feedback_time_s > self._release_previous_feedback_time_s)
        feedback_dt = (dt_s if feedback_time_s is None else
                       0. if self._release_previous_feedback_time_s is None else
                       feedback_time_s - self._release_previous_feedback_time_s)
        if new_feedback and self._release_previous_actual is not None and feedback_dt > 0:
            previous = self._release_previous_actual
            quiet = (np.linalg.norm(actual[:3] - previous[:3]) <= .01 * feedback_dt
                     and (Rotation.from_quat(previous[3:]).inv()
                          * Rotation.from_quat(actual[3:])).magnitude() <= .05 * feedback_dt)
        if new_feedback:
            self._release_previous_actual = actual.copy()
            self._release_previous_feedback_time_s = feedback_time_s
        # No measured stop claim from duplicate cached poses. In the configured
        # unenforced mode holding still means only frontend command deceleration.
        if not self.config.enforce_tracking_error_limits:
            self._release_settled_s = (self._release_settled_s + dt_s
                                       if stopped_command and self.fault_reason is None else 0.)
        elif not stopped_command or self.fault_reason:
            self._release_settled_s = 0.
        elif new_feedback:
            self._release_settled_s = self._release_settled_s + feedback_dt if quiet else 0.
        held = self._release_settled_s >= self.config.release_settle_s
        if (self.config.enforce_tracking_error_limits and not held
                and self._release_elapsed_s > self.config.release_timeout_s):
            self.fault_reason = self.fault_reason or 'release_settle_timeout'
        if self.fault_reason:
            return self._result('fault', stop=True)
        if held:
            self._release_elapsed_s = 0.  # A later disturbance gets its own settle budget.
        result = self._result('holding' if held else 'releasing')
        result['relative_target'] = follower_relative(self.startup_anchor, self.command_pose).tolist()
        result.update(release_elapsed_s=self._release_elapsed_s,
                      command_stopped=bool(stopped_command), feedback_quiet=bool(quiet),
                      feedback_updated=new_feedback, feedback_interval_s=feedback_dt,
                      release_stable_s=self._release_settled_s)
        return result


def replay_main(argv=None):
    """Replay JSONL or a deterministic demo. This entrypoint has no live mode."""
    import argparse
    from collections import Counter
    import json
    from pathlib import Path

    parser = argparse.ArgumentParser(description='VR 参考目标离线回放：不连接 VR、机器人或相机，不发送 UDP。')
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--input', type=Path, help='现有 teleop_diagnostics.jsonl')
    source.add_argument('--demo', action='store_true', help='使用内置接管、平移、旋转、释放示例')
    parser.add_argument('--output', type=Path, help='逐帧输出 JSONL；未指定时只打印摘要')
    parser.add_argument('--feedback-source', choices=['recorded', 'synthetic-follow', 'synthetic-hold'],
                        help='input 默认 recorded；demo 默认 synthetic-follow。合成反馈不能证明真机效果。')
    parser.add_argument('--fps', type=float, default=30., help='日志缺少时间时使用的离线名义频率，默认 30')
    parser.add_argument('--max-steps', type=int, help='最多回放多少帧')
    parser.add_argument('--no-acceleration-limit', action='store_true', help='关闭新增的前端加速度整形作对照')
    parser.add_argument('--max-reference-lead-m', type=float, help='可选平移 reference 超前上限（米）')
    parser.add_argument('--max-reference-lead-rad', type=float, help='可选姿态 reference 超前上限（弧度）')
    parser.add_argument('--pose-mapping', choices=['anchor_relative', 'calibrated_absolute'],
                        default='anchor_relative', help='目标映射方式，默认保留原锚点相对映射')
    parser.add_argument('--pose-calibration', type=Path,
                        help='新模式的标定 JSON，含 vr_to_robot_quat_xyzw 和 hand_to_tool_quat_xyzw')
    args = parser.parse_args(argv)
    if not math.isfinite(args.fps) or args.fps <= 0 or (args.max_steps is not None and args.max_steps <= 0):
        parser.error('fps 和 max-steps 必须为正数')
    if args.output and args.input and args.output.resolve() == args.input.resolve():
        parser.error('输出文件不能覆盖输入日志')
    if args.output and args.pose_calibration and args.output.resolve() == args.pose_calibration.resolve():
        parser.error('输出文件不能覆盖标定文件')
    feedback_source = args.feedback_source or ('synthetic-follow' if args.demo else 'recorded')
    if args.demo and feedback_source == 'recorded':
        parser.error('demo 没有真实反馈，请使用 synthetic-follow 或 synthetic-hold')
    config = ReferenceConfig(limit_acceleration=not args.no_acceleration_limit,
                             max_reference_lead_m=args.max_reference_lead_m,
                             max_reference_lead_rad=args.max_reference_lead_rad)
    calibration = None if args.pose_calibration is None else json.loads(args.pose_calibration.read_text())
    if isinstance(calibration, dict) and calibration.get('kind') == 'vr_pose_calibration':
        calibration = calibration.get('pose_calibration')
    controller = VRPoseController(config, pose_mapping=args.pose_mapping, pose_calibration=calibration)
    initial = np.array([.2, -.1, .3, 0., 0., 0., 1.])

    def demo_rows():
        for index in range(120):
            active = index < 90
            offset = [min(index / 30., 1.) * .02, 0., 0., 0., 0., min(index / 30., 1.) * .1]
            command = dict(active=active, session_id='offline-demo', anchor_id=1,
                           sample_sequence=index, offset=offset, gripper=0.)
            command.update(hand_pose_vr=[*offset[:3], *Rotation.from_rotvec(offset[3:]).as_quat()],
                           position_scale=1.)
            yield dict(mapped_action={'_xlevr.anchor_command': command},
                       sample_age_s=0., sample_receive_time_s=index / args.fps)

    def file_rows():
        with args.input.open() as stream:
            for index, line in enumerate(stream, 1):
                if line.strip():
                    try:
                        row = json.loads(line)
                        if row.get('event') == 'controller_speed_override':
                            continue  # Lifecycle metadata has no hand/robot sample.
                        yield row
                    except json.JSONDecodeError as error:
                        raise ValueError(f'输入第 {index} 行不是有效 JSON') from error

    counts = Counter()
    fault_frames = Counter()
    first_fault = None
    previous_time = None
    previous_source_frame = None
    previous_session = None
    previous_result = None
    segments = 0
    synthetic = feedback_source != 'recorded'
    output = None
    max_error = 0.
    nominal_time_frames = 0
    try:
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            output = args.output.open('w')
        for index, row in enumerate(demo_rows() if args.demo else file_rows()):
            if args.max_steps is not None and index >= args.max_steps:
                break
            command = (row.get('mapped_action') or {}).get('_xlevr.anchor_command')
            if command is None:
                raise ValueError(f'第 {index + 1} 帧缺少 _xlevr.anchor_command；请使用当前 mapper 生成的日志')
            if args.pose_mapping == 'calibrated_absolute' and command.get('active') and 'hand_pose_vr' not in command:
                raise ValueError('新模式需要 mapper 校验过的 hand_pose_vr；旧日志不能直接当作新模式输入')
            if row.get('pose_mapping') == 'calibrated_absolute' and row.get('pose_calibration') != calibration:
                raise ValueError('回放标定与日志标定不一致，请提供录制时的 pose_calibration')
            sample_age = row.get('sample_age_s')
            sample_time = row.get('sample_receive_time_s')
            valid_time = (isinstance(sample_time, (float, int)) and math.isfinite(sample_time) and sample_time >= 0
                          and isinstance(sample_age, (float, int)) and math.isfinite(sample_age) and sample_age >= 0)
            timestamp = sample_time + sample_age if valid_time else None
            source_frame = row.get('control_frame_index')
            session = command.get('session_id')
            boundary = (controller.startup_anchor is None or session != previous_session
                        or (source_frame is not None and previous_source_frame is not None and source_frame <= previous_source_frame))
            if boundary:
                previous_time = None
            dt = timestamp - previous_time if timestamp is not None and previous_time is not None else 1. / args.fps
            if timestamp is None or previous_time is None:
                nominal_time_frames += 1
            if dt <= 0:
                raise ValueError(f'第 {index + 1} 帧时间未递增，不能按名义频率掩盖旧时间戳')
            frame = row.get('feedback') or {}
            if synthetic:
                actual = initial.copy()
                if not boundary and feedback_source == 'synthetic-follow' and previous_result is not None:
                    actual = pose(previous_result['command_pose'])
                age = 0.
                has_error = False
            else:
                if frame.get('tcp_pq_m') is None or type(frame.get('has_error')) is not bool:
                    raise ValueError('日志缺少真实末端反馈。VR-only 日志请显式选择 --feedback-source synthetic-follow；不能作为实机验证')
                actual = pose(frame['tcp_pq_m'])
                stamp = frame.get('received_monotonic_s')
                if timestamp is None or not isinstance(stamp, (float, int)) or not math.isfinite(stamp) or stamp > timestamp:
                    raise ValueError('真实反馈缺少同一时钟域的有效时间，不能确认反馈年龄')
                age, has_error = timestamp - stamp, frame['has_error']
            if boundary:
                controller.start(actual)
                segments += 1
            if not valid_time:
                if command.get('active'):
                    raise ValueError(f'第 {index + 1} 帧有效操作缺少采样时间或输入年龄')
                sample_age = 0.
            result = controller.step(command, actual, dt, sample_age_s=sample_age,
                                     feedback_age_s=age, has_error=has_error)
            result.update(frame_index=index, source_frame_index=source_frame, dt_s=dt,
                          feedback_source=feedback_source, synthetic_feedback=synthetic,
                          offline_only=True, transmitted=False,
                          startup_anchor_assumed_from_first_feedback=True,
                          actual_pose=actual.tolist(), input_age_s=controller.input_age_s)
            counts[result['state']] += 1
            if result['fault_reason']:
                fault_frames[result['fault_reason']] += 1
                if first_fault is None:
                    first_fault = dict(frame_index=index, reason=result['fault_reason'], dt_s=dt,
                                       sample_age_s=sample_age, feedback_age_s=age)
            max_error = max(max_error, result['tracking_error_m'])
            if output:
                # No-input age is infinite only before the first accepted packet.
                if not math.isfinite(result['input_age_s']):
                    result['input_age_s'] = None
                output.write(json.dumps(result, ensure_ascii=False, allow_nan=False) + '\n')
            previous_time, previous_result = timestamp, result
            previous_source_frame, previous_session = source_frame, session
    finally:
        if output:
            output.close()
    summary = dict(offline_only=True, transmitted=False, feedback_source=feedback_source,
                   pose_mapping=args.pose_mapping, pose_calibration=calibration,
                   synthetic_feedback=synthetic, segments=segments, frames=sum(counts.values()),
                   states=dict(counts), max_reference_tracking_error_m=max_error,
                   fault_frames=dict(fault_frames), first_fault=first_fault,
                   nominal_time_frames=nominal_time_frames, input_lowpass_enabled=False,
                   acceleration_limit=config.limit_acceleration,
                   max_reference_lead_m=config.max_reference_lead_m,
                   max_reference_lead_rad=config.max_reference_lead_rad,
                   output=None if args.output is None else str(args.output))
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(replay_main())
