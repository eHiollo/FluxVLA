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

    def __post_init__(self):
        for name, value in vars(self).items():
            if name == 'limit_acceleration':
                if type(value) is not bool:
                    raise ValueError('limit_acceleration must be bool')
            elif isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be positive and finite')


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

    def engage(self, actual):
        self.reference = pose(actual)
        self.state, self.frozen_s = 'tracking', 0.
        self.tracking_error_m = self.tracking_error_rad = 0.

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
        rotation = Rotation.from_quat(ref[3:])
        self.tracking_error_m = float(np.linalg.norm(ref[:3] - actual[:3]))
        self.tracking_error_rad = float((rotation.inv() * Rotation.from_quat(actual[3:])).magnitude())
        if self.tracking_error_m > cfg.max_tracking_error_m or self.tracking_error_rad > cfg.max_tracking_error_rad:
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
                    frozen_s=self.governor.frozen_s,
                    linear_velocity_m_s=self.linear_velocity.tolist(),
                    angular_velocity_rad_s=self.angular_velocity.tolist())

    def step(self, command, actual, dt_s, *, sample_age_s=0., feedback_age_s=0., has_error=False):
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
                    self.governor.engage(actual)
                    self.command_pose = actual.copy()
                    self.linear_velocity.fill(0.)
                    self.angular_velocity.fill(0.)
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
        state = self.governor.step(self.anchor.target, actual, dt_s)
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
            if state == 'frozen':
                linear.fill(0.)
                angular.fill(0.)
            # Conservative frontend adaptation: norm-bounded velocity changes.
            # C++ slew_vector above is component-wise; applying that directly
            # can temporarily exceed a Cartesian vector speed cap on turns.
            self.linear_velocity += clamp_vector(linear - self.linear_velocity,
                                                 self.config.max_linear_accel_m_s2 * dt_s)
            self.angular_velocity += clamp_vector(angular - self.angular_velocity,
                                                  self.config.max_angular_accel_rad_s2 * dt_s)
            self.command_pose = np.r_[self.command_pose[:3] + self.linear_velocity * dt_s,
                                      (rotation * Rotation.from_rotvec(self.angular_velocity * dt_s)).as_quat()]
        else:
            self.command_pose = reference.copy()
        # Acceleration shaping adds lag and may overshoot a reference briefly.
        # It must never turn that into an unbounded command/feedback separation.
        error_m = np.linalg.norm(self.command_pose[:3] - actual[:3])
        error_rad = (Rotation.from_quat(actual[3:]).inv() * Rotation.from_quat(self.command_pose[3:])).magnitude()
        if error_m > self.config.max_tracking_error_m or error_rad > self.config.max_tracking_error_rad:
            self.fault_reason = 'command_tracking_error'
            self.governor.force_fault(actual)
            return self._result('fault', stop=True, packet=packet)
        return self._result(state, packet=packet)


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
    config = ReferenceConfig(limit_acceleration=not args.no_acceleration_limit)
    calibration = None if args.pose_calibration is None else json.loads(args.pose_calibration.read_text())
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
                        yield json.loads(line)
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
                   output=None if args.output is None else str(args.output))
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(replay_main())
