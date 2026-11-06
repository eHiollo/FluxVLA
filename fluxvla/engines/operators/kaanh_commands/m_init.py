"""m_init: return to the single init_position defined by the robot operator."""

import math
import threading
import time
from collections.abc import Mapping

import numpy as np


class InitMotionError(RuntimeError):
    """Preserve both the motion failure and a subsequent stop failure."""

    def __init__(self, primary, stop_error, phase, sent):
        self.primary_error, self.stop_error = primary, stop_error
        super().__init__(f'初始化在 {phase} 阶段失败：{primary}；已发送 {sent} 帧目标。'
                         f'停止清理也未确认：{stop_error}')


def joint_map(values, joint_dofs, label='joints'):
    """Copy and validate a complete named control group, without truncation."""
    if not isinstance(values, Mapping) or set(values) != set(joint_dofs):
        raise ValueError(f'{label} requires exactly these arms: {tuple(joint_dofs)}')
    result = {}
    for name, dof in joint_dofs.items():
        raw = np.asarray(values[name])
        if (raw.shape != (dof,) or raw.dtype.kind not in 'iuf'
                or any(isinstance(v, (bool, np.bool_)) for v in values[name])):
            raise ValueError(f'{label}[{name}] requires {dof} numeric joint radians')
        result[name] = np.array(raw, dtype=float, copy=True)
        if not np.isfinite(result[name]).all():
            raise ValueError(f'{label}[{name}] must be finite')
    return result


def configure_init_motion(robot, *, joint_dofs, init_position,
                          init_gripper_positions=None,
                          init_motion=None):
    if init_position is None:
        raise ValueError('Set operator.init_position in the robot config before m_init')
    if (not isinstance(joint_dofs, Mapping) or not joint_dofs
            or any(not isinstance(name, str) or not name for name in joint_dofs)
            or any(type(n) is not int or n <= 0 for n in joint_dofs.values())):
        raise ValueError('joint_dofs must map arm names to positive integers')
    robot.joint_dofs = dict(joint_dofs)
    robot.init_position = joint_map(init_position, robot.joint_dofs, 'init_position')
    robot.init_gripper_positions = dict(
        {name: 0. for name in robot.joint_dofs}
        if init_gripper_positions is None else init_gripper_positions)
    if (set(robot.init_gripper_positions) != set(robot.joint_dofs)
            or any(isinstance(v, bool) or not math.isfinite(v) or v < 0
                   for v in robot.init_gripper_positions.values())):
        raise ValueError('Init grippers require a nonnegative finite width in mm per arm')
    defaults = dict(publish_hz=30., tolerance_rad=.01, arrival_timeout_s=60., feedback_interval_s=.1)
    options = {} if init_motion is None else dict(init_motion)
    if set(options) - set(defaults):
        raise ValueError(f'Unknown init_motion options: {set(options) - set(defaults)}')
    defaults.update(options)
    if any(isinstance(v, bool) or not math.isfinite(v) or v <= 0
           for v in defaults.values()):
        raise ValueError('init_motion settings must be positive and finite')
    robot.init_motion = defaults
    if not hasattr(robot, '_init_motion_lock'):
        robot._init_motion_lock = threading.Lock()


def initialize_grippers(robot, targets_mm):
    """Extension point: future drivers consume named absolute widths in mm.

    False explicitly means no gripper was commanded or confirmed.
    """
    return False


def move_to_init(robot, progress=None):
    """Block until arm arrival and confirmed follower stop; never raw fallback."""
    configure_init_motion(
        robot, joint_dofs=robot.joint_dofs,
        init_position=getattr(robot, 'init_position', None),
        init_gripper_positions=getattr(robot, 'init_gripper_positions', None),
        init_motion=getattr(robot, 'init_motion', None))
    target = joint_map(robot.init_position, robot.joint_dofs, 'init_position')
    if not robot._init_motion_lock.acquire(blocking=False):
        raise RuntimeError('Initialization is already running')
    claimed = attempted = False
    previous_mode = robot.command_mode
    result = dict(arms_arrived=False, grippers_initialized=False,
                  follower_stopped=False, completed=False, phase='preflight',
                  joint_packets_sent=0, error=None, stop_error=None)
    robot.last_init_motion = result
    failure = None
    try:
        robot._claim_init_motion()
        claimed = True
        robot._prepare_init_motion()
        joint_map(robot._read_init_joints(), robot.joint_dofs, 'actual joints')
        robot._traj_stop_event.clear()
        attempted = True
        result['phase'] = 'start_follower'
        if progress:
            progress('正在启动关节 follower；确认前不发送目标。输入 stop 可取消。')
        robot.start_follower(mode='joint')
        settings = robot.init_motion
        deadline = time.monotonic() + settings['arrival_timeout_s']
        result['phase'] = 'sending'
        if robot._traj_stop_event.is_set():
            raise InterruptedError('Initialization cancelled')
        robot.send_joints(target)
        result['joint_packets_sent'] = 1
        interval = 1. / settings['publish_hz']
        next_send = time.monotonic() + interval
        next_feedback = time.monotonic()
        if progress:
            progress(f"按 {settings['publish_hz']:g} Hz 持续发送同一初始关节目标，等待实际到位。输入 stop 可取消。")
        matches = 0
        result['phase'] = 'arrival'
        while time.monotonic() < deadline:
            if robot._traj_stop_event.is_set():
                raise InterruptedError('Initialization cancelled')
            if time.monotonic() >= next_feedback:
                actual = joint_map(robot._read_init_joints(), robot.joint_dofs, 'actual joints')
                if time.monotonic() >= deadline:
                    raise TimeoutError('Initialization feedback exceeded its deadline')
                matches = (matches + 1 if all(
                    np.max(np.abs(actual[name] - target[name])) <= settings['tolerance_rad']
                    for name in target) else 0)
                if matches == 3:
                    result['arms_arrived'] = True
                    result['grippers_initialized'] = bool(
                        initialize_grippers(robot, dict(robot.init_gripper_positions)))
                    break
                next_feedback = time.monotonic() + settings['feedback_interval_s']
            if robot._traj_stop_event.is_set():
                raise InterruptedError('Initialization cancelled')
            if time.monotonic() >= next_send:
                # Repeat the final absolute target; motion planning stays in the controller.
                robot.send_joints(target)
                result['joint_packets_sent'] += 1
                next_send += interval
                if next_send <= time.monotonic():
                    next_send = time.monotonic() + interval  # no catch-up burst after slow I/O
            wait = max(0., min(next_send, next_feedback, deadline) - time.monotonic())
            if robot._traj_stop_event.wait(wait):
                raise InterruptedError('Initialization cancelled')
        else:
            raise TimeoutError('Initialization joint arrival was not confirmed')
    except BaseException as exc:
        failure = exc
        result['error'] = f'{type(exc).__name__}: {exc}'
        raise
    finally:
        try:
            if attempted and robot.follower_stop_pending:
                try:
                    robot.stop_follower()
                    result['follower_stopped'] = True
                except BaseException as stop_error:
                    result['stop_error'] = f'{type(stop_error).__name__}: {stop_error}'
                    if failure is None:
                        result['phase'] = 'stopping'
                        raise
                    if not isinstance(failure, Exception):
                        raise failure from stop_error
                    raise InitMotionError(failure, stop_error, result['phase'],
                                          result['joint_packets_sent']) from failure
        finally:
            robot.command_mode = previous_mode
            if claimed:
                robot._release_init_motion()
            robot._init_motion_lock.release()
    result['completed'] = result['arms_arrived'] and result['follower_stopped']
    if not result['completed']:
        raise RuntimeError('Initialization follower stop was not confirmed')
    result['phase'] = 'complete'
    return dict(result)


def configure_parser(parser):
    """m_init takes no arguments; the operator owns its target position."""


def execute(robot, args, output):
    output('执行 FluxVLA 本地 m_init，返回机器人定义的初始位置。')
    result = move_to_init(robot, progress=output)
    output('机械臂初始化到位（实际关节反馈已确认），本次 follower 已停止。')
    output('夹爪初始化已完成。' if result['grippers_initialized'] else
           '夹爪初始化尚未执行（预留接口，初始开度默认 0 mm）。')
    return result
