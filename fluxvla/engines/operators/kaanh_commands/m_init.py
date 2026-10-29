"""m_init: return to the single init_position defined by the robot operator."""

import math
import threading
import time
from collections.abc import Mapping

import numpy as np


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


def plan_init_motion(start, target, joint_dofs, *, publish_hz, max_velocity_rad_s,
                     max_acceleration_rad_s2):
    """Return synchronized quintic rows, including measured start and exact end."""
    start = joint_map(start, joint_dofs, 'actual joints')
    target = joint_map(target, joint_dofs, 'init_position')
    for value in (publish_hz, max_velocity_rad_s, max_acceleration_rad_s2):
        if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
            raise ValueError('Initialization rate and motion limits must be positive and finite')
    distance = max(float(np.max(np.abs(target[name] - start[name]))) for name in start)
    # max s'(u) = 15/8; max |s''(u)| = 10/sqrt(3).
    duration = max(1 / publish_hz, 1.875 * distance / max_velocity_rad_s,
                   math.sqrt((10 / math.sqrt(3)) * distance / max_acceleration_rad_s2))
    if not math.isfinite(duration):
        raise ValueError('Initialization duration must be finite')
    steps = math.ceil(duration * publish_hz)
    duration = steps / publish_hz

    def rows():
        for index in range(steps + 1):
            u = index / steps
            blend = u ** 3 * (10 + u * (-15 + 6 * u))
            yield {name: (target[name].copy() if index == steps else
                          start[name] + blend * (target[name] - start[name]))
                   for name in start}

    return duration, rows()


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
    defaults = dict(publish_hz=50., max_velocity_rad_s=.1,
                    max_acceleration_rad_s2=.2, tolerance_rad=.01,
                    arrival_timeout_margin_s=10., feedback_interval_s=.1)
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


def move_to_init(robot):
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
                  follower_stopped=False, completed=False)
    robot.last_init_motion = result
    try:
        robot._claim_init_motion()
        claimed = True
        robot._prepare_init_motion()
        joint_map(robot._read_init_joints(), robot.joint_dofs, 'actual joints')
        robot._traj_stop_event.clear()
        attempted = True
        robot.start_follower(mode='joint')
        start = robot._read_init_joints()
        settings = robot.init_motion
        duration, rows = plan_init_motion(
            start, target, robot.joint_dofs, publish_hz=settings['publish_hz'],
            max_velocity_rad_s=settings['max_velocity_rad_s'],
            max_acceleration_rad_s2=settings['max_acceleration_rad_s2'])
        result['duration_s'] = duration
        begun = last_feedback = time.monotonic()
        deadline = begun + duration + settings['arrival_timeout_margin_s']
        for row in rows:
            if robot._traj_stop_event.is_set():
                raise InterruptedError('Initialization cancelled')
            now = time.monotonic()
            if now >= deadline:
                raise TimeoutError('Initialization exceeded its deadline')
            if now - last_feedback >= settings['feedback_interval_s']:
                joint_map(robot._read_init_joints(), robot.joint_dofs, 'actual joints')
                last_feedback = time.monotonic()
                if last_feedback >= deadline:
                    raise TimeoutError('Initialization feedback exceeded its deadline')
            robot.send_joints(row)
            # Do not catch up after a slow I/O operation: preserve row spacing.
            if robot._traj_stop_event.wait(1 / settings['publish_hz']):
                raise InterruptedError('Initialization cancelled')
        matches = 0
        while time.monotonic() < deadline:
            if robot._traj_stop_event.is_set():
                raise InterruptedError('Initialization cancelled')
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
            robot.send_joints(target)
            if robot._traj_stop_event.wait(settings['feedback_interval_s']):
                raise InterruptedError('Initialization cancelled')
        else:
            raise TimeoutError('Initialization joint arrival was not confirmed')
    finally:
        try:
            if attempted and robot.follower_stop_pending:
                robot.stop_follower()
                result['follower_stopped'] = True
        finally:
            robot.command_mode = previous_mode
            if claimed:
                robot._release_init_motion()
            robot._init_motion_lock.release()
    result['completed'] = result['arms_arrived'] and result['follower_stopped']
    if not result['completed']:
        raise RuntimeError('Initialization follower stop was not confirmed')
    return dict(result)


def configure_parser(parser):
    """m_init takes no arguments; the operator owns its target position."""


def execute(robot, args, output):
    output('执行 FluxVLA 本地 m_init，返回机器人定义的初始位置。')
    result = move_to_init(robot)
    output('机械臂初始化到位（实际关节反馈已确认），本次 follower 已停止。')
    output('夹爪初始化已完成。' if result['grippers_initialized'] else
           '夹爪初始化尚未执行（预留接口，初始开度默认 0 mm）。')
    return result
