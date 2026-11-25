"""Console-only, reversible acceleration configuration for m_init."""

import copy
import json
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path
import uuid

from .motor_acceleration import acceleration_matches
from .set_acc import _context, _verify, _write_verified_motor_row


_BACKUP_ROOT = Path(__file__).resolve().parents[4] / 'work_dirs/a10/motor_acc_commands'


def _idle_activation(state):
    context = _context(state)
    info = context['robot_msg']
    if (info.get('motion') != 'Stopped' or state.follower_active is not False
            or context['program_msg']['state'] != 'Unloaded'):
        raise RuntimeError('切换 m_init 加速度要求机械臂已停止、follower 已退出、程序为 Unloaded')
    return info['activate']


@contextmanager
def temporary_init_acceleration(robot, acceleration, *, may_reenable, progress=None, cancel_event=None):
    robot._require_motion()
    original = robot.read_motor_config()
    if acceleration_matches(original, acceleration):
        yield
        return
    if robot.shared_control and robot.shared_control.get('role') != 'console':
        raise PermissionError('临时切换 m_init 加速度要求控制台角色')
    robot.begin_shared_console_operation(require_idle=True)
    robot._init_shared_reserved = bool(robot.shared_control)
    original_state = robot.get_state()
    if _idle_activation(original_state) != 'Enabled':
        raise RuntimeError('切换 m_init 加速度前请先按现场流程完成使能')
    original = copy.deepcopy(robot.read_motor_config())
    if acceleration_matches(original, acceleration):
        yield original_state
        return
    if len({row.get('slave_id') for row in original}) != 12 or any('slave_id' not in row for row in original):
        raise RuntimeError('m_init 电机配置须有 12 个不同的 slave_id')
    desired = [dict(row, max_acc=acceleration, min_acc=-acceleration) for row in original]
    directory = _BACKUP_ROOT / f"{datetime.now(timezone.utc):%Y%m%dT%H%M%S_%fZ}_m_init_{uuid.uuid4().hex[:8]}"
    directory.mkdir(parents=True, exist_ok=False)

    def save(name, data):
        (directory / name).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n')

    save('before.json', original)
    save('desired.json', desired)
    attempted = set()

    def cancelled():
        return (robot._command_cancelled.is_set()
                or (cancel_event is not None and cancel_event.is_set()))

    def write(command, *, restoring=False):
        with robot._io_lock:
            robot._require_motion(allow_cancelled=restoring)
            if not restoring and cancelled():
                raise InterruptedError('m_init 加速度切换已取消')
            _context(robot.get_state(), writable=True)
            reply = robot._connection.request(command, raw=True, timeout_s=robot.command_timeout_s,
                                              allow_cancelled=restoring)
            if reply.get('ret_code') != 0:
                raise RuntimeError(f'控制器拒绝 m_init 电机加速度写入：{reply}')

    if progress:
        progress(f'm_init 临时加速度 ±{acceleration:g}°/s²，原始十二轴参数备份：{directory}')
    try:
        robot._set_manual_enable(False, init_switch=True)
        expected = copy.deepcopy(original)
        for index in range(12):
            if original[index] != desired[index]:
                attempted.add(index)
                _write_verified_motor_row(robot, index, desired, expected, write)
        if cancelled():
            raise InterruptedError('m_init 加速度切换已取消')
        robot._set_manual_enable(True, init_switch=True)
        yield original_state
    finally:
        try:
            # Confirm no motion before taking power off to write. The shared
            # reservation and cancellation latch remain held through cleanup.
            activation = _idle_activation(robot.get_state())
            current = robot.read_motor_config()
            acceleration_matches(current, acceleration)  # Validate numeric bounds.
            expected = copy.deepcopy(original)
            for index in attempted:
                bounds = (current[index]['max_acc'], current[index]['min_acc'])
                old = (original[index]['max_acc'], original[index]['min_acc'])
                if bounds != old and bounds != (acceleration, -acceleration):
                    raise RuntimeError(f'电机 {index} 加速度发生未知变化')
                if bounds != old:
                    expected[index] = copy.deepcopy(desired[index])
            _verify(current, expected)
            pending = [i for i in range(12) if expected[i] != original[i]]
            if pending and activation == 'Enabled':
                robot._set_manual_enable(False, init_switch=True, init_cleanup=True)
                activation = 'Disabled'
            for index in pending:
                _write_verified_motor_row(robot, index, original, expected,
                                          lambda command: write(command, restoring=True))
            restored = robot.read_motor_config()
            _verify(restored, original)
            save('after.json', restored)
            if activation == 'Disabled' and not cancelled() and may_reenable():
                robot._set_manual_enable(True, init_switch=True)
            if progress:
                progress('已回读确认十二轴原始加速度恢复。')
        except BaseException as exc:
            save('restore_error.json', dict(error=repr(exc), attempted_motor_ids=sorted(attempted)))
            raise RuntimeError(f'm_init 原加速度恢复或状态确认失败，参数备份：{directory}；{exc}') from exc
