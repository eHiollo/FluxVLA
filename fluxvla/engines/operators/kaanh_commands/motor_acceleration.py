"""Strict startup acceleration policy; no stop, unload, disable or enable actions."""

import math

from .set_acc import positive_acceleration, set_acceleration, _context


def acceleration_matches(motors, acceleration):
    if (not isinstance(motors, list) or len(motors) != 12
            or any(not isinstance(m, dict) or m.get('motion_type') != 'rotatory'
                   or any(type(m.get(key)) not in (int, float)
                          or not math.isfinite(m[key]) for key in ('max_acc', 'min_acc'))
                   for m in motors)):
        raise RuntimeError('运动前加速度检查失败：需要12轴完整旋转电机配置')
    return all(math.isclose(m['max_acc'], acceleration, rel_tol=0, abs_tol=1e-6)
               and math.isclose(m['min_acc'], -acceleration, rel_tol=0, abs_tol=1e-6)
               for m in motors)


class PreparedMotorWriter:
    """Reuse set_acc but reject every state preparation or motion command.

    Every state read enforces Manufacturer, Manual, Disabled, Stopped, inactive,
    and Unloaded. Even if state changes, the existing set_acc preparation path
    cannot stop/unload/disable the controller through this restricted adapter.
    """
    def __init__(self, robot):
        self.robot = robot

    def __getattr__(self, name):
        return getattr(self.robot, name)

    def get_state(self):
        state = self.robot.get_state()
        _context(state, writable=True)
        return state

    def disable(self):
        raise PermissionError('启动检查禁止自动改变使能状态')

    def send_controller_command(self, command):
        if not command.startswith('write_motor --id='):
            raise PermissionError('启动检查只允许完整电机配置写入，禁止停止、卸载或运动命令')
        self.get_state()
        return self.robot.send_controller_command(command)


def ensure_motor_acceleration(robot, acceleration, *, output=print, backup_root=None):
    acceleration = positive_acceleration(acceleration)
    if acceleration_matches(robot.read_motor_config(), acceleration):
        return dict(changed=False, max_acc_deg_s2=acceleration, min_acc_deg_s2=-acceleration)
    message = (f'电机加速度与配置 ±{acceleration:g}°/s² 不一致，禁止启动运动。'
               '写入要求厂家权限、手动模式、未使能、Stopped、follower退出、程序Unloaded；'
               '本程序不会停止其他运动、卸载程序或改变使能，请准备上述状态后重试。')
    try:
        _context(robot.get_state(), writable=True)
    except (RuntimeError, KeyError, TypeError) as exc:
        raise RuntimeError(message) from exc
    result = set_acceleration(PreparedMotorWriter(robot), acceleration,
                              output=output, backup_root=backup_root)
    return dict(result, changed=True)
