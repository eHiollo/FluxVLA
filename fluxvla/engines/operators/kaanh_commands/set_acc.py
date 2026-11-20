"""Acceleration-only motor configuration writes through the A10 console."""

import argparse
import copy
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
import uuid


def positive_acceleration(value):
    try:
        number = float(value)
    except (ValueError, TypeError) as exc:
        raise argparse.ArgumentTypeError('加速度须为正的有限数值，单位 °/s²') from exc
    if isinstance(value, bool) or not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('加速度须为正的有限数值，单位 °/s²')
    return number


def configure_parser(parser):
    parser.add_argument('acceleration', type=positive_acceleration)
    scope = parser.add_mutually_exclusive_group()
    scope.add_argument('--id', type=int, choices=range(12), dest='motor_id')
    scope.add_argument('--all', action='store_true', help='全部12轴（默认）')


def _context(state, *, writable=False):
    ctx = state.raw['ret_context']
    info = ctx.get('robot_msg', {})
    if (state.has_error is not False or info.get('op_mode') != 'Manual'
            or ctx.get('user_management_msg', {}).get('user') != 'Manufacturer'):
        raise RuntimeError('set_acc 要求无故障、厂家权限、手动模式；不会自动切换权限或模式')
    if not isinstance(ctx.get('program_msg', {}).get('state'), str) or not ctx['program_msg']['state']:
        raise RuntimeError('程序状态未知，拒绝修改电机配置')
    if info.get('activate') not in ('Enabled', 'Disabled'):
        raise RuntimeError('使能状态未知，拒绝修改电机配置')
    if writable and (info.get('activate') != 'Disabled' or info.get('motion') != 'Stopped'
                     or state.follower_active is not False
                     or ctx['program_msg']['state'] != 'Unloaded'):
        raise RuntimeError('write_motor 前必须确认 Disabled、Stopped、follower退出、程序Unloaded')
    return ctx


def _verify(actual, expected):
    if not isinstance(actual, list) or len(actual) != 12:
        raise RuntimeError('电机回读数量不是12')
    for index, (got, want) in enumerate(zip(actual, expected)):
        if not isinstance(got, dict) or got.keys() != want.keys():
            raise RuntimeError(f'电机 {index} 回读字段变化')
        for key, value in want.items():
            if key in ('max_acc', 'min_acc'):
                same = (type(got[key]) in (int, float)
                        and math.isclose(got[key], value, rel_tol=0, abs_tol=1e-9))
            else:
                same = got[key] == value
            if not same:
                raise RuntimeError(f'电机 {index} 回读字段 {key} 与预期不符')


def set_acceleration(robot, acceleration, *, motor_id=None, output=print, backup_root=None):
    """Set symmetric acceleration bounds; no enable, reload, retry or rollback.

    Caller starts a fresh console command (begin_console_command), as with m_init.
    A shared caller must have the console role. Motor IDs are controller indices.
    """
    acceleration = positive_acceleration(acceleration)
    if motor_id is not None and (type(motor_id) is not int or not 0 <= motor_id < 12):
        raise ValueError('motor_id 必须是0..11，或None表示全部')
    if not robot.allow_motion:
        raise PermissionError('只读连接禁止 set_acc')
    shared = getattr(robot, 'shared_control', None)
    if shared and shared.get('role') != 'console':
        raise PermissionError('set_acc 必须使用共享控制台 console 角色')
    state = robot.get_state()
    _context(state)
    original = copy.deepcopy(robot.read_motor_config())
    if (not isinstance(original, list) or len(original) != 12
            or any(not isinstance(m, dict) for m in original)
            or any('slave_id' not in m for m in original)
            or len({m['slave_id'] for m in original}) != 12):
        raise RuntimeError('电机配置必须包含12个不同的slave_id')
    ids = list(range(12)) if motor_id is None else [motor_id]
    for i in ids:
        m = original[i]
        if (m.get('motion_type') != 'rotatory'
                or any(type(m.get(k)) not in (int, float) or not math.isfinite(m[k])
                       for k in ('max_acc', 'min_acc'))):
            raise ValueError(f'电机 {i} 必须是有完整加速度配置的旋转轴（单位 °/s²）')
    desired = copy.deepcopy(original)
    for i in ids:
        desired[i].update(max_acc=acceleration, min_acc=-acceleration)
    root = (Path(backup_root) if backup_root is not None else
            Path(__file__).resolve().parents[4] / 'work_dirs/a10/motor_acc_commands')
    directory = root / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ_') + uuid.uuid4().hex[:8])
    directory.mkdir(parents=True, exist_ok=False)
    def save(name, value):
        (directory / name).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    save('before.json', original);save('desired.json', desired);save('state_before.json', state.raw)
    output(f'set_acc 备份：{directory}')
    commands, completed = [], []
    expected = copy.deepcopy(original)
    reserved = False
    def read(*, writable=False):
        st = robot.get_state();_context(st, writable=writable)
        return st
    def native(command):
        attempt = dict(command=command, reply=None)
        commands.append(attempt);save('commands.json', commands)
        reply = robot.send_controller_command(command)
        attempt['reply'] = reply;save('commands.json', commands)
        if not isinstance(reply, dict) or reply.get('ret_code') != 0:
            raise RuntimeError(f'控制器拒绝 {command.split()[0]}：{reply}')
    def wait(predicate, description):
        deadline = time.monotonic()+robot.command_timeout_s
        while True:
            st = read()
            if predicate(st):
                return st
            if time.monotonic() >= deadline:
                raise TimeoutError(f'{description}未回读确认，不继续写入')
            time.sleep(.02)
    try:
        robot.begin_shared_console_operation();reserved = True
        st = read();ctx = _context(st)
        if st.follower_active is not False:
            native('stop_follower')
        if ctx['program_msg']['state'] != 'Unloaded' or ctx['robot_msg'].get('motion') != 'Stopped':
            native('pgmctrl --stop')
            wait(lambda x: x.raw['ret_context']['robot_msg'].get('motion') == 'Stopped'
                 and x.raw['ret_context']['program_msg']['state'] != 'Running', '程序停止')
        if read().raw['ret_context']['program_msg']['state'] != 'Unloaded':
            native('pgmctrl --unload')
            wait(lambda x: x.raw['ret_context']['program_msg']['state'] == 'Unloaded', '程序卸载Unloaded')
        st = read()
        if st.raw['ret_context']['robot_msg']['activate'] == 'Enabled':
            robot.disable()
        save('state_writable.json', read(writable=True).raw)
        for i in ids:
            read(writable=True)
            _verify(robot.read_motor_config(), expected)
            command = f'write_motor --id={i} --data=' + json.dumps(
                desired[i], ensure_ascii=False, separators=(',', ':'), allow_nan=False)
            native(command)
            expected[i] = copy.deepcopy(desired[i])
            read(writable=True)
            actual = robot.read_motor_config();_verify(actual, expected)
            completed.append(i);save('after_partial.json', actual)
            output(f'电机 {i}（slave_id={actual[i]["slave_id"]}）：max_acc={acceleration:g}、min_acc={-acceleration:g} °/s²，回读一致')
        actual = robot.read_motor_config();_verify(actual, desired)
        save('after.json', actual);save('state_after.json', read(writable=True).raw)
        result = dict(success=True, completed_motor_ids=completed, max_acc_deg_s2=acceleration,
                      min_acc_deg_s2=-acceleration, other_motor_fields_unchanged=True,
                      backup_dir=str(directory))
        save('verification.json', result)
        output('set_acc 完成：所选电机回读一致；保持未使能、程序Unloaded。')
        return result
    except BaseException as exc:
        save('error.json', dict(error=repr(exc), completed_motor_ids=completed,
                               write_results_may_be_partial=True))
        output(f'set_acc 中止，已回读确认的电机：{completed}；未重试或回滚。记录：{directory}')
        raise
    finally:
        if reserved:
            robot.end_shared_console_operation()


def execute(robot, args, output):
    return set_acceleration(robot, args.acceleration, motor_id=args.motor_id, output=output)
