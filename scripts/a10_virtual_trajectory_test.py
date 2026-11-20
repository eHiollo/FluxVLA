#!/usr/bin/env python3
"""Move all twelve virtual joints on .198 and verify measured A10 feedback."""

import argparse
import json
from pathlib import Path
import runpy
import shlex
import subprocess
import sys
import time
from datetime import datetime, timezone

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
HOST = '192.168.0.198'
CONTROL_PORT = 5888
TARGET_HZ = 30
FOLLOWER_PERCENT = 30
FEEDBACK_INTERVAL = 0.10
ERROR_LIMIT_DEG = 0.5

# Run remotely as a read-only check. Fail closed for any unexpected topology.
PREFLIGHT = r"""
import json
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET

project = Path('/home/kaanh/Desktop/kaanh-project')
exe = project / 'kaanhbin.sdk-2.5.10.6'
running = []
for path in Path('/proc').glob('[0-9]*/exe'):
    try:
        if path.readlink() == exe:
            running.append(path.parent)
    except OSError:
        continue
if len(running) != 1:
    raise SystemExit('未找到唯一的新版仿真 kaanhbin 进程')
if not (running[0] / 'cwd').resolve() == project:
    raise SystemExit('运行目录与仿真工程不符')
if '/usr/kaanh/kaanh-2.5.10.6/lib/release/libkaanh_lib.so' not in (running[0] / 'maps').read_text():
    raise SystemExit('运行中的 SDK 版本与仿真工程不符')
xml = ET.parse(project / 'kaanh.xml')
slaves = list(xml.iter('EthercatSlave'))
if len(slaves) != 14 or any(slave.get('is_virtual') != 'true' for slave in slaves):
    raise SystemExit('XML 必须配置为 14 个虚拟从站')
master = subprocess.check_output(['ethercat', 'master'], text=True)
if re.findall(r'^\s*Slaves:\s*(\d+)\s*$', master, re.M) != ['0'] or re.findall(r'^\s*Link:\s*(\w+)\s*$', master, re.M) != ['DOWN']:
    raise SystemExit('EtherCAT 物理总线必须为 DOWN，物理从站数必须为 0')
connections = subprocess.check_output(['ss', '-Htn', 'state', 'established', '( sport = :5888 )'], text=True)
if connections.strip():
    raise SystemExit('5888 已有控制客户端，请先结束该控制会话')
motors = xml.findall('./Controller/MotorPoolObject/MotorConfig')
if len(motors) != 12 or any(motor.get('motion_type') != 'rotatory' for motor in motors):
    raise SystemExit('控制器 XML 必须有 12 个旋转轴')
groups = [(x.get('tg_id'), x.get('ee_ids')) for x in xml.iter('kaanh.TgConfig')
          if x.get('is_cart') == 'false']
if groups != [('1', '{0,1,2,3,4,5,6,7,8,9,10,11}')]:
    raise SystemExit('控制器关节 follower 组与双臂轴序不符')
print(json.dumps({'pid': int(running[0].name), 'limits_rad': [
    [float(motor.attrib['min_pos']), float(motor.attrib['max_pos'])] for motor in motors],
    'max_vel_rad_s': [float(motor.attrib['max_vel']) for motor in motors]}))
"""


def preflight(known_hosts):
    command = ['ssh', '-o', 'StrictHostKeyChecking=yes', '-o', 'ConnectTimeout=6',
               '-o', 'PreferredAuthentications=publickey,password']
    if known_hosts is not None:
        command += ['-o', f'UserKnownHostsFile={known_hosts}']
    command += [f'kaanh@{HOST}', shlex.join(['sudo', '-n', 'python3', '-'])]
    result = subprocess.run(command, input=PREFLIGHT, stdout=subprocess.PIPE,
                            text=True, check=True, timeout=60)
    return json.loads(result.stdout)


def prepare_virtual_vr(robot, known_hosts=None):
    """Return the isolated virtual controller to the configured m_init pose."""
    from fluxvla.engines.operators.kaanh_commands.m_init import move_to_init

    connection = robot._connection
    if (connection.host, connection.ws_port, robot.status_ws_port, robot.shared_control) != (
            HOST, CONTROL_PORT, CONTROL_PORT, None):
        raise ValueError('仿真归位只允许直连 .198:5888')
    validation = preflight(known_hosts)
    limits = np.asarray(validation['limits_rad'], dtype=float)
    target = np.concatenate([robot.init_position[name] for name in robot.joint_target_order])
    if (target.shape != (12,) or limits.shape != (12, 2) or not np.isfinite(limits).all()
            or not np.isfinite(target).all() or np.any(target < limits[:, 0])
            or np.any(target > limits[:, 1])):
        raise ValueError('m_init 目标与 .198 的十二轴关节限位不符')

    print(f'已确认 .198 使用虚拟驱动（PID {validation["pid"]}），正在准备 m_init。', flush=True)
    enable_attempted = False
    try:
        robot.connect()
        state = robot.get_state()
        info = controller_info(state)
        user = state.raw['ret_context'].get('user_management_msg', {}).get('user')
        if (user != 'Manufacturer' or info.get('op_mode') != 'Manual'
                or info.get('motion') != 'Stopped' or state.follower_active is not False
                or info.get('activate') not in ('Enabled', 'Disabled')):
            raise RuntimeError('仿真归位要求已登录、手动模式、已停止且 follower 未运行')
        if state.has_error:
            raise RuntimeError('.198 仍报告故障，请在控制器上清除报警后重试')
        if [state.arms[name].ee_id for name in robot.joint_target_order] != [0, 1]:
            raise RuntimeError('仿真归位要求按 ID 0、ID 1 排列两组关节')
        if info['activate'] == 'Disabled':
            enable_attempted = True
            robot.enable()
        move_to_init(robot, progress=lambda line: print(line, flush=True))
        print('双臂已到 m_init，已确认关节跟随退出。VR 现在可以接管。', flush=True)
    except BaseException:
        if enable_attempted:
            restore(robot, follower_attempted=False)
        raise
    finally:
        robot.close()


def trajectory_offsets(start, limits):
    limits = np.rad2deg(limits)
    magnitude = np.array([[10., 14., 18., 12., 15., 20.],
                          [10., 14., 18., 12., 15., 20.]])
    directions = np.array([[1., -1., 1., -1., 1., -1.],
                           [-1., 1., -1., 1., -1., 1.]])
    if limits.shape != (12, 2) or not np.isfinite(limits).all():
        raise ValueError('12 轴限位不完整')
    low = limits[:, 0].reshape(2, 6) + 1.
    high = limits[:, 1].reshape(2, 6) - 1.
    if (start < low).any() or (start > high).any():
        raise ValueError('起点不在关节软限位内侧')
    offsets = magnitude * directions
    offsets = np.where((start + offsets < low) | (start + offsets > high), -offsets, offsets)
    if (start + offsets < low).any() or (start + offsets > high).any():
        raise ValueError('当前姿态没有足够余量完成规定的关节动作')
    return offsets


def controller_info(state):
    return state.raw['ret_context']['robot_msg']


def angle_rows(state, order, field):
    return np.rad2deg(np.stack([getattr(state.arms[name], field) for name in order]))


def read_active(robot):
    state = robot.get_state(timeout_s=0.25)
    if (state.has_error or state.follower_active is not True or state.follower_mode != 'Axis'
            or tuple(controller_info(state).get(key) for key in ('op_mode', 'activate'))
            != ('Manual', 'Enabled')):
        raise RuntimeError(f'跟随状态异常 {controller_info(state).get("motion")}, '
                           f'{state.follower_active}, {state.follower_mode}')
    return state


def measure(robot, order, start, offset, speed_deg_s, name, log):
    latest = read_active(robot)
    angles = []
    final_error = None
    # A quintic ramp peaks at 1.875 times its mean speed.
    ramp_duration = max(4., float(np.max(1.875 * np.abs(offset) / speed_deg_s)))
    print(f'{name} 开始，单程约 {ramp_duration:.1f} 秒', flush=True)
    for step_name, origin, end, duration, ramp in (
        ('out', start, start + offset, ramp_duration, True),
        ('peak', start + offset, start + offset, 25., False),
        ('back', start + offset, start, ramp_duration, True),
        ('home', start, start, 25., False),
    ):
        began = time.monotonic()
        next_send = began
        next_feedback = began
        in_range_since = None
        settled = False
        while True:
            now = time.monotonic()
            if now - began >= duration:
                break
            if now - latest.received_monotonic_s > 0.30:
                raise TimeoutError('关节反馈已过期')
            if now >= next_feedback:
                latest = read_active(robot)
                actual = angle_rows(latest, order, 'actual_joints_rad')
                controller_target = angle_rows(latest, order, 'target_joints_rad')
                angles.append(actual)
                log.write(json.dumps(dict(
                    phase=name, segment=step_name, monotonic_s=latest.received_monotonic_s,
                    actual_deg=actual.tolist(), controller_target_deg=controller_target.tolist()
                )) + '\n')
                next_feedback = time.monotonic() + FEEDBACK_INTERVAL
                if not ramp:
                    if np.max(np.abs(actual - end)) <= ERROR_LIMIT_DEG:
                        if in_range_since is None:
                            in_range_since = latest.received_monotonic_s
                        if latest.received_monotonic_s - in_range_since >= 0.4:
                            settled = True
                            break
                    else:
                        in_range_since = None
            elapsed = time.monotonic() - began
            if elapsed >= duration:
                break
            if time.monotonic() >= next_send:
                u = min(1., elapsed / duration)
                blend = (u*u*u*(10 + u*(-15 + 6*u))) if ramp else 1.
                commanded = origin + (end - origin) * blend
                robot.send_joints(dict(zip(order, np.deg2rad(commanded))))
                log.write(json.dumps(dict(
                    phase=name, segment=step_name, sent_monotonic_s=time.monotonic(),
                    commanded_deg=commanded.tolist()
                )) + '\n')
                next_slot = int((time.monotonic() - began) * TARGET_HZ) + 1
                next_send = began + next_slot / TARGET_HZ
            time.sleep(max(0., min(next_send, next_feedback) - time.monotonic()))
        if not ramp:
            latest = read_active(robot)
            current = angle_rows(latest, order, 'actual_joints_rad')
            angles.append(current)
            final_error = float(np.max(np.abs(current - end)))
            if final_error > ERROR_LIMIT_DEG or not settled:
                raise RuntimeError(f'{name} {step_name} 未在 {duration:g} 秒内稳定到位，'
                                   f'最大误差 {final_error:.2f}°')
    excursion = np.max(np.abs(np.stack(angles) - start), axis=0)
    active = np.abs(offset) > 0
    if (excursion[active] < np.abs(offset[active]) - ERROR_LIMIT_DEG).any():
        raise RuntimeError(f'{name} 未观察到所有指定轴到达幅度')
    if (excursion[~active] > ERROR_LIMIT_DEG).any():
        raise RuntimeError(f'{name} 未指定运动的轴偏移超过 0.5°')
    print(f'{name} 通过，各轴最大离开起点角度 {np.round(excursion, 2).tolist()}，'
          f'归位误差 {final_error:.3f}°', flush=True)


def restore(robot, follower_attempted):
    if not robot.is_connected:
        robot.connect()
    if follower_attempted:
        stopped = robot.request_stop_follower()
        # A transport reconnect clears the operator's local ownership flags.
        if not stopped['requested'] and robot.get_state().follower_active is not False:
            reply = robot.send_controller_command('stop_follower')
            if reply.get('ret_code') != 0:
                raise RuntimeError(f"控制器拒绝停止：{reply.get('ret_msg')}")
    state = robot.wait_stopped(timeout_s=4.)
    if controller_info(state).get('activate') == 'Enabled':
        state = robot.disable()
    info = controller_info(state)
    if (info.get('activate'), info.get('motion'), state.follower_active) != (
            'Disabled', 'Stopped', False):
        raise RuntimeError(f'停止未确认：{info.get("activate")}, {info.get("motion")}, '
                           f'follower={state.follower_active}')
    print('最终状态 Disabled / Stopped，Follower 已退出', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ssh-known-hosts', type=Path, help='可选的 SSH 主机指纹文件')
    args = parser.parse_args()
    from fluxvla.engines.operators.a10_operator import A10Operator

    robot = None
    enable_attempted = follower_attempted = False
    error = None
    try:
        validation = preflight(args.ssh_known_hosts)
        config = runpy.run_path(str(ROOT / 'configs/a10/a10_dual_arm_operator.py'))['operator']
        settings = dict(config)
        settings.pop('type')
        settings.update(host=HOST, ws_port=CONTROL_PORT, status_ws_port=CONTROL_PORT,
                        shared_control=None, status_poll_interval_s=None, allow_motion=True,
                        joint_follower_velocity_percent=FOLLOWER_PERCENT, joint_follower_filter=False)
        robot = A10Operator(**settings)
        robot.connect()
        state = robot.get_state()
        info = controller_info(state)
        user = state.raw['ret_context'].get('user_management_msg', {}).get('user')
        if (state.has_error or state.follower_active is not False or user != 'Manufacturer'
                or (info.get('op_mode'), info.get('activate'), info.get('motion'))
                != ('Manual', 'Disabled', 'Stopped')):
            raise RuntimeError(f'需要自动登录及 Manual / Disabled / Stopped，'
                               f'当前 {user}, {info.get("op_mode")}, {info.get("activate")}, '
                               f'{info.get("motion")}, follower={state.follower_active}')
        order = robot.joint_target_order
        if [state.arms[name].ee_id for name in order] != [0, 1]:
            raise RuntimeError('FluxVLA 关节组必须按 ID 0、ID 1 顺序发送')
        start = angle_rows(state, order, 'actual_joints_rad')
        offsets = trajectory_offsets(start, np.asarray(validation['limits_rad']))
        jog_percent = state.raw['ret_context']['motion_msg'].get('jog_vel')
        speed = np.rad2deg(np.asarray(validation['max_vel_rad_s'], dtype=float)).reshape(2, 6)
        if (type(jog_percent) not in (int, float) or not 0 < jog_percent <= 100
                or not np.isfinite(speed).all() or (speed <= 0).any()):
            raise RuntimeError('无法确认控制器手动倍率和各轴速度上限')
        speed *= FOLLOWER_PERCENT * jog_percent / 10000 * 0.75
        # At high controller override, retain a conservative rate for this visual check.
        speed = np.minimum(speed, 10.)
        if np.max(1.875 * np.abs(offsets) / speed) > 60:
            raise RuntimeError('当前倍率过低，单程预计超过 60 秒')
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        output = ROOT / 'work_dirs/a10/virtual_trajectory' / f'{stamp}.jsonl'
        output.parent.mkdir(parents=True, exist_ok=True)
        print(f'虚拟隔离确认，控制器 PID {validation["pid"]}，记录 {output}', flush=True)
        with output.open('x', encoding='utf-8', buffering=1) as log:
            log.write(json.dumps(dict(host=HOST, start_deg=start.tolist(),
                                      offset_deg=offsets.tolist(), target_hz=TARGET_HZ,
                                      jog_percent=jog_percent, follower_percent=FOLLOWER_PERCENT)) + '\n')
            enable_attempted = True
            robot.enable()
            follower_attempted = True
            robot.start_follower('joint')
            for side in range(2):
                one = np.zeros_like(offsets)
                one[side] = offsets[side]
                measure(robot, order, start, one, speed, f'ID {side} 六轴', log)
            measure(robot, order, start, offsets, speed, '双臂十二轴', log)
    except (Exception, KeyboardInterrupt) as exc:
        error = str(exc) or '测试已中断'
    finally:
        if robot is not None:
            try:
                if enable_attempted:
                    restore(robot, follower_attempted)
            except Exception as exc:
                error = f'{error + "；" if error else ""}最终停止确认失败：{exc}'
            finally:
                robot.close()
    if error:
        print(f'验收失败：{error}', file=sys.stderr)
        return 1
    print('在线轨迹验收通过', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
