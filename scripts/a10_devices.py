#!/usr/bin/env python3
"""检查 A10 相机/夹爪 USB 身份；默认不打开视频流或夹爪串口。"""

import argparse
import ast
from dataclasses import asdict
from datetime import datetime
import errno
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import stat
import struct
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SYS = Path('/sys/class')
DEV = Path('/dev')
LABELS = {'base_0_rgb': '外部相机', 'left_wrist_0_rgb': '左腕相机',
          'right_wrist_0_rgb': '右腕相机', 'left': '左夹爪', 'right': '右夹爪'}
STATUS = {'matched': '已匹配', 'path_changed': '路径需更新', 'missing': '未找到',
          'ambiguous': '匹配不唯一', 'unusable': '无法确认接口/路径'}


def read_attr(path):
    return path.read_text().strip() if path.exists() else ''


def usb_identity(node):
    """USB interface and physical device identity, independent of port location."""
    interface = ''
    for parent in node.resolve().parents:
        if (parent / 'bInterfaceNumber').exists() and not interface:
            interface = read_attr(parent / 'bInterfaceNumber').lower()
        if (parent / 'idVendor').exists():
            return dict(serial=read_attr(parent / 'serial'),
                        model=read_attr(parent / 'product'),
                        vendor_id=read_attr(parent / 'idVendor'),
                        product_id=read_attr(parent / 'idProduct'),
                        usb_interface=interface)
    return None


def video_formats(device):
    """V4L2 ENUM_FMT only: no streaming, no camera setting changes."""
    formats = []
    fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK)
    try:
        for index in range(64):
            buf = bytearray(64)  # struct v4l2_fmtdesc
            struct.pack_into('II', buf, 0, index, 1)  # VIDEO_CAPTURE
            try:
                fcntl.ioctl(fd, 0xC0405602, buf, True)  # VIDIOC_ENUM_FMT
            except OSError as exc:
                if exc.errno == errno.EINVAL:
                    break
                raise
            formats.append(bytes(buf[44:48]).decode('ascii', errors='replace'))
    finally:
        os.close(fd)
    return formats


def preferred_path(device, kind, *, allow_by_id=True):
    folders = ('v4l/by-id', 'v4l/by-path') if kind == 'camera' else ('serial/by-id', 'serial/by-path')
    for folder in folders:
        if not allow_by_id and folder.endswith('by-id'):
            continue
        for link in sorted((DEV / folder).glob('*')):
            if link.exists() and link.resolve() == Path(device).resolve():
                return str(link)
    return ''


def scan_devices():
    devices = []
    for node in sorted((SYS / 'video4linux').glob('video*')):
        identity = usb_identity(node)
        if identity is None:
            continue
        device = str(DEV / node.name)
        item = dict(identity, kind='camera', node=device,
                    video_index=int(read_attr(node / 'index')), formats=[], error='')
        try:
            item['formats'] = video_formats(device)
        except OSError as exc:
            item['error'] = str(exc)
        # Skip metadata endpoints; retain inaccessible nodes to report errors.
        if item['formats'] or item['error']:
            # Some cameras reuse an index on separate interfaces. Their by-id
            # symlink winner depends on enumeration order; prefer by-path.
            duplicates = [other for other in (SYS / 'video4linux').glob('video*')
                          if read_attr(other / 'index') == str(item['video_index'])
                          and (usb_identity(other) or {}).get('serial') == item['serial']]
            item['by_id_unique'] = len(duplicates) == 1
            item['path'] = preferred_path(device, 'camera', allow_by_id=item['by_id_unique'])
            devices.append(item)
    for node in sorted((SYS / 'tty').glob('tty*')):
        if not node.name.startswith(('ttyUSB', 'ttyACM')):
            continue
        identity = usb_identity(node)
        if identity:
            device = str(DEV / node.name)
            devices.append(dict(identity, kind='gripper', node=device,
                                path=preferred_path(device, 'gripper'), error=''))
    return devices


def dict_nodes(node):
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == 'dict' and not node.args:
        if any(k.arg is None for k in node.keywords):
            raise ValueError('设备配置不支持 ** 展开，请使用直接的 dict 字段')
        return {k.arg: k.value for k in node.keywords}
    if isinstance(node, ast.Dict):
        return {ast.literal_eval(k): v for k, v in zip(node.keys, node.values)}
    raise ValueError('设备清单须使用 dict(...) 或 {...}，不能执行动态表达式')


def literal(node):
    if isinstance(node, (ast.Dict, ast.Call)):
        return {key: literal(value) for key, value in dict_nodes(node).items()}
    return ast.literal_eval(node)


def load_config(source):
    nodes = {}
    for node in ast.parse(source).body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            if node.targets[0].id in ('cameras', 'grippers'):
                nodes[node.targets[0].id] = dict_nodes(node.value)
    if set(nodes) != {'cameras', 'grippers'}:
        raise ValueError('配置需要 cameras 和 grippers 两份设备清单')
    config = {group: {name: literal(node) for name, node in entries.items()}
              for group, entries in nodes.items()}
    return config, nodes


def match_devices(config, devices):
    rows = []
    for group, entries in config.items():
        for name, settings in entries.items():
            kind = 'camera' if group == 'cameras' else 'gripper'
            field = 'device' if kind == 'camera' else 'port'
            identity = settings.get('identity', {})
            required = ('serial', 'usb_interface', 'video_index') if kind == 'camera' else ('serial', 'usb_interface')
            if any(identity.get(k) is None or identity.get(k) == '' for k in required):
                raise ValueError(f'{group}.{name} 缺少身份字段：{required}')
            candidates = [d for d in devices if d['kind'] == kind and
                          all(d.get(k) == identity[k] for k in required)]
            row = dict(group=group, name=name, label=LABELS.get(name, name),
                       serial=identity['serial'], model=identity.get('model', ''),
                       field=field, configured_path=settings[field], path='', node='',
                       status='missing')
            if len(candidates) > 1:
                row['status'] = 'ambiguous'
            elif candidates:
                candidate = candidates[0]
                row.update(node=candidate['node'], path=candidate['path'])
                if candidate.get('error') or not candidate['path'] or (kind == 'camera' and not
                        set(candidate['formats']) & {'YUYV', 'UYVY', 'MJPG', 'JPEG', 'RGB3', 'BGR3'}):
                    row['status'] = 'unusable'
                    row['error'] = candidate.get('error') or '没有可确认的RGB格式或稳定设备链接'
                else:
                    current = Path(settings[field])
                    # Preserve a working by-path selection; never accept a plain /dev/videoN.
                    stable = str(current).startswith(('/dev/v4l/by-id/', '/dev/v4l/by-path/',
                                                          '/dev/serial/by-id/', '/dev/serial/by-path/'))
                    if kind == 'camera' and not candidate.get('by_id_unique', True) and '/by-id/' in str(current):
                        stable = False
                    if stable and current.exists() and current.resolve() == Path(candidate['node']).resolve():
                        row.update(status='matched', path=str(current))
                    else:
                        row['status'] = 'path_changed'
            rows.append(row)
    for row in rows:
        if row['node'] and sum(r['node'] == row['node'] for r in rows) > 1:
            row['status'] = 'ambiguous'
    return rows


def update_paths(path, original, nodes, rows):
    """Patch only literal device/port values; preserve comments and all parameters."""
    edits = []
    lines = original.splitlines(keepends=True)
    for row in rows:
        if row['status'] != 'path_changed':
            continue
        node = dict_nodes(nodes[row['group']][row['name']])[row['field']]
        if not isinstance(node, ast.Constant) or not isinstance(node.value, str):
            raise ValueError('更新路径要求 device/port 是直接的字符串')
        start = sum(map(len, lines[:node.lineno - 1])) + node.col_offset
        end = sum(map(len, lines[:node.end_lineno - 1])) + node.end_col_offset
        edits.append((start, end, repr(row['path']).encode('utf-8')))
    if not edits:
        return None
    updated = original
    for start, end, replacement in sorted(edits, reverse=True):
        updated = updated[:start] + replacement + updated[end:]
    load_config(updated.decode('utf-8'))
    if path.read_bytes() != original:
        raise ValueError('配置已被其他程序修改，取消更新；请重新扫描')
    mode = stat.S_IMODE(path.stat().st_mode)
    backup = path.with_name(path.name + '.' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.bak')
    with backup.open('xb') as stream:
        stream.write(original)
        stream.flush()
        os.fsync(stream.fileno())
    temp_name = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix=path.name + '.', suffix='.tmp', delete=False) as stream:
            temp_name = stream.name
            os.fchmod(stream.fileno(), mode)
            stream.write(updated)
            stream.flush()
            os.fsync(stream.fileno())
        if path.read_bytes() != original:
            raise ValueError('配置已被其他程序修改，取消更新；原配置未覆盖')
        os.replace(temp_name, path)
    finally:
        if temp_name and Path(temp_name).exists():
            Path(temp_name).unlink()
    return backup


def check_grippers(rows, config):
    # Load standalone driver without initializing FluxVLA, torch or ROS.
    spec = importlib.util.spec_from_file_location('_a10_devices_zx', ROOT / 'fluxvla/engines/operators/zx_gripper.py')
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    for row in rows:
        if row['group'] != 'grippers' or row['status'] not in ('matched', 'path_changed'):
            continue
        settings = config['grippers'][row['name']]
        try:
            with module.ZXGripper(row['path'], baudrate=settings['baudrate']) as gripper:
                row['gripper_state'] = asdict(gripper.read_state(settings['device_id']))
        except Exception as exc:
            row['gripper_error'] = str(exc)


def print_report(report):
    print('设备          型号       序列号                             当前节点       状态')
    for row in report['bindings']:
        print(f"{row['label']:<10} {row['model']:<10} {row['serial']:<34} {row['node'] or '-':<14} {STATUS[row['status']]}")
        if row['status'] == 'path_changed':
            print(f"  建议路径：{row['path']}")
        if row.get('error'):
            print('  检查失败：' + row['error'])
        if 'gripper_state' in row:
            state = row['gripper_state']
            print(f"  Modbus ID={state['device_id']} 已响应；位置raw={state['position_raw']}；"
                  f"报警=0x{state['alarm_a']:04X}/0x{state['alarm_b']:04X}；使能={state['enabled']}")
        if 'gripper_error' in row:
            print('  夹爪只读查询失败：' + row['gripper_error'])
    known = {(r['group'], r['serial']) for r in report['bindings']}
    unknown = {}
    for device in report['devices']:
        group = 'cameras' if device['kind'] == 'camera' else 'grippers'
        if (group, device['serial']) not in known:
            key = (group, device['serial'], device['model'])
            unknown.setdefault(key, []).append(device['node'])
    for (_, serial, model), nodes in unknown.items():
        print(f"未登记设备：{model}，序列号={serial or '无'}，节点={','.join(nodes)}；不自动分配左右")
    if report['backup']:
        print('路径已更新，原配置备份：' + report['backup'])
    elif report['update_requested']:
        print('没有可更新的路径，配置未写入。')
    print('默认不打开画面/串口；--check-grippers 仅查询配置的Modbus ID，不扫描或修改地址。')
    print('夹爪序列号属于USB转接器；转接器与左右夹爪接线互换时，需要人工重新确认。')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--config', type=Path, default=ROOT / 'configs/a10/a10_devices.py', help='设备清单路径')
    parser.add_argument('--update', action='store_true', help='先备份，只更新按身份唯一匹配设备的连接路径')
    parser.add_argument('--check-grippers', action='store_true', help='额外打开已匹配夹爪串口，只读配置ID的状态')
    parser.add_argument('--json', action='store_true', help='输出完整JSON，包含各视频接口和格式')
    args = parser.parse_args(argv)
    try:
        path = args.config.resolve()
        original = path.read_bytes()
        config, nodes = load_config(original.decode('utf-8'))
        devices = scan_devices()
        rows = match_devices(config, devices)
        if args.check_grippers:
            check_grippers(rows, config)
        backup = update_paths(path, original, nodes, rows) if args.update else None
        if backup:
            for row in rows:
                if row['status'] == 'path_changed':
                    row.update(status='matched', configured_path=row['path'])
        report = dict(bindings=rows, devices=devices, backup=str(backup) if backup else None,
                      update_requested=args.update)
        if args.json:
            print(json.dumps(report, ensure_ascii=False, indent=2))
        else:
            print_report(report)
        return int(any(row['status'] != 'matched' or row.get('gripper_error') or
                       any(row.get('gripper_state', {}).get(k, 0) for k in ('alarm_a', 'alarm_b'))
                       for row in rows))
    except (OSError, ValueError, SyntaxError) as exc:
        print(f'设备检查失败：{exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
