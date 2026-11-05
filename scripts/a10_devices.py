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


def load_operator(kind):
    filename = 'camera_operator.py' if kind == 'camera' else 'zx_gripper.py'
    spec = importlib.util.spec_from_file_location(
        '_a10_devices_' + kind, ROOT / 'fluxvla/engines/operators' / filename)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def require_matched(rows, group, name=None):
    selected = [row for row in rows if row['group'] == group and
                (name is None or row['name'] == name)]
    if not selected:
        raise ValueError('设备清单中没有所选设备')
    bad = [f"{row['label']}：{STATUS[row['status']]}" for row in selected if row['status'] != 'matched']
    if bad:
        raise ValueError('；'.join(bad) + '。请先检查设备，路径变化时单独执行 --update')
    return selected


def take_snapshots(config, rows, output_dir=None):
    from PIL import Image, ImageDraw, ImageFont, ImageOps

    selected = require_matched(rows, 'cameras')
    for row in selected:
        if not row['name'] or any(ch not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for ch in row['name']):
            raise ValueError('相机名称只能包含字母、数字、下划线或短横线')
    if output_dir is not None and output_dir.exists():
        raise FileExistsError('拍照目录已存在，请选择新目录，避免覆盖之前的检查图片')
    # Read all cameras before writing images: failure cannot masquerade as a full capture.
    camera_module = load_operator('camera')
    with camera_module.CameraOperator(config['cameras']) as cameras:
        for _ in range(5):
            frames = cameras.get_frame()
    images = {row['name']: Image.fromarray(frames[row['name']], 'RGB') for row in selected}
    folder = (output_dir if output_dir is not None else
              ROOT / 'work_dirs/a10/hardware_checks' / datetime.now().strftime('device_check_%Y%m%d_%H%M%S_%f'))
    folder = folder.resolve()
    folder.mkdir(parents=True, exist_ok=False)
    font_path = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    font = ImageFont.truetype(str(font_path), 21) if font_path.exists() else ImageFont.load_default()
    width, height, header = 640, 480, 86
    overview = Image.new('RGB', (width * len(selected), height + header), '#151a22')
    draw = ImageDraw.Draw(overview)
    manifest = dict(created=datetime.now().astimezone().isoformat(),
                    camera_received_monotonic_s=frames['camera_received_monotonic_s'],
                    hardware_synchronized=False, cameras={})
    for i, row in enumerate(selected):
        name = row['name']
        image_path = folder / (name + '.jpg')
        images[name].save(image_path, quality=95)
        label = f"{row['label']} / {name}" if font_path.exists() else name
        draw.text((i * width + 12, 8), label, font=font, fill='white')
        draw.text((i * width + 12, 43), 'SN: ' + row['serial'], font=font, fill='#9bd5ff')
        thumb = ImageOps.contain(images[name], (width, height))
        overview.paste(thumb, (i * width + (width - thumb.width) // 2,
                              header + (height - thumb.height) // 2))
        manifest['cameras'][name] = dict(serial=row['serial'], device=row['configured_path'],
                                         node=row['node'], image=str(image_path),
                                         width=images[name].width, height=images[name].height)
    overview.save(folder / 'overview.jpg', quality=95)
    (folder / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    return dict(directory=str(folder), overview=str(folder / 'overview.jpg'), **manifest)


def move_gripper(config, rows, side, action, position_raw=None):
    row, = require_matched(rows, 'grippers', side)
    settings = config['grippers'][side]
    check = settings.get('check', {})
    positions = check.get('positions_raw')
    if positions not in ((0, 1000), [0, 1000]):
        raise ValueError('检查动作只支持已验证的位置0和1000，需要配置 check.positions_raw=(0, 1000)')
    if action in ('open', 'close'):
        opened, closed = check.get('open_raw'), check.get('close_raw')
        if (type(opened) is not int or type(closed) is not int or
                opened not in positions or closed not in positions or opened == closed):
            raise ValueError('开合方向未确认；先用 --position-raw 0 / 1000 观察，再填写 check.open_raw 和 close_raw')
        position_raw = opened if action == 'open' else closed
    if type(position_raw) is not int or position_raw not in positions:
        raise ValueError('检查位置仅允许0或1000，不是毫米或完整开合行程')
    speed, torque = settings.get('speed'), settings.get('torque')
    if (type(speed) is not int or not 1 <= speed <= 5 or
            type(torque) is not int or not 1 <= torque <= 60):
        raise ValueError('设备辨认动作要求 speed 在1..5、torque在1..60，不允许提高至完整操作速度')
    gripper_module = load_operator('gripper')
    # Only the selected port is opened. No robot controller is constructed.
    with gripper_module.ZXGripper(row['configured_path'], baudrate=settings['baudrate'],
                                  max_position_raw=1000) as gripper:
        initial = gripper.read_state(settings['device_id'])
        if initial.has_error:
            raise RuntimeError(f"夹爪有报警，不发送动作：0x{initial.alarm_a:04X}/0x{initial.alarm_b:04X}")
        if not 0 <= initial.position_raw <= 1005:
            raise ValueError('实际位置不在本次0..1000检查范围附近，不发送可能的大行程动作')
        final = gripper.set_position_raw(settings['device_id'], position_raw,
                                         speed=speed, torque=torque, wait=True, move_timeout=10)
    return dict(side=side, serial=row['serial'], port=row['configured_path'],
                action=action, initial_raw=initial.position_raw, target_raw=position_raw,
                speed=speed, torque=torque, final=asdict(final))


def check_grippers(rows, config):
    module = load_operator('gripper')
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
    if report.get('snapshot'):
        print('三路相机检查图片：' + report['snapshot']['directory'])
        print('带名称和序列号的拼图：' + report['snapshot']['overview'])
    if report.get('gripper_motion'):
        motion = report['gripper_motion']
        print(f"{LABELS[motion['side']]} 检查目标raw={motion['target_raw']}；"
              f"实际raw={motion['final']['position_raw']}；完成原因={motion['final']['completion']}")
        print('串口已关闭；保持最后目标和力矩，不自动张开或掉使能。')
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
    parser.add_argument('--snapshot', action='store_true', help='三路相机各拍一张并保存带标签的拼图')
    parser.add_argument('--output-dir', type=Path, help='拍照输出目录，须为新目录；默认保存到本仓库work_dirs')
    parser.add_argument('--gripper', choices=('left', 'right'), help='仅操作指定侧夹爪，用于空载设备辨认')
    actions = parser.add_mutually_exclusive_group()
    actions.add_argument('--position-raw', type=int, choices=(0, 1000), help='小范围辨认位置，仅0或1000')
    actions.add_argument('--open', dest='open_gripper', action='store_true', help='方向确认配置后，移动到检查张开位置')
    actions.add_argument('--close', dest='close_gripper', action='store_true', help='方向确认配置后，移动到检查闭合位置')
    args = parser.parse_args(argv)
    has_action = args.position_raw is not None or args.open_gripper or args.close_gripper
    if bool(args.gripper) != bool(has_action):
        parser.error('--gripper left/right 必须配合 --position-raw 0/1000 或 --open/--close')
    if args.snapshot and args.gripper:
        parser.error('拍照和夹爪动作请分开执行')
    if (args.snapshot or args.gripper) and (args.update or args.check_grippers):
        parser.error('拍照/夹爪动作不能和 --update 或 --check-grippers 混用')
    if args.output_dir is not None and not args.snapshot:
        parser.error('--output-dir 仅用于 --snapshot')
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
        if args.snapshot:
            report['snapshot'] = take_snapshots(config, rows, args.output_dir)
        if args.gripper:
            action = 'open' if args.open_gripper else 'close' if args.close_gripper else 'position'
            report['gripper_motion'] = move_gripper(config, rows, args.gripper, action, args.position_raw)
        if args.json:
            print(json.dumps(report, ensure_ascii=False, indent=2))
        else:
            print_report(report)
        if args.snapshot or args.gripper:
            return 0
        return int(any(row['status'] != 'matched' or row.get('gripper_error') or
                       any(row.get('gripper_state', {}).get(k, 0) for k in ('alarm_a', 'alarm_b'))
                       for row in rows))
    except (OSError, ValueError, SyntaxError, RuntimeError, ImportError) as exc:
        print(f'设备检查失败：{exc}', file=sys.stderr)
        if args.gripper:
            print('不会自动重发目标、张开或掉使能；等待失败不代表夹爪已停止。', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
