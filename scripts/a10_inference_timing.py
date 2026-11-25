#!/usr/bin/env python3
"""Plot A10 inference timing from client events and optional controller captures."""

import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import re
import struct
import subprocess

import numpy as np


JOINTS = [*range(6), *range(7, 13)]
HEADER = struct.Struct('<IIQqqq')


def epoch(utc):
    return datetime.fromisoformat(utc).timestamp()


def joint_row(row):
    return np.rad2deg(np.asarray(row, dtype=float)[JOINTS])


def read_client(path):
    chunks = {}
    feedback = []
    anchors = []
    for line in path.open():
        row = json.loads(line)
        kind = row['event']
        if kind in ('sent', 'inference_end'):
            anchors.append(epoch(row['utc']) - row['monotonic_s'])
        if kind == 'feedback':
            feedback.append(row)
        if 'cycle' in row:
            chunks.setdefault(row['cycle'], {}).setdefault(kind, []).append(row)
    chunks = {cycle: group for cycle, group in chunks.items() if group.get('sent')}
    if not chunks or not anchors:
        raise ValueError('No joint packets found in the client log')
    return chunks, feedback, float(np.median(anchors))


def read_capture(path, port, offset, epoch_delta):
    command = ['tshark', '-n', '-r', str(path), '-Y', f'udp.dstport == {port}',
               '-T', 'fields', '-e', 'frame.time_epoch', '-e', 'udp.payload']
    try:
        result = subprocess.run(command, capture_output=True, text=True, check=True)
    except FileNotFoundError as exc:
        raise ValueError('Install tshark to read the UDP capture') from exc
    except subprocess.CalledProcessError as exc:
        raise ValueError(f'tshark could not read the capture: {exc.stderr.strip()}') from exc
    packets = []
    for line in result.stdout.splitlines():
        stamp, _, payload = line.partition('\t')
        packet = bytes.fromhex(payload.replace(':', ''))
        if len(packet) < HEADER.size:
            continue
        size, *flags = HEADER.unpack_from(packet)
        if any(flags) or size != len(packet) - HEADER.size:
            continue
        body = packet[HEADER.size:].decode('utf-8', errors='replace')
        try:
            if body.startswith('jp --motor_pos='):
                values = json.loads(body[len('jp --motor_pos='):])
            else:
                values = json.loads(body).get('motor_pos')
            wire = np.asarray(values, dtype=float)
        except (ValueError, TypeError, AttributeError):
            continue
        if wire.shape == (12,) and np.isfinite(wire).all():
            # Current A10 wire order is ee_id 0, then ee_id 1. Policy is 1, then 0.
            packets.append((float(stamp) - offset - epoch_delta, np.r_[wire[6:], wire[:6]]))
    return packets


def read_controller(csv_path, inspect_path, axes, offset, epoch_delta):
    segments = {}
    for record in json.loads(inspect_path.read_text()):
        meta = record.get('metadata')
        if not meta:
            continue
        segments[Path(record['file']).name] = meta
    if not segments:
        raise ValueError('Recorder inspect JSON has no time calibration metadata')
    feedback = []
    with csv_path.open(newline='') as stream:
        for row in csv.DictReader(stream):
            meta = segments[row['segment_file']]
            physical_axes = meta.get('axes', [])
            if max(axes) >= len(physical_axes) or any(physical_axes[i].get('position_unit') != 'rad' for i in axes):
                raise ValueError('Selected recorder axes must exist and have position_unit=rad')
            start_wall = meta['wall_time_unix_ns'] / 1e9
            stamp = start_wall + (int(row['monotonic_ns']) - meta['monotonic_start_ns']) / 1e9
            stamp -= offset + epoch_delta
            joints = {}
            for name, mask in [('actual_pos', 1), ('target_pos', 16)]:
                joints[name] = np.array([
                    np.rad2deg(float(row[f'axis{i}_{name}']))
                    if int(row[f'axis{i}_valid']) & mask and int(row[f'axis{i}_link']) & 3 == 3
                    else np.nan for i in axes])
            feedback.append((stamp, joints['target_pos'], joints['actual_pos']))
    return sorted(feedback, key=lambda sample: sample[0])


def sent_stamp(packet):
    return packet.get('udp_send_started_s')


def match_udp(packets, capture, clock_error, max_transit):
    """Match by joint values, timestamp, and order. Admit clock-overlap ambiguity."""
    result = {}
    times = np.array([stamp for stamp, _ in capture])
    last_used = -1
    for packet in packets:
        sent = sent_stamp(packet)
        if sent is None:
            continue
        first = max(last_used + 1, int(times.searchsorted(sent - clock_error)))
        end = int(times.searchsorted(sent + clock_error + max_transit, side='right'))
        goal = joint_row(packet['target'])
        matches = [i for i in range(first, end)
                   if np.max(np.abs(capture[i][1] - goal)) <= 1e-6]
        if len(matches) == 1:
            last_used = matches[0]
            result[packet['frame']] = (float(times[last_used]), 'matched')
        else:
            result[packet['frame']] = (None, 'ambiguous' if matches else 'unmatched')
    return result


def nearby_times(points, starts, samples, column, tolerance):
    """Only use samples near a single distinct point in this chunk."""
    if not samples:
        return [(None, 'no_capture') for _ in points]
    stamps = np.asarray([s[0] for s in samples])
    angles = np.asarray([s[column] for s in samples])
    near = np.max(np.abs(angles[:, None, :] - points[None, :, :]), axis=2) <= tolerance
    one_point = near.sum(axis=1) == 1
    position = 0
    result = []
    for i, start in enumerate(starts):
        eligible = (stamps >= start) & (np.arange(len(stamps)) >= position)
        candidates = np.flatnonzero(eligible & near[:, i] & one_point)
        if not len(candidates):
            status = 'overlap' if np.any(eligible & near[:, i]) else 'not_observed'
            result.append((None, status))
            continue
        position = int(candidates[0]) + 1
        stamp = float(stamps[position - 1])
        result.append((stamp, 'near_at_start' if position == 1 or near[position - 2, i] else 'near'))
    return result


def first(group, event):
    return next(iter(group.get(event, ())), {})


def model_ready(group):
    event = first(group, 'inference_end')
    return event.get('response_received_s') or event.get('finished_s')


def point_rows(group, end, capture, controller, clock_error, tolerance, max_transit):
    by_point = {}
    packets = group['sent']
    for packet in packets:
        by_point.setdefault(packet.get('policy_frame') or packet['frame'], []).append(packet)
    matching = match_udp(packets, capture, clock_error, max_transit) if capture is not None else {}
    groups = list(by_point.values())
    starts = []
    for g in groups:
        sent = sent_stamp(g[0]) or g[0]['send_started_s']
        received = [matching[p['frame']][0] for p in g
                    if matching.get(p['frame'], (None,))[0] is not None]
        starts.append(max(sent, min(received) - clock_error) if received else sent)
    points = np.stack([joint_row(g[0]['target']) for g in groups])
    samples = [s for s in (controller or []) if starts[0] - clock_error <= s[0] <= end]
    internal = nearby_times(points, starts, samples, 1, tolerance)
    actual = nearby_times(points, starts, samples, 2, tolerance)
    rows = []
    for i, (point, group_packets) in enumerate(by_point.items()):
        row = dict(point=point, fx_plan=group_packets[0]['scheduled_s'],
                   fx_send_1=sent_stamp(group_packets[0]), fx_send_2=None,
                   udp_rx_1=None, udp_rx_2=None, udp_status_1=None, udp_status_2=None,
                   controller_near=internal[i][0], controller_status=internal[i][1],
                   actual_near=actual[i][0], actual_status=actual[i][1])
        for repeat, packet in enumerate(group_packets[:2], 1):
            row[f'fx_send_{repeat}'] = sent_stamp(packet)
            rx, status = matching.get(packet['frame'], (None, 'no_capture'))
            row[f'udp_rx_{repeat}'], row[f'udp_status_{repeat}'] = rx, status
        rows.append(row)
    return rows


def joint_feedback(feedback, controller, joint, start, end):
    if controller is not None:
        return [(stamp, target[joint], actual[joint]) for stamp, target, actual in controller
                if start <= stamp <= end]
    return [(r['received_monotonic_s'],
             np.nan if r['controller_target_joints_rad'] is None else np.rad2deg(r['controller_target_joints_rad'][joint]),
             np.rad2deg(r['actual_joints_rad'][joint])) for r in feedback
            if start <= r['received_monotonic_s'] <= end]


def draw_detail(directory, cycle, group, feedback, controller, joint, rows, detail_ms, detail_points,
                next_chunk_start):
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from matplotlib.ticker import MultipleLocator

    families = ['DejaVu Sans']
    for family in ('Noto Sans CJK SC', 'Noto Sans CJK JP', 'Droid Sans Fallback'):
        try:
            font = font_manager.findfont(family, fallback_to_default=False)
        except ValueError:
            continue
        families.append(font_manager.FontProperties(fname=font).get_name())
        break
    style = {'font.family': families, 'axes.unicode_minus': False}
    sent = [(sent_stamp(p), joint_row(p['target'])[joint]) for p in group['sent'] if sent_stamp(p) is not None]
    origin = sent[0][0] if sent else rows[0]['fx_plan']
    x_ms = lambda stamp: (np.asarray(stamp) - origin) * 1000
    end = origin + detail_ms / 1000
    if next_chunk_start is not None and origin < next_chunk_start < end:
        end = next_chunk_start
        detail_ms = float(x_ms(end))
    samples = joint_feedback(feedback, controller, joint, origin - .1, end)
    sample_x, target, actual = np.asarray(samples).T if samples else ([], [], [])
    source = '高频采样时刻' if controller is not None else 'GET 回包时刻'
    stop = first(group, 'stop').get('monotonic_s')
    reached = first(group, 'target_arrival_confirmed').get('monotonic_s')
    with plt.rc_context(style):
        fig, (packets, angle, error) = plt.subplots(3, 1, figsize=(14, 10), layout='constrained')
        title = f'第 {cycle + 1} 个动作块细节 · ID {1 - joint // 6} J{joint % 6 + 1}'
        requested = first(group, 'inference_start').get('started_s')
        ready = model_ready(group)
        if requested is not None and ready is not None and sent:
            title += f'\n等待模型回复 {(ready - requested) * 1000:.1f} ms · 回复到首包 {-x_ms(ready):.1f} ms'
        fig.suptitle(title, fontsize=16)
        packet_rows = rows[:detail_points]
        fields = (
            ('fx_plan', '灰竖线  计划首发', '|', '#64748b'),
            ('fx_send_1', '深蓝点  实际首发', 'o', '#2563eb'),
            ('fx_send_2', '浅蓝点  重发同一目标', 'o', '#60a5fa'),
            ('udp_rx_1', '深橙叉  控制器收到首发', 'x', '#b45309'),
            ('udp_rx_2', '浅橙叉  控制器收到重发', 'x', '#f59e0b'),
        )
        for field, label, marker, color in fields:
            marks = [(float(x_ms(r[field])), r['point']) for r in packet_rows if r[field] is not None]
            if marks:
                xs, ids = zip(*marks)
                packets.scatter(xs, ids, s=55, marker=marker, color=color, label=label, zorder=3)
        for row in packet_rows:
            if row['fx_send_1'] is not None:
                lag = float(x_ms(row['fx_send_1']) - x_ms(row['fx_plan']))
                packets.annotate(f'{lag:+.1f} ms', (x_ms(row['fx_send_1']), row['point']),
                                 xytext=(0, 8), textcoords='offset points', ha='center', fontsize=9)
        packets.set(title=f'前 {len(packet_rows)} 个动作点 · 数字是实际首发比计划晚多少毫秒',
                    xlabel='相对第一个实际 UDP 包（ms）', ylabel='动作点',
                    yticks=[r['point'] for r in packet_rows])
        packets.xaxis.set_major_locator(MultipleLocator(10 if len(packet_rows) <= 8 else 20))
        packets.margins(x=.065, y=.2)
        packets.legend(loc='upper left', ncol=3)

        if sent:
            stamps, q = map(np.asarray, zip(*sent))
            angle.step(x_ms(stamps), q, where='post', color='#2563eb', marker='.',
                       markersize=4, label='蓝线  FluxVLA 实发目标', zorder=4)
            angle.axvline(x_ms(stamps[-1]), color='#b45309', linestyle='--', linewidth=1,
                          label='橙虚线  末包发出')
            angle.axvspan(x_ms(stamps[-1]), x_ms(stop) if stop is not None else detail_ms,
                          color='#fb923c', alpha=.10, zorder=0)
            if len(samples):
                latest = np.searchsorted(stamps, sample_x, side='right') - 1
                active = latest >= 0
                error.plot(x_ms(sample_x[active]), target[active] - q[latest[active]], '.-', color='#7c3aed',
                           markersize=4, label='紫线  控制器目标 − FluxVLA 最近发送目标')
        if len(samples):
            angle.plot(x_ms(sample_x), target, '.-', color='#7c3aed', markersize=5, linewidth=2.5,
                       label=f'紫线  控制器回读目标（{source}）')
            angle.plot(x_ms(sample_x), actual, 'o-', color='#059669', markersize=4,
                       markerfacecolor='white', linewidth=1, label=f'绿线  实际关节角（{source}）')
            error.plot(x_ms(sample_x), actual - target, 'o-', color='#059669', markersize=3,
                       markerfacecolor='white', label='绿线  实际关节角 − 控制器目标')
        for stamp, color, label in ((reached, '#059669', '绿点线  停止前到位确认'),
                                    (stop, '#b45309', '棕实线  收到停止回复')):
            if stamp is not None and origin <= stamp <= end:
                angle.axvline(x_ms(stamp), color=color, linestyle=':' if stamp == reached else '-',
                              linewidth=1, label=label)
        angle.set(title=f'同一关节的目标和反馈 · 首包后的 {detail_ms:.0f} ms', ylabel='角度（°）')
        error.axhline(0, color='#64748b', linewidth=.8)
        error.set(title='两段误差 · 越接近零越一致', ylabel='角度差（°）')
        for panel in (angle, error):
            panel.set(xlim=(-35, detail_ms), xlabel='相对第一个实际 UDP 包（ms）')
            if panel.get_legend_handles_labels()[0]:
                panel.legend(loc='best', ncol=2)
        for panel in (packets, angle, error):
            panel.grid(alpha=.23)
        fig.savefig(directory / 'detail.png', dpi=175)
        plt.close(fig)


def draw(directory, chunks, feedback, controller, first_cycle, count, joint, rows, tolerance, clock_error):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    chosen = list(chunks).index(first_cycle)
    groups = list(chunks.items())[chosen:chosen + count]
    axis = joint
    origin = first(groups[0][1], 'inference_start').get('started_s', rows[0]['fx_plan'])
    y = [r['point'] for r in rows]
    ms = lambda stamp: (stamp - origin) * 1000
    fig, (points_plot, overview) = plt.subplots(2, 1, figsize=(14, 9), layout='constrained')
    points_plot.set(title=f'Chunk {first_cycle + 1}: event times per policy point',
                    xlabel='ms from first model request', ylabel='Policy point (1-based)')
    for key, label, marker, color in (
            ('fx_plan', 'FluxVLA schedule', '|', '#64748b'),
            ('fx_send_1', 'UDP send 1', '.', '#2563eb'),
            ('fx_send_2', 'UDP send 2', '.', '#60a5fa'),
            ('udp_rx_1', 'Controller UDP ingress 1', 'x', '#b45309'),
            ('udp_rx_2', 'Controller UDP ingress 2', 'x', '#f59e0b'),
            ('controller_near', 'Internal target near waypoint', 's', '#7c3aed'),
            ('actual_near', 'Actual position near waypoint', 'o', '#059669')):
        matching = [(ms(r[key]), r['point']) for r in rows if r[key] is not None]
        if matching:
            x, ids = zip(*matching)
            points_plot.scatter(x, ids, marker=marker, color=color, label=label, s=25)
    after_read = model_ready(groups[0][1])
    server_pack = first(groups[0][1], 'inference_end').get('server_response_pack_started_s')
    if server_pack is not None:
        points_plot.axvline(ms(server_pack), color='#4338ca', linestyle=':', label='OpenPI response pack start')
    if after_read is not None:
        event = first(groups[0][1], 'inference_end')
        label = 'Complete response read' if event.get('response_received_s') else 'Response read and decoded'
        points_plot.axvline(ms(after_read), color='#0f766e', linestyle='--', label=label)
    points_plot.set_yticks(y)
    points_plot.legend(loc='best', ncol=2)

    earliest = origin
    last = groups[-1][1]
    latest = max(event['monotonic_s'] for values in last.values() for event in values)
    arm = 1 if joint < 6 else 0
    overview.set(title=f'Chunks {first_cycle + 1}–{groups[-1][0] + 1} | ee_id={arm}, J{joint % 6 + 1}',
                 xlabel='ms from first model request', ylabel='Joint angle (degrees)')
    for index, (cycle, group) in enumerate(groups):
        label = index == 0
        sends = [(ms(sent_stamp(p)), joint_row(p['target'])[axis]) for p in group['sent']
                 if sent_stamp(p) is not None]
        if sends:
            x, q = zip(*sends)
            overview.step(x, q, where='post', color='#2563eb',
                          label='FluxVLA targets' if label else None, zorder=3)
            entry = first(group, 'inference_start').get('started_s')
            ready = model_ready(group)
            if entry is not None and ready is not None:
                overview.axvspan(ms(entry), ms(ready), color='#38bdf8', alpha=.19,
                                 label='Wait for OpenPI' if label else None)
            stop = first(group, 'stop').get('monotonic_s')
            if stop is not None:
                overview.axvspan(x[-1], ms(stop), color='#fb923c', alpha=.18,
                                 label='Tail wait / stop' if label else None)
                overview.axvline(ms(stop), color='#b45309', linewidth=.7,
                                 label='Follower stop reply' if label else None)
            arrived = first(group, 'target_arrival_confirmed').get('monotonic_s')
            if arrived is not None:
                overview.axvline(ms(arrived), color='#059669', linewidth=.7, linestyle=':',
                                 label='Pre-stop arrival' if label else None)
            overview.annotate(str(cycle + 1), (x[0], q[0]), xytext=(4, 7), textcoords='offset points')
    samples = joint_feedback(feedback, controller, axis, earliest, latest)
    suffix = ' (GET receive time)' if controller is None else ' (.rlog)'
    if samples:
        x, targets, actual = zip(*samples)
        overview.plot([ms(t) for t in x], [np.nan if q is None else q for q in targets],
                      color='#7c3aed', alpha=.85, label='Controller target' + suffix,
                      marker='.' if controller is None else None, markersize=2)
        overview.plot([ms(t) for t in x], actual, color='#059669', label='Actual angle' + suffix,
                      marker='.' if controller is None else None, markersize=2)
    overview.legend(loc='best', ncol=2)
    for plot in (points_plot, overview):
        plot.grid(alpha=.22)
    available = 'High-rate controller state loaded' if controller is not None else 'No .rlog: point execution times left blank'
    clock_label = '' if clock_error is None else f' | cross-host clock ±{clock_error*1000:g} ms'
    fig.suptitle(f'A10 inference timing\n{available} | proximity {tolerance:g}°{clock_label}')
    fig.savefig(directory / 'timeline.png', dpi=145)
    plt.close(fig)
    return origin


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path, help='Inference log directory or events.jsonl')
    parser.add_argument('--cycle', type=int, default=1, help='Visible chunk number (1-based)')
    parser.add_argument('--count', type=int, default=4, help='Number of chunks in the overview')
    parser.add_argument('--detail-ms', type=float, default=1500., help='Zoom window after the first UDP send')
    parser.add_argument('--detail-points', type=int, default=6, help='Number of initial waypoints in the packet close-up')
    parser.add_argument('--ee-id', type=int, choices=(0, 1), default=0)
    parser.add_argument('--joint', type=int, choices=range(1, 7), default=4)
    parser.add_argument('--tolerance-deg', type=float, default=.5)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--udp-pcap', type=Path)
    parser.add_argument('--udp-port', type=int, default=9999)
    parser.add_argument('--max-udp-transit-ms', type=float, default=10.)
    parser.add_argument('--rlog-csv', type=Path)
    parser.add_argument('--rlog-inspect', type=Path)
    parser.add_argument('--rlog-axes', type=int, nargs=12, metavar='AXIS',
                        help='Zero-based recorder axis columns, ee_id 1 J1–J6 then ee_id 0 J1–J6')
    parser.add_argument('--controller-offset-ms', type=float,
                        help='Controller wall time minus FluxVLA wall time, in ms')
    parser.add_argument('--clock-error-ms', type=float)
    args = parser.parse_args()
    positive = (args.count, args.detail_ms, args.detail_points, args.tolerance_deg, args.max_udp_transit_ms)
    if not np.isfinite(positive).all() or min(positive) <= 0:
        parser.error('counts, detail window, tolerance and max UDP transit must be positive')
    rlog_args = (args.rlog_csv, args.rlog_inspect, args.rlog_axes)
    if any(rlog_args) and (not all(rlog_args) or len(set(args.rlog_axes)) != 12 or min(args.rlog_axes) < 0):
        parser.error('provide CSV, inspect JSON and twelve distinct nonnegative recorder axis indices')
    remote = args.udp_pcap or args.rlog_csv
    if remote and (args.controller_offset_ms is None or args.clock_error_ms is None
                   or not np.isfinite([args.controller_offset_ms, args.clock_error_ms]).all()
                   or args.clock_error_ms < 0):
        parser.error('controller data requires finite controller offset and nonnegative clock error')
    offset = (args.controller_offset_ms or 0.) / 1000
    clock_error = (args.clock_error_ms or 0.) / 1000
    try:
        path = args.run / 'events.jsonl' if args.run.is_dir() else args.run
        chunks, feedback, epoch_delta = read_client(path)
        first_cycle = args.cycle - 1
        if first_cycle not in chunks:
            raise ValueError(f'Chunk {args.cycle} has no sent packets')
        capture = read_capture(args.udp_pcap, args.udp_port, offset, epoch_delta) if args.udp_pcap else None
        controller = read_controller(*rlog_args, offset, epoch_delta) if args.rlog_csv else None
        group = chunks[first_cycle]
        end_event = first(group, 'stop') or first(group, 'chunk_complete')
        valid_end = end_event.get('monotonic_s', group['sent'][-1]['monotonic_s'])
        rows = point_rows(group, valid_end, capture, controller, clock_error,
                          args.tolerance_deg, args.max_udp_transit_ms / 1000)
        joint = (1 - args.ee_id) * 6 + args.joint - 1
        args.output.mkdir(parents=True, exist_ok=True)
        origin = draw(args.output, chunks, feedback, controller, first_cycle, args.count,
                      joint, rows, args.tolerance_deg, clock_error if remote else None)
        next_group = next((g for cycle, g in chunks.items() if cycle > first_cycle), {})
        next_chunk_start = first(next_group, 'execution_start').get('started_s')
        draw_detail(args.output, first_cycle, group, feedback, controller, joint, rows,
                    args.detail_ms, args.detail_points, next_chunk_start)
        duration_fields = ('send_lag_ms', 'udp_ingress_delay_ms', 'send_to_internal_ms', 'send_to_actual_ms')
        for row in rows:
            row['response_read'] = model_ready(group)
            row['server_response_pack_started'] = first(group, 'inference_end').get('server_response_pack_started_s')
            clocks = [('fx_plan', 'fx_send_1'), ('fx_send_1', 'udp_rx_1'),
                      ('fx_send_1', 'controller_near'), ('fx_send_1', 'actual_near')]
            durations = [round((row[b] - row[a]) * 1000, 3) if row[a] is not None and row[b] is not None else None
                         for a, b in clocks]
            for k, v in row.items():
                if k != 'point' and isinstance(v, (int, float)):
                    row[k] = round((v - origin) * 1000, 3)
            row.update(zip(duration_fields, durations))
        columns = {k: k if k in duration_fields or k == 'point' or 'status' in k else k + '_ms' for k in rows[0]}
        with (args.output / 'points.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(columns.values()))
            writer.writeheader()
            writer.writerows({columns[k]: v for k, v in row.items()} for row in rows)
    except (OSError, ValueError, KeyError) as exc:
        parser.exit(1, f'{exc}\n')
    print(args.output / 'timeline.png')
    print(args.output / 'detail.png')
    print(args.output / 'points.csv')


if __name__ == '__main__':
    main()
