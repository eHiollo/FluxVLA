"""Offline gripper replay on recorded forecast, packet and serial-read clocks.

No device construction or networking. Historical feedback/observations remain
fixed; candidate serial writes assume successful immediate acknowledgement.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from fluxvla.engines.utils.action_fusion import ExponentialActionFusion
from fluxvla.projects.a10.inference import GripperReversalGuard, gripper_target_raw


def significant_reversals(values, excursion=450):
    high = low = float(values[0])
    direction = count = 0
    for value in values[1:]:
        if not direction:
            if value-low >= excursion:
                direction, high = 1, value
            elif high-value >= excursion:
                direction, low = -1, value
            else:
                high, low = max(high, value), min(low, value)
        elif direction == 1:
            high = max(high, value)
            if high-value >= excursion:
                direction, low, count = -1, value, count+1
        else:
            low = min(low, value)
            if value-low >= excursion:
                direction, high, count = 1, value, count+1
    return count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    source = args.run/'events.jsonl'
    events = [json.loads(line) for line in source.read_text().splitlines()]
    config = json.loads((args.run/'config.json').read_text())
    records = {e['cycle']: e for e in events if e['event'] == 'action_fusion'}
    packets = [e for e in events if e['event'] == 'sent']
    groups = {}
    for packet in packets:
        groups.setdefault(packet['cycle'], []).append(packet)
    times = np.array([p['send_finished_s'] for p in packets])
    recorded = np.array([p['target'] for p in packets])
    boundary = np.diff([p['cycle'] for p in packets]) != 0
    dt = config['inference']['action_dt']

    def replay(alpha):
        fusion = ExponentialActionFusion(dict(config['inference']['action_fusion'], gripper_alpha=alpha))
        result = []
        for cycle, rows in groups.items():
            entry = records[cycle]
            plan, _ = fusion.fuse(entry['raw_targets'], start_s=entry['start_s'], dt=dt)
            result.extend(plan[e['frame']-1] for e in rows)
        return np.asarray(result)

    original = replay(None)
    reproduction = float(abs(original-recorded).max())
    if reproduction > 1e-12:
        raise ValueError(f'Historical reproduction differs: {reproduction}')

    def serial_replay(q, side, col, band, confirm):
        device = config['grippers'][side]
        guard = GripperReversalGuard(band, confirm)
        samples = [e['received_s'] for e in events
                   if e['event'] == 'gripper_feedback' and e['side'] == side
                   and times[0] <= e['received_s'] <= times[-1]]
        last = None
        curve, commands = [], []
        for now in samples:
            i = np.searchsorted(times, now, side='right')-1
            if now-times[i] >= .1:
                guard.pending_s = None
                curve.append(last if last is not None else np.nan)
                continue
            _, target = gripper_target_raw(q[i, col], device)
            changed = (last is None or target in (0, 9000)
                       or abs(target-last) > device.get('target_deadband_raw', 0))
            allowed = guard.allows(target, now) or packets[i].get('final', False)
            if changed and allowed:
                if target != last:
                    commands.append((now, target))
                last = target
                guard.acknowledge(target)
            curve.append(last if last is not None else np.nan)
        command_values = np.array([value for _, value in commands])
        metrics = dict(command_changes=len(commands),
                       total_travel_raw=float(abs(np.diff(command_values)).sum()),
                       significant_reversals=significant_reversals(command_values),
                       max_command_jump_raw=float(abs(np.diff(command_values)).max()))
        return dict(times=np.asarray(samples), values=np.asarray(curve), commands=commands, metrics=metrics)

    candidates, curves = [], {}
    for alpha, band, confirm in [(None, 0, 0), (.25, 0, 0), (.35, 0, 0),
                                  (.35, 450, .12), (.35, 450, .25), (.35, 450, .3)]:
        q = replay(alpha)
        joint_error = float(abs(q[:, [0,1,2,3,4,5,7,8,9,10,11,12]]
                                - recorded[:, [0,1,2,3,4,5,7,8,9,10,11,12]]).max())
        assert joint_error == 0.
        row = dict(gripper_alpha=alpha, reversal_deadband_raw=band, reversal_confirm_s=confirm,
                   arm_reference_max_difference_rad=joint_error)
        series = {}
        for side, col in [('left', 6), ('right', 13)]:
            data = serial_replay(q, side, col, band, confirm)
            delta = abs(np.diff(q[:, col])*9000)
            row[side] = dict(data['metrics'], max_packet_jump_raw=float(delta.max()),
                             max_block_jump_raw=float(delta[boundary].max()),
                             prediction_deviation_rms_raw=float(np.sqrt(np.mean(
                                 (q[:, col]-recorded[:, col])**2))*9000))
            series[side] = data
        candidates.append(row)
        curves[alpha, band, confirm] = series
    selected = next(c for c in candidates if c['gripper_alpha'] == .35 and c['reversal_confirm_s'] == .25)
    out = args.output or args.run/'gripper_fix'
    out.mkdir(parents=True, exist_ok=True)
    report = dict(source=str(source.resolve()), sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  packets=len(packets), blocks=len(groups), historical_reproduction_max_error=reproduction,
                  candidates=candidates, selected=selected,
                  limitations=['Fixed historical observations and original input/output clocks.',
                               'Serial candidate assumes immediate successful ACK at recorded read times.',
                               'Software targets only; no predicted physical feedback or task success.',
                               'Reversals require 450 raw excursion and 250ms sustained fresh input; '
                               'exact endpoints bypass amplitude only; explicit final task target bypasses confirmation.'])
    (out/'comparison.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    lines = ['# 夹爪反复开合修正', '', f'原始输入：`{source.resolve()}`',
             f'SHA256：`{report["sha256"]}`', '',
             '原始模型预测、发送时刻、串口反馈采样时刻固定；不连接机器人。',
             '保留连续开度、已有450 raw闭合补偿和实际反馈输入；手臂参考逐值不变。', '',
             '| 方案 | 左/右最大换块跳变 raw | 左/右反向次数 | 左/右目标累计行程 raw |',
             '| --- | ---: | ---: | ---: |']
    for c in candidates:
        l, r = c['left'], c['right']
        lines.append(f'| α={c["gripper_alpha"]}, 反向确认{c["reversal_confirm_s"]}s | '
                     f'{l["max_block_jump_raw"]:.0f} / {r["max_block_jump_raw"]:.0f} | '
                     f'{l["significant_reversals"]} / {r["significant_reversals"]} | '
                     f'{l["total_travel_raw"]:.0f} / {r["total_travel_raw"]:.0f} |')
    lines += ['', '反向次数按450 raw峰谷幅度统计；不是实机实际开合次数。累计行程为模拟串口目标变化量。',
              '选α=.35、450 raw反向幅度、250ms确认，拒绝短暂相反预测；持续反向时仍恢复模拟量跟随。',
              '代价：真实反向也要等待确认，另有融合动态响应；确认时间不是总响应延迟上界。',
              '精确全开/全闭仍需确认，避免每块末端重闭合绕过过滤；明确任务final在新鲜期内直接通过。',
              '过期目标不续命，不补发；停止/新任务清历史。串口写入延迟及真实接触力未建模，须实机验证。']
    (out/'结论.md').write_text('\n'.join(lines)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 1, figsize=(15, 7), sharex=True, layout='constrained')
    for ax, side, col in zip(axes, ['left', 'right'], [6, 13]):
        ax.plot(times-times[0], recorded[:, col]*9000, color='gray', alpha=.5, label='Recorded unfused target')
        for key, label in [((None,0,0), 'Serial baseline replay'), ((.35,450,.25), 'Fusion + reversal confirmation')]:
            data = curves[key][side]
            ax.step(data['times']-times[0], data['values'], where='post', label=label, linewidth=1)
        ax.set_ylabel(f'{side}: raw closure (0=open)')
        ax.grid(alpha=.25); ax.legend()
    axes[-1].set_xlabel('Original elapsed time (s)')
    fig.savefig(out/'comparison.png', dpi=150)
    plt.close(fig)
    print(json.dumps(dict(output=str(out), selected=selected), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
