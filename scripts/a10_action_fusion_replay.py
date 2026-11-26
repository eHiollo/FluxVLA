"""Offline, fixed-observation replay of A10 asynchronous prediction fusion.

No device imports/connections. Uses recorded prediction origins, first-send
slots and packet counts; never shifts curves or changes the model clock.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from fluxvla.engines.utils.action_fusion import ExponentialActionFusion
from fluxvla.engines.utils.trajectory_utils import resample_remaining

JOINTS = [0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12]
NAMES = [f'ee{ee}_j{j}' for ee in (1, 0) for j in range(1, 7)]


def derivative(q, t):
    return np.diff(q, axis=0) / np.diff(t)[:, None], (t[1:] + t[:-1]) / 2


def dynamics(q, t):
    result = []
    for _ in range(3):
        q, t = derivative(q, t)
        result.append(q)
    return result


def metrics(q, t, boundary, reference):
    delta = np.diff(q, axis=0)
    v, a, j = dynamics(q, t)
    # Uniform reconstruction solely for a spectral diagnostic, on the same
    # absolute time support for every candidate. Nyquist at 30Hz is 15Hz.
    uniform_t = np.arange(t[0], t[-1], 1/30)
    uniform_q = np.column_stack([np.interp(uniform_t, t, col) for col in q.T])
    uniform_v = np.diff(uniform_q, axis=0) * 30
    window = np.hanning(len(uniform_v))
    spectrum = np.fft.rfft((uniform_v-uniform_v.mean(axis=0))*window[:, None], axis=0)
    frequency = np.fft.rfftfreq(len(window), 1/30)
    power = np.abs(spectrum)**2
    power[1:-1] *= 2
    if len(window) % 2:
        power[-1] *= 2
    high = np.sqrt(power[frequency >= 10].sum(axis=0)/(len(window)*np.sum(window**2)))
    error = q-reference
    return dict(boundary_peak_deg=float(np.abs(delta[boundary]).max()),
                boundary_rms_deg=float(np.sqrt(np.mean(delta[boundary]**2))),
                step_peak_deg=float(np.abs(delta).max()),
                velocity_peak_deg_s=float(np.abs(v).max()),
                acceleration_peak_deg_s2=float(np.abs(a).max()),
                jerk_rms_deg_s3=float(np.sqrt(np.mean(j*j))),
                high_frequency_velocity_rms_deg_s=float(np.sqrt(np.mean(high*high))),
                correction_rms_deg=float(np.sqrt(np.mean(error*error))),
                correction_peak_deg=float(np.abs(error).max()),
                correction_per_axis={name:dict(rms_deg=float(np.sqrt(np.mean(error[:, i]**2))),
                                              peak_deg=float(np.abs(error[:, i]).max()))
                                     for i, name in enumerate(NAMES)})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--alpha', type=float, default=.25)
    parser.add_argument('--tail-steps', type=int, default=8)
    parser.add_argument('--handover-steps', type=int, default=0)
    parser.add_argument('--joint', type=int, choices=range(12), default=3)
    args = parser.parse_args()
    source = args.run/'events.jsonl'
    out = args.output or args.run/'fusion_scan'
    out.mkdir(parents=True, exist_ok=True)
    events = [json.loads(line) for line in source.read_text().splitlines()]
    predictions = {e['cycle']:np.asarray(e['converted_actions']) for e in events if e['event']=='prediction'}
    schedules = {e['cycle']:e for e in events if e['event']=='chunk_schedule'}
    starts = {e['cycle']:e['started_s'] for e in events if e['event']=='execution_start'}
    recorded_fusion = {e['cycle']:e for e in events if e['event']=='action_fusion'}
    sent = [e for e in events if e['event']=='sent']
    t = np.array([e.get('udp_send_started_s') or e['send_started_s'] for e in sent])
    original = np.rad2deg(np.array([e['target'] for e in sent])[:, JOINTS])
    boundary = np.diff([e['cycle'] for e in sent]) != 0
    config = json.loads((args.run/'config.json').read_text())
    dt = config['inference']['action_dt']
    groups = {}
    for e in sent:
        groups.setdefault(e['cycle'], []).append(e)

    def replay(alpha, tail, handover=0):
        fusion = ExponentialActionFusion(dict(enabled=True, alpha=alpha, tail_steps=tail,
                                              handover_steps=handover))
        result = []
        for cycle, packets in groups.items():
            start, origin = starts[cycle], schedules[cycle]['action_timestamp']
            raw = (recorded_fusion[cycle]['raw_targets'] if cycle in recorded_fusion
                   else resample_remaining(predictions[cycle], max(0., (start-origin)/dt)))
            plan, _ = fusion.fuse(raw, start_s=start, dt=dt)
            for packet in packets:
                index = packet['frame']-1
                if index >= len(plan):
                    raise ValueError('Replay cannot preserve the recorded packet count')
                result.append(plan[index])
        return np.rad2deg(np.asarray(result)[:, JOINTS])

    aligned = replay(1., 4)
    rows, curves = [], {}
    for alpha in (.1, .15, .2, .25, .35, .5, .75, 1.):
        for tail in sorted({2, 4, 6, 8, args.tail_steps}):
            q = replay(alpha, tail, args.handover_steps)
            m = metrics(q, t, boundary, aligned)
            rows.append(dict(alpha=alpha, tail_steps=tail, handover_steps=args.handover_steps, **m))
            curves[alpha, tail] = q
    chosen = replay(args.alpha, args.tail_steps, args.handover_steps)
    baseline = metrics(original, t, boundary, aligned)
    precise = metrics(aligned, t, boundary, aligned)
    chosen_metrics = metrics(chosen, t, boundary, aligned)
    report = dict(source=str(source.resolve()), sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  packets=len(sent), boundaries=int(boundary.sum()), dt=dt,
                  original=baseline, alignment_only=precise, candidates=rows,
                  selected=dict(alpha=args.alpha, tail_steps=args.tail_steps,
                                handover_steps=args.handover_steps, **chosen_metrics),
                  limitations=[
                      'Same recorded observations and inference outputs; closed-loop predictions will change after fusion.',
                      'All dynamics describe transmitted software references, not motor dynamics or vibration.',
                      'Corrections are deviations from the unfused reference, not robot tracking errors.',
                      'No controller simulation, no physical settling/overshoot or physical delay measurement.',
                      '30Hz references only resolve 10-15Hz for the high-frequency metric.',
                      'Packet times, counts and inference origins are fixed; no extra buffering or timeline shift.'])
    (out/'comparison.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
    fields = ['alpha', 'tail_steps', 'handover_steps'] + [k for k in chosen_metrics if k != 'correction_per_axis']
    with (out/'parameters.csv').open('w') as f:
        writer = csv.DictWriter(f, fields, extrasaction='ignore')
        writer.writeheader(); writer.writerows(rows)
    np.savez_compressed(out/'curves.npz', time_s=t, recorded_deg=original,
                        aligned_deg=aligned, fused_deg=chosen, joint_names=NAMES)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True, layout='constrained')
    axis = args.joint
    for q, label in ((original, 'Recorded targets'), (chosen, f'Fusion a={args.alpha}, tail={args.tail_steps}, handover={args.handover_steps}')):
        v, vt = derivative(q, t)
        axes[0].plot(t-t[0], q[:, axis], label=label, linewidth=1)
        axes[1].plot(vt-t[0], v[:, axis], label=label, linewidth=1)
    axes[2].plot(t-t[0], chosen-aligned, linewidth=.8)
    for ax in axes:
        ax.grid(alpha=.25)
        for i in np.flatnonzero(boundary)+1:
            ax.axvline(t[i]-t[0], alpha=.1, color='gray')
    axes[0].set_ylabel(f'{NAMES[axis]} reference (deg)'); axes[0].legend()
    axes[1].set_ylabel('Reference diff. velocity (deg/s)')
    axes[2].set_ylabel('All joint corrections (deg)'); axes[2].set_xlabel('Original elapsed time (s)')
    fig.savefig(out/'comparison.png', dpi=160); plt.close(fig)
    lines = ['# 指数融合离线回放', '', f'输入：`{source.resolve()}`',
             f'SHA256：`{report["sha256"]}`', '',
             f'{len(sent)}个已发送点、{int(boundary.sum())}次换块。所有候选复用原预测、发送时刻和点数。', '',
             '| 方案 | 最大换块跳变 ° | 全部点最大步长 ° | jerk RMS改善 | 10–15Hz速度波动改善 | 目标修正RMS ° |',
             '| --- | ---: | ---: | ---: | ---: | ---: |']
    comparisons = [('原始记录中的发送目标', baseline), ('时间对齐、关闭融合', precise)]
    comparisons.extend((f'α={r["alpha"]}, 尾部{r["tail_steps"]}点, 入场{r["handover_steps"]}点', r)
                       for r in rows if r['alpha'] in (.15, .25, .35, .5) and r['tail_steps'] == args.tail_steps)
    for label, m in comparisons:
        lines.append(f'| {label} | {m["boundary_peak_deg"]:.3f} | {m["step_peak_deg"]:.3f} | '
                     f'{100*(1-m["jerk_rms_deg_s3"]/baseline["jerk_rms_deg_s3"]):.1f}% | '
                     f'{100*(1-m["high_frequency_velocity_rms_deg_s"]/baseline["high_frequency_velocity_rms_deg_s"]):.1f}% | '
                     f'{m["correction_rms_deg"]:.3f} |')
    lines += ['', f'选用 α={args.alpha}、尾部{args.tail_steps}点、入场{args.handover_steps}点。相对同一时刻未融合参考的合并RMS修正为 '
              f'{chosen_metrics["correction_rms_deg"]:.3f}°，最大修正{chosen_metrics["correction_peak_deg"]:.3f}°。', '',
              '改善来自新旧预测的融合及权重过渡。若原始记录已启用融合，关闭融合那行也移除了原有融合，不能解读为仅改变对齐时刻。更低α使预测修正更慢，'
              '更长尾部过渡避免把换块跳变挪到旧预测结束处。这是平滑与新预测响应的取舍。', '',
              '关节按相同未来时刻融合，夹爪采用最新的时间对齐指令。无新增播放缓冲；尾部点数是未来计划内的权重过渡长度，'
              '不是额外等待时间。相同匀速预测不会因融合而延后，突变的新预测会被逐次吸收。', '',
              '限制：这些是软件目标的差分动态，不是电机速度、加速度或振动测量，也不是下游限幅结果。'
              '修正量不是跟踪误差；没有测出实机延后、过冲或停稳时间。30Hz记录的高频指标仅覆盖10–15Hz。'
              '固定历史观测回放不能替代闭环实机验收。原运行被主动中断，末端收敛由独立构造测试覆盖。', '',
              '逐轴修正RMS/峰值及全部候选见 [comparison.json](comparison.json)，参数表见 [parameters.csv](parameters.csv)，'
              '统一时间轴图见 [comparison.png](comparison.png)。']
    (out/'结论.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(dict(original=baseline, selected=report['selected']), indent=2))


if __name__ == '__main__':
    main()
