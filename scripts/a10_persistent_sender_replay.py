"""Offline counterfactual for a persistent publisher using recorded forecasts.

Original observation and forecast arrival clocks are retained. Wakeup/transport
delays are interpolated from the recording; new hardware timing is not known.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from a10_action_fusion_replay import JOINTS, metrics
from fluxvla.engines.utils.action_fusion import ExponentialActionFusion
from fluxvla.engines.utils.trajectory_utils import resample_remaining


def replay(directory, preparation_s=.002):
    source = directory/'events.jsonl'
    events = [json.loads(line) for line in source.read_text().splitlines()]
    sent = [e for e in events if e['event'] == 'sent']
    schedules = [e for e in events if e['event'] == 'chunk_schedule']
    predictions = {e['cycle']: np.asarray(e['converted_actions'])
                   for e in events if e['event'] == 'prediction'}
    config = json.loads((directory/'config.json').read_text())['inference']
    dt = config['action_dt']
    wire = np.array([e['udp_send_started_s'] for e in sent])
    refs = np.array([e['scheduled_s'] for e in sent])
    wake = np.maximum(0., np.array([e['send_started_s'] for e in sent])-refs)
    delay = np.maximum(0., wire-np.array([e['send_started_s'] for e in sent]))
    after = np.maximum(0., np.array([e['send_finished_s'] for e in sent])-wire)
    observed = np.array([e['target'] for e in sent])
    raw = np.array([e['unfused_target'] for e in sent])
    cycles = np.array([e['cycle'] for e in sent])
    fusion = ExponentialActionFusion(dict(config['action_fusion'], persistent_sender=True))
    arrival = np.array([e['monotonic_s'] for e in schedules])
    available = refs[0]
    deadline = refs[0]
    adopted = -1
    plan = None
    start = None
    points, stamps, labels, reference_stamps = [], [], [], []
    adoption = []
    while deadline <= wire[-1]:
        now = max(available, deadline+float(np.interp(deadline, refs, wake)))
        if stamps:
            now = max(now, stamps[-1]+dt/2)
        # Same skip rule as the real publisher; never drain overdue ticks.
        missed = max(0, int(np.floor((now-deadline)/dt+1e-8)))
        deadline += missed*dt
        latest = int(np.searchsorted(arrival, now, side='right')-1)
        if latest >= 0 and latest > adopted:
            message = schedules[latest]
            aligned = resample_remaining(predictions[message['cycle']],
                max(0., (deadline-message['action_timestamp'])/dt))
            if not len(aligned):
                raise ValueError('Recorded prediction expired in persistent replay')
            plan, info = fusion.fuse(aligned, start_s=deadline, dt=dt)
            start, adopted = deadline, latest
            now += preparation_s
            adoption.append(dict(cycle=message['cycle'], start_s=deadline,
                                 history_expired=info['history_expired']))
            missed = max(0, int(np.floor((now-deadline)/dt+1e-8)))
            deadline += missed*dt
        if plan is None:
            deadline += dt
            continue
        index = int(round((deadline-start)/dt))
        if index >= len(plan):
            # Actual absence of future data holds the last motor target.
            # It does not produce duplicate packets.
            deadline += dt
            continue
        stamp = now+float(np.interp(deadline, refs, delay))
        if stamp > wire[-1]:
            break
        points.append(plan[index])
        stamps.append(stamp)
        labels.append(schedules[adopted]['cycle'])
        reference_stamps.append(deadline)
        available = stamp+float(np.interp(deadline, refs, after))
        deadline += dt
    points = np.asarray(points)
    stamps = np.asarray(stamps)
    labels = np.asarray(labels)
    reference_stamps = np.asarray(reference_stamps)

    def evaluate(q, t, cc):
        q = np.rad2deg(q[:, JOINTS])
        reference = np.column_stack([np.interp(t, wire, col)
                                     for col in np.rad2deg(raw[:, JOINTS]).T])
        result = metrics(q, t, np.diff(cc) != 0, reference)
        intervals = np.diff(t)
        boundary = np.diff(cc) != 0
        result.update(packets=len(q), duration_s=float(t[-1]-t[0]),
            interval_median_ms=float(np.median(intervals)*1000),
            interval_max_ms=float(intervals.max()*1000),
            gaps_over_50ms=int(np.sum(intervals > .05)),
            boundary_gaps_over_50ms=int(np.sum((intervals > .05) & boundary)))
        return result

    report = dict(run=directory.name, source=str(source.resolve()),
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        assumptions=dict(preparation_s=preparation_s,
            wakeup_and_transport='linear interpolation from original host timings',
            model_forecasts='same recorded inputs; no new model inference'),
        observed=evaluate(observed, wire, cycles),
        persistent_counterfactual=evaluate(points, stamps, labels),
        adoptions=len(adoption), history_expired=sum(e['history_expired'] for e in adoption))
    arrays = dict(observed=observed, observed_t=wire, observed_cycles=cycles,
                  persistent=points, persistent_t=stamps, persistent_cycles=labels,
                  persistent_reference_t=reference_stamps, raw=raw)
    return report, arrays


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    fonts = {f.name for f in font_manager.fontManager.ttflist}
    for name in ('Noto Sans CJK SC', 'Noto Sans CJK JP'):
        if name in fonts:
            plt.rcParams['font.family'] = name
            break
    plt.rcParams['axes.unicode_minus'] = False
    reports = []
    for directory in args.runs:
        report, arrays = replay(directory)
        reports.append(report)
        out = args.output/directory.name
        out.mkdir(exist_ok=True)
        np.savez_compressed(out/'curves.npz', **arrays)
        t0 = arrays['observed_t'][0]
        gaps = np.diff(arrays['observed_t'])
        boundary = np.diff(arrays['observed_cycles']) != 0
        candidates = np.flatnonzero((gaps > .05) & boundary)
        if not len(candidates):
            candidates = [int(np.argmax(gaps))]
        # Prefer a gap during movement over a stationary idle window.
        motion = np.linalg.norm(np.diff(arrays['observed'][:, JOINTS], axis=0), axis=1)
        selected = max(candidates, key=lambda i: motion[i])
        center = arrays['observed_t'][selected+1]-t0
        axis = int(np.argmax(abs(arrays['observed'][selected+1, JOINTS]
                                 - arrays['observed'][selected, JOINTS])))
        window = (max(0., center-.6), center+.7)
        fig, axes = plt.subplots(3, 1, figsize=(14, 10), layout='constrained')
        for prefix, label, color in [('observed', '旧版实录', '#137cb8'),
                                     ('persistent', '常驻线程离线推演', '#da711b')]:
            t = arrays[prefix+'_t']-t0
            q = np.rad2deg(arrays[prefix][:, JOINTS])[:, axis]
            axes[0].plot(t, q, marker='.', markersize=3, linewidth=1.1, color=color, label=label)
            axes[1].plot(t[1:], np.diff(t)*1000, marker='.', markersize=2,
                         linewidth=.8, color=color, label=label)
            axes[2].plot((t[1:]+t[:-1])/2, np.diff(q)/np.diff(t),
                         color=color, linewidth=1.1, label=label)
        axes[1].axhline(1000/30, color='gray', linestyle='--', label='33.33ms')
        for ax in axes:
            ax.set_xlim(*window)
            ax.grid(alpha=.2)
            ax.legend()
            ax.axvspan(arrays['observed_t'][selected]-t0, center,
                       color='#d62728', alpha=.07)
        for row in (0, 2):
            displayed = []
            for prefix in ('observed', 'persistent'):
                t = arrays[prefix+'_t']-t0
                q = np.rad2deg(arrays[prefix][:, JOINTS])[:, axis]
                if row == 2:
                    q = np.diff(q)/np.diff(t)
                    t = (t[1:]+t[:-1])/2
                displayed.extend(q[(t >= window[0]) & (t <= window[1])])
            low, high = min(displayed), max(displayed)
            margin = max(.01, (high-low)*.12)
            axes[row].set_ylim(low-margin, high+margin)
        axes[0].set_ylabel(f'物理{"左" if axis<6 else "右"}臂 J{axis%6+1}参考 / °')
        axes[1].set_ylabel('实际/推演发包间隔 / ms')
        axes[2].set_ylabel('参考差分速度 / °/s')
        axes[2].set_xlabel('原始首次发送后时间 / s')
        fig.suptitle(directory.name+'\n保留历史预测与到达时刻；橙色为延迟插值及2ms规划耗时假设下的推演')
        fig.savefig(out/'comparison.png', dpi=150)
        plt.close(fig)
    payload = dict(runs=reports, implementation_sha256={name:hashlib.sha256(Path(name).read_bytes()).hexdigest()
        for name in ('fluxvla/engines/operators/a10_dual_operator.py',
                     'fluxvla/engines/utils/action_fusion.py', __file__)},
        limitations=['Fixed historical observations and forecasts, no time-axis shifts.',
                     'Interpolated old delays plus assumed 2ms per-adoption planning; no measured new hardware timing.',
                     'Reference derivatives do not establish real motor vibration or physical settling.',
                     'Gripper fusion parameters, bias and reversal guard unchanged; serial dynamics not simulated.'])
    (args.output/'comparison.json').write_text(json.dumps(payload, ensure_ascii=False, indent=2)+'\n')
    lines = ['# 常驻发送线程修正换块空档', '',
        '原实现：每次新预测取消旧线程，再重新等待全局下一槽。模型提交恰逢发送槽时，可能丢掉一个槽。',
        '本次：唯一发送线程拥有30Hz时钟；新预测写入容量1的邮箱，下一个控制槽取最新预测。',
        '普通换块不取消线程、不重新起钟；只有预测实际耗尽才保持末点并结束发布，续接时恢复。',
        '沿用原始模型时钟、融合参数、物理映射、运动上限及夹爪处理，不补发、不生成重复保持点。', '',
        '| 输入 | 旧版>50ms间隔 | 其中换块空档 | 常驻线程推演>50ms间隔 | 最大间隔：旧/推演 ms |',
        '| --- | ---: | ---: | ---: | ---: |']
    for report in reports:
        old, new = report['observed'], report['persistent_counterfactual']
        lines.append(f'| {report["run"]} | {old["gaps_over_50ms"]} | '
            f'{old["boundary_gaps_over_50ms"]} | {new["gaps_over_50ms"]} | '
            f'{old["interval_max_ms"]:.2f} / {new["interval_max_ms"]:.2f} |')
    lines += ['', '新曲线是离线推演：固定原始模型预测和提交时间，线程唤醒/通信延迟由旧日志插值，并给每次融合预留2ms。',
              '这些数字不能替代新版实机测量，也不保证操作系统/网络今后不会迟发。',
              '独立的真实线程＋虚拟时钟测试覆盖1000点、频繁换块、发送中连续更新、最后一包final、耗尽保持续接、停止清缓存及后台故障。',
              '完整输入指纹和指令指标保存在comparison.json，逐点曲线保存在各运行子目录的curves.npz。',
              '回退常驻线程：inference.action_fusion.persistent_sender=False；保留此前统一时钟与夹爪设置。']
    (args.output/'结论.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(reports, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
