"""Compare position and velocity fusion on fixed, original A10 run clocks.

Offline only: no robot/device construction or network calls. Historical
observations stay fixed, so this does not predict closed-loop task success.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from a10_action_fusion_replay import JOINTS, NAMES, derivative, metrics
from fluxvla.engines.utils.action_fusion import ExponentialActionFusion, fusion_config
from fluxvla.engines.utils.trajectory_utils import resample_remaining

POSITION = dict(enabled=True, method='position', alpha=.25, handover_steps=6, tail_steps=12)
VELOCITY = dict(POSITION, method='velocity', position_gain=6., correction_tau_s=.15,
                correction_fraction=1., correction_floor_deg_s=3.,
                correction_max_deg_s=30., endpoint_steps=12)


def load_run(directory):
    source = directory/'events.jsonl'
    events = [json.loads(line) for line in source.read_text().splitlines()]
    sent = [e for e in events if e['event'] == 'sent']
    records = {e['cycle']:e for e in events if e['event'] == 'action_fusion'}
    predictions = {e['cycle']:e['converted_actions'] for e in events if e['event'] == 'prediction'}
    schedule = {e['cycle']:e for e in events if e['event'] == 'chunk_schedule'}
    starts = {e['cycle']:e['started_s'] for e in events if e['event'] == 'execution_start'}
    dt = json.loads((directory/'config.json').read_text())['inference']['action_dt']
    groups = {}
    for e in sent:
        groups.setdefault(e['cycle'], []).append(e)
    chunks = []
    for cycle, packets in groups.items():
        start = starts[cycle]
        raw = (np.asarray(records[cycle]['raw_targets']) if cycle in records
               else resample_remaining(np.asarray(predictions[cycle]),
                                       max(0., (start-schedule[cycle]['action_timestamp'])/dt)))
        chunks.append(dict(cycle=cycle, start_s=start, raw=raw,
                           indices=[p['frame']-1 for p in packets]))
    t = np.array([e.get('udp_send_started_s') or e['send_started_s'] for e in sent])
    q = np.rad2deg(np.array([e['target'] for e in sent])[:, JOINTS])
    raw_q = np.rad2deg(np.array([chunk['raw'][i] for chunk in chunks for i in chunk['indices']])[:, JOINTS])
    return dict(directory=directory, source=source, dt=dt, chunks=chunks, sent=sent,
                t=t, recorded=q, raw=raw_q, cycles=np.array([e['cycle'] for e in sent]),
                recorded_config=json.loads((directory/'config.json').read_text())['inference'].get('action_fusion'))


def replay(run, config):
    fusion = ExponentialActionFusion(config)
    points, plans = [], []
    for chunk in run['chunks']:
        plan, info = fusion.fuse(chunk['raw'], start_s=chunk['start_s'], dt=run['dt'])
        points.extend(plan[i] for i in chunk['indices'])
        plans.append(dict(plan=plan, info=info))
    return np.rad2deg(np.asarray(points)[:, JOINTS]), plans


def phase_metrics(run, q):
    """Use the SAME active cycles selected by the recorded trajectory."""
    selected, profiles, absolute = [], [], []
    for chunk in run['chunks']:
        mask = run['cycles'] == chunk['cycle']
        if mask.sum() < 7:
            continue
        recorded_speed = np.linalg.norm(np.diff(run['recorded'][mask][:7], axis=0)/run['dt'], axis=1)
        if recorded_speed.mean() <= 5.:
            continue
        speed = np.linalg.norm(np.diff(q[mask][:7], axis=0)/run['dt'], axis=1)
        profiles.append(speed/max(speed.mean(), 1e-12))
        absolute.append(speed)
        selected.append(chunk['cycle'])
    profiles, absolute = np.asarray(profiles), np.asarray(absolute)
    return dict(cycles=selected, count=len(selected), incoming_frames=list(range(2, 8)),
                normalized_median=np.median(profiles, axis=0).tolist(),
                normalized_p25=np.percentile(profiles, 25, axis=0).tolist(),
                normalized_p75=np.percentile(profiles, 75, axis=0).tolist(),
                absolute_median_deg_s=np.median(absolute, axis=0).tolist(),
                mean_speed_deg_s=float(absolute.mean()),
                middle_to_edges_ratio_median=float(np.median(
                    absolute[:, 1:4].mean(axis=1)/np.maximum(absolute[:, [0, 4, 5]].mean(axis=1), 1e-12))))


def drain_metrics(run, plans):
    """Construct the unsent remainder after Ctrl+C; never label it measured."""
    chunk = run['chunks'][-1]
    plan = np.rad2deg(plans[-1]['plan'][:, JOINTS])
    raw = np.rad2deg(chunk['raw'][:, JOINTS])
    # Include the final already-issued sample for the join, and a hold only
    # for offline finite differences. No extra packets are added by runtime.
    start = chunk['indices'][-1]
    extended = np.vstack([plan[start:], np.tile(plan[-1], (4, 1))])
    duration = (len(plan)-1-start)*run['dt']
    dynamics = []
    t = np.arange(len(extended))*run['dt']
    q = extended
    for _ in range(3):
        q, t = derivative(q, t)
        dynamics.append(float(abs(q).max()))
    direction = np.sign(raw[-1]-plan[start])
    beyond = np.maximum(0., (plan[start:]-raw[-1])*direction)
    return dict(kind='constructed_unsent_remainder_and_offline_hold', remainder_s=float(duration),
                remaining_reference_points=len(plan)-1-start,
                latest_endpoint_error_deg=float(abs(plan[-1]-raw[-1]).max()),
                beyond_endpoint_deg=float(beyond.max()),
                reference_velocity_peak_deg_s=dynamics[0], reference_acceleration_peak_deg_s2=dynamics[1],
                reference_jerk_peak_deg_s3=dynamics[2],
                endpoint_recovery_deg=float(np.rad2deg(plans[-1]['info'].get('endpoint_recovery_rad', 0.))))


def evaluate(run, q, plans=None):
    result = metrics(q, run['t'], np.diff(run['cycles']) != 0, run['raw'])
    result['phase'] = phase_metrics(run, q)
    if plans is not None:
        result['drain'] = drain_metrics(run, plans)
        result['max_endpoint_recovery_deg'] = float(np.rad2deg(max(
            p['info'].get('endpoint_recovery_rad', 0.) for p in plans)))
    return result


def synthetic_checks():
    """Small constructed cases, kept separate from real model recordings."""
    f = ExponentialActionFusion(VELOCITY)
    f.fuse(np.zeros((30, 14)), start_s=0., dt=1/30)
    raw = np.full((30, 14), .01)
    plan, _ = f.fuse(raw, start_s=.25, dt=1/30)
    step = dict(kind='constructed_stationary_position_step', height_deg=float(np.rad2deg(.01)),
                reference_overshoot_deg=float(np.rad2deg(max(0., plan[:, 0].max()-.01))),
                latest_endpoint_error_deg=float(np.rad2deg(abs(plan[-1, 0]-.01))))
    f = ExponentialActionFusion(VELOCITY)
    points = []
    for frame in range(0, 1000, 8):
        raw = np.zeros((30, 14))
        raw[:, 0] = .02+.1*np.arange(30)/30
        plan, _ = f.fuse(raw, start_s=frame/30, dt=1/30)
        points.extend(plan[:min(8, 1000-frame), 0])
    biased = dict(kind='constructed_repeated_biased_forecasts', reference_points=len(points),
                  duration_s=999/30, max_abs_reference_deg=float(np.rad2deg(abs(np.asarray(points)).max())),
                  final_reference_deg=float(np.rad2deg(points[-1])),
                  note='Every fresh forecast restarts at .02rad with .1rad/s; pure velocity integration would drift.')
    return dict(stationary_step=step, biased_thousand_points=biased)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--scan', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    results = []
    for directory in args.runs:
        run = load_run(directory)
        old, old_plans = replay(run, POSITION)
        new, new_plans = replay(run, VELOCITY)
        result = dict(run=directory.name, source=str(run['source'].resolve()),
                      sha256=hashlib.sha256(run['source'].read_bytes()).hexdigest(),
                      packets=len(run['t']), duration_s=float(run['t'][-1]-run['t'][0]),
                      recorded=evaluate(run, run['recorded']),
                      position_baseline=evaluate(run, old, old_plans), velocity=evaluate(run, new, new_plans))
        if (run['recorded_config'] or {}).get('enabled'):
            reproduction, _ = replay(run, run['recorded_config'])
            result['recorded_reproduction_max_deg'] = float(abs(reproduction-run['recorded']).max())
            if result['recorded_reproduction_max_deg'] > 1e-9:
                raise ValueError('Historical configuration did not reproduce recorded targets')
        if args.scan:
            result['scan'] = []
            for gain in (2., 4., 6., 8.):
                for tau in (.06, .1, .15):
                    config = dict(VELOCITY, position_gain=gain, correction_tau_s=tau)
                    q, plans = replay(run, config)
                    result['scan'].append(dict(config=fusion_config(config), metrics=evaluate(run, q, plans)))
        results.append(result)
        out = args.output/directory.name
        out.mkdir(exist_ok=True)
        np.savez_compressed(out/'curves.npz', time_s=run['t'], recorded_deg=run['recorded'],
                            position_deg=old, velocity_deg=new, raw_deg=run['raw'], cycles=run['cycles'])
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(3, 1, figsize=(13, 9), layout='constrained')
        for name, q, label in (('position_baseline', old, 'Position blend'), ('velocity', new, 'Velocity blend + position correction')):
            phase = result[name]['phase']
            axes[0].plot(phase['incoming_frames'], phase['normalized_median'], 'o-', label=label)
            axes[0].fill_between(phase['incoming_frames'], phase['normalized_p25'], phase['normalized_p75'], alpha=.15)
            v, vt = derivative(q, run['t'])
            axes[1].plot(vt-run['t'][0], np.linalg.norm(v, axis=1), linewidth=.8, label=label)
            axes[2].plot(run['t']-run['t'][0], np.sqrt(np.mean((q-run['raw'])**2, axis=1)), linewidth=.8, label=label)
        axes[0].set_ylabel('Within-cycle speed / cycle mean'); axes[0].set_xlabel('Incoming point in SAME active cycles')
        axes[1].set_ylabel('12-joint reference norm (deg/s)'); axes[1].set_xlabel('Original elapsed time (s)')
        axes[2].set_ylabel('Deviation from raw reference (deg RMS)'); axes[2].set_xlabel('Original elapsed time (s)')
        for ax in axes:
            ax.grid(alpha=.25); ax.legend()
        fig.savefig(out/'comparison.png', dpi=160); plt.close(fig)
        print(directory.name, json.dumps({name:{k:result[name][k] for k in (
            'step_peak_deg', 'jerk_rms_deg_s3', 'correction_rms_deg', 'correction_peak_deg')}
            for name in ('position_baseline', 'velocity')}), flush=True)
    report = dict(position_config=fusion_config(POSITION), velocity_config=fusion_config(VELOCITY), results=results,
                  synthetic=synthetic_checks(),
                  implementation_sha256=hashlib.sha256((Path(__file__).resolve().parents[1]/
                      'fluxvla/engines/utils/action_fusion.py').read_bytes()).hexdigest(),
                  caveats=['Fixed historical observations, not new closed-loop rollouts.',
                           'All velocities/dynamics are software references, not motor vibration.',
                           'Reference deviation is not measured robot tracking error.',
                           'Unsent tail after interruption is constructed, not measured settling.',
                           'No model-clock shift, extra buffering, packet retries or changed downstream limits.'])
    (args.output/'comparison.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    lines = ['# 速度融合与持续位置校正：离线对照', '',
             '新旧方法复用相同原始预测、发布时间与实际发送点数。曲线未平移，未改变模型时钟。', '',
             '旧方法：α=.25、入场6点、尾部12点的位置融合。新方法保留这组运动融合权重，'
             '改为融合速度并积分，再用持续状态修正绝对位置：增益6/s、校正时间常数150ms、'
             '校正目标速度上限min(30,max(3,|运动速度|))°/s，未来末尾12点恢复最新末点。', '',
             '| 原记录时间 | 方法 | 最大单步变化 ° | jerk RMS °/s³ | 高频速度RMS °/s | 目标偏移RMS ° | 块中段/两侧速度比 |',
             '| --- | --- | ---: | ---: | ---: | ---: | ---: |']
    for r in results:
        for name, label in (('position_baseline', '位置融合'), ('velocity', '速度融合＋位置校正')):
            m = r[name]
            lines.append(f'| {r["run"].split("_")[1]} | {label} | {m["step_peak_deg"]:.3f} | '
                         f'{m["jerk_rms_deg_s3"]:.1f} | {m["high_frequency_velocity_rms_deg_s"]:.3f} | '
                         f'{m["correction_rms_deg"]:.3f} | {m["phase"]["middle_to_edges_ratio_median"]:.3f} |')
    lines += ['', '速度比按同一组已记录活跃块计算：块内第3～5点速度模长均值 / 第2、6、7点均值，'
              '再取各块比值的中位数；匀速为1。原记录的速度低于5°/s或不足7点的块不参与该指标。'
              '这里的速度是12轴软件参考向量模长，不是电机/TCP速度或实机振动测量。', '',
              '位置校正把位置差的吸收从每次换块重启的权重曲线中分离出来。'
              '纯速度积分不能保证绝对位置准确，因此仍保留有限带宽的位置校正；新目标或换向仍可能要求减速。'
              '新方案不保证所有动作都匀速，也没有从离线记录测出实机跟随延后。', '',
              '目标偏移相对于同一时刻的未融合参考，不能称作实际跟踪误差。'
              '逐轴RMS/峰值、输入SHA256、所选参数及扫描保存在comparison.json中。'
              '输入日志已启用融合时，独立回放原配置还会校验与实录目标的一致性。', '',
              '## 中断后的构造收尾', '',
              '以下仅把最后一块尚未发出的预测按原30Hz参考时间轴排完，再在离线计算中保持末点4个周期。'
              '这些点没有发送到机器人，也不代表实机停稳时间；运行时未新增保持包。', '',
              '| 原记录时间 | 最新末点误差 ° | 收尾中超过末点 ° | 构造剩余时长 s |',
              '| --- | ---: | ---: | ---: |']
    for r in results:
        m = r['velocity']['drain']
        lines.append(f'| {r["run"].split("_")[1]} | {m["latest_endpoint_error_deg"]:.6f} | '
                     f'{m["beyond_endpoint_deg"]:.3f} | {m["remainder_s"]:.3f} |')
    step = report['synthetic']['stationary_step']
    lines += ['', f'另一个独立构造用例：静止参考阶跃{step["height_deg"]:.3f}°，软件参考过冲约'
              f'{step["reference_overshoot_deg"]:.3f}°，最新末点误差{step["latest_endpoint_error_deg"]:.6f}°。'
              '不能把末点精确解释为全程零过冲。另有1000点有偏预测用例检查纯速度积分可能产生的漂移，见JSON的synthetic字段。', '',
              '新预测暂时不来时，当前预测走完到最新末点并保持。'
              '若新块在收尾修正过程中到来，会从当时已应用的参考速度续接；历史耗尽后不外推旧速度。'
              '停止/重启清理位置、运动速度及位置校正状态。下游实际限速、限加速度和限jerk继续生效，'
              '本对照没有运行下游控制器，软件目标差分峰值不能作为物理限幅验收。', '',
              '## 实际使用', '',
              '`python scripts/inference.py --config configs/a10/a10_dual_inference.py --model openpi`', '',
              '保持OpenPI WS/8001、30Hz、K=0、每个输出点单发。夹爪维持最新目标与45 raw死区。'
              '复现旧位置融合可加 `--cfg-options inference.action_fusion.method=position`。'
              '本轮没有自动启动实机。']
    (args.output/'结论.md').write_text('\n'.join(lines)+'\n')


if __name__ == '__main__':
    main()
