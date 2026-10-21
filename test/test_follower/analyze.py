#!/usr/bin/env python3
"""Plot follower experiments or existing A10 inference/frequency JSONL logs."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def load_signals(directory):
    with (Path(directory) / 'events.jsonl').open() as stream:
        rows = [json.loads(line) for line in stream]
    sends, samples, phases, starts = [], {}, [], {}
    boundaries = []
    for row in rows:
        event = row['event']
        if event == 'send':
            sends.append((row['send_started_s'], row['target_deg']))
        if event == 'state' and row.get('actual_deg') is not None:
            samples[row['received_monotonic_s']] = row['actual_deg']
        if event == 'observation':
            f = row['feedback']
            if f.get('joints') is not None:
                samples[f['received_monotonic_s']] = np.rad2deg(f['joints']).tolist()
        if event == 'cycle' or (event == 'run_error' and row.get('trajectory')):
            trajectory = row.get('trajectory') or {}
            trace = trajectory.get('trace', [])
            if trace:
                boundaries.append(trace[0]['joint_send_started_s'])
            for packet in trace:
                if packet.get('joint_sent'):
                    sends.append((packet['joint_send_started_s'], np.rad2deg(packet['target_joints_rad']).tolist()))
            for f in [x.get('feedback') for x in trace] + trajectory.get('feedback_checks', []):
                if f and f.get('actual_joints_rad') is not None:
                    samples[f['received_monotonic_s']] = np.rad2deg(f['actual_joints_rad']).tolist()
        if event == 'phase_start':
            starts[row['phase']] = row
        if event == 'chunk_boundary':
            boundaries.append(row['monotonic_s'])
        if event == 'phase_result' and row['phase'] in starts:
            start = starts.pop(row['phase'])
            phases.append(dict(name=row['phase'], start=start['monotonic_s'], end=row['monotonic_s'],
                               start_deg=start.get('start_deg'), target_deg=start.get('target_deg'),
                               completed=True))
    # Interrupted phases remain visible and must not be reported as completed.
    end = max((row['monotonic_s'] for row in rows), default=0.)
    phases.extend(dict(name=name, start=r['monotonic_s'], end=end,
                       start_deg=r.get('start_deg'), target_deg=r.get('target_deg'), completed=False)
                  for name, r in starts.items())
    sends = sorted({float(t): q for t, q in sends}.items())
    samples = sorted(samples.items())
    if not sends or not samples:
        raise ValueError('No target sends or actual state samples available for plotting')
    st = np.array([x[0] for x in sends])
    sq = np.array([x[1] for x in sends], dtype=float)
    qt = np.array([x[0] for x in samples])
    qq = np.array([x[1] for x in samples], dtype=float)
    valid = (qt >= st[0]) & (qt <= end)
    qt, qq = qt[valid], qq[valid]
    if not len(qt):
        raise ValueError('No actual state samples after the first target')
    ref = sq[np.searchsorted(st, qt, side='right') - 1]
    return dict(send_t=st, target=sq, state_t=qt, actual=qq, reference=ref,
                phases=sorted(phases, key=lambda p: p['start']), boundaries=boundaries)


def error_metrics(error):
    a = np.asarray(error, dtype=float)
    if not len(a):
        return None
    return dict(samples=len(a), mae_deg=np.abs(a).mean(axis=0).tolist(),
                rmse_deg=np.sqrt(np.square(a).mean(axis=0)).tolist(),
                p95_abs_deg=np.percentile(np.abs(a), 95, axis=0).tolist(),
                max_abs_deg=np.abs(a).max(axis=0).tolist())


def step_metrics(t, actual, initial, target, tolerance=.2, settle_s=.3):
    """Times are observed threshold crossings; no claim of sub-sample precision."""
    t, actual = np.asarray(t), np.asarray(actual)
    delta = target - initial
    if not len(t) or abs(delta) <= 1e-9:
        return dict(valid=False)
    progress = (actual - initial) / delta
    def crossing(level):
        found = np.flatnonzero(progress >= level)
        return float(t[found[0]]) if len(found) else None
    reaction = max(.1, .02 * abs(delta)) / abs(delta)
    t10, t90 = crossing(.1), crossing(.9)
    settled = None
    for i in range(len(t)):
        j = np.searchsorted(t, t[i] + settle_s)
        if j < len(t) and np.all(np.abs(actual[i:j+1] - target) <= tolerance):
            settled = float(t[i])
            break
    tail = actual[t >= t[-1] - settle_s]
    return dict(valid=True, response_delay_s=crossing(reaction), t10_s=t10,
                t50_s=crossing(.5), t90_s=t90,
                rise_10_90_s=None if t10 is None or t90 is None else t90-t10,
                settling_time_s=settled, settling_tolerance_deg=tolerance,
                overshoot_deg=float(max(0., (np.max(progress) - 1.) * abs(delta))),
                tail_mean_abs_error_deg=float(np.mean(np.abs(tail - target))))


def estimate_delay(send_t, target, state_t, actual, joint, max_lag_s=.8):
    """Fit a pure delay using causal zero-order-held targets, fixed sample set."""
    lag_grid = np.arange(0., max_lag_s + .001, .005)
    mask = state_t >= send_t[0] + max_lag_s
    t, q = state_t[mask], actual[mask, joint]
    if len(t) < 10 or np.ptp(target[:, joint]) < .5 or np.ptp(q) < .2:
        return dict(valid=False, reason='insufficient_motion_or_samples')
    losses = []
    for lag in lag_grid:
        index = np.searchsorted(send_t, t - lag, side='right') - 1
        losses.append(float(np.mean((q - target[index, joint]) ** 2)))
    best = int(np.argmin(losses))
    return dict(valid=best < len(lag_grid)-1, estimated_delay_s=float(lag_grid[best]),
                at_search_limit=best == len(lag_grid)-1, search_max_s=max_lag_s,
                zero_delay_rmse_deg=float(np.sqrt(losses[0])),
                delay_compensated_rmse_deg=float(np.sqrt(losses[best])),
                note='Effective trajectory lag, including reply latency and follower dynamics; not pure network delay.')


def analyze(directory, joint=4, tolerance=.2, settle_s=.3, output=None):
    directory = Path(directory)
    output = Path(output) if output else directory / 'follower_report'
    output.mkdir(parents=True, exist_ok=True)
    data = load_signals(directory)
    st, target, qt, actual, ref = [data[k] for k in ('send_t', 'target', 'state_t', 'actual', 'reference')]
    t0 = st[0]
    error = actual - ref
    report = dict(source=str(directory.resolve()), joint=joint, units='degrees and seconds',
                  time_semantics='State timestamp is local reply receipt, target timestamp is local send start. Controller sampling timestamps unavailable.',
                  overall=error_metrics(error), phases=[], health=None)
    report['errors'] = [dict(event=r['event'],error=r.get('error'))
                       for r in _read_events(directory) if r['event'] in ('test_error','run_error','setup_error','cleanup_error')]
    health = directory / 'summary.json'
    if health.exists():
        report['health'] = json.loads(health.read_text())
    report['measurement_complete'] = bool(report['health'] and
        report['health'].get('dropped_events') == 0 and report['health'].get('write_errors') == 0)
    if (directory/'run.json').exists():
        report['run'] = json.loads((directory/'run.json').read_text())
    phases = data['phases'] or [dict(name='recorded_run',start=st[0],end=qt[-1],
                                    start_deg=None,target_deg=None,completed=None)]
    for phase in phases:
        sm = (st >= phase['start']) & (st <= phase['end'])
        qm = (qt >= phase['start']) & (qt <= phase['end'])
        if not np.any(sm) or not np.any(qm):
            continue
        ps, pq = st[sm], target[sm]
        ts, aq, reference = qt[qm], actual[qm], ref[qm]
        item = dict(name=phase['name'], completed=phase['completed'], errors=error_metrics(aq-reference))
        if phase['target_deg'] is not None and phase['start_deg'] is not None:
            item['step'] = step_metrics(ts-ps[0], aq[:,joint-1], phase['start_deg'][joint-1],
                                        phase['target_deg'][joint-1], tolerance, settle_s)
        if phase['name'] == 'chunk':
            item['delay'] = estimate_delay(ps, pq, ts, aq, joint-1)
        report['phases'].append(item)
        fig, ax = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
        plot_t,plot_q=ps-ps[0],pq[:,joint-1]
        if phase['target_deg'] is not None and phase['start_deg'] is not None:
            plot_t=np.r_[-.05,plot_t];plot_q=np.r_[phase['start_deg'][joint-1],plot_q]
        ax[0].step(plot_t, plot_q, where='post', label='Sent target', color='#dc7b17')
        ax[0].plot(ts-ps[0], aq[:,joint-1], '.-', label='Actual feedback', markersize=3, color='#176ba0')
        ax[0].set_ylabel(f'Joint {joint} position (deg)')
        ax[0].set_title(f'{phase["name"]} | {directory.name}')
        ax[0].legend()
        ax[1].plot(ts-ps[0], aq[:,joint-1]-reference[:,joint-1], color='#a43243')
        ax[1].axhspan(-tolerance,tolerance,alpha=.12,color='green', label=f'+/-{tolerance:g} deg')
        ax[1].set(xlabel='Time from first phase target (s)', ylabel='Actual - sent target (deg)')
        for key in ('response_delay_s','t90_s','settling_time_s'):
            value = item.get('step',{}).get(key)
            if value is not None:
                ax[0].axvline(value,ls='--',alpha=.5,label=f'{key}: {value:.3f}s')
        if item.get('step'):
            ax[0].legend(fontsize=8)
        for boundary in data['boundaries']:
            if ps[0] < boundary < ts[-1]:
                for a in ax:a.axvline(boundary-ps[0],color='gray',alpha=.25,lw=.8)
        for a in ax:a.grid(alpha=.2)
        fig.tight_layout()
        fig.savefig(output / f'{phase["name"]}.png',dpi=150)
        fig.savefig(output / f'{phase["name"]}.svg')
        plt.close(fig)
    fig, axes = plt.subplots(6,2,figsize=(15,15),sharex=True)
    for j in range(6):
        axes[j,0].step(st-t0,target[:,j],where='post',color='#dc7b17',label='Sent target',lw=1)
        axes[j,0].plot(qt-t0,actual[:,j],'.-',color='#176ba0',label='Actual feedback',lw=1,markersize=2)
        axes[j,0].set_ylabel(f'J{j+1} (deg)')
        axes[j,1].plot(qt-t0,error[:,j],color='#a43243',lw=1)
        axes[j,1].axhline(0,color='gray',lw=.5)
        axes[j,1].set_ylabel(f'J{j+1} error (deg)')
        for a in axes[j]:a.grid(alpha=.2)
    axes[0,0].legend();axes[0,0].set_title('Sent targets vs actual feedback')
    axes[0,1].set_title('Error against latest sent target (causal alignment)')
    for a in axes[-1]:a.set_xlabel('Time since first target (s)')
    fig.suptitle(directory.name,fontsize=11)
    fig.tight_layout()
    fig.savefig(output/'overview.png',dpi=150)
    fig.savefig(output/'overview.svg')
    plt.close(fig)
    with (output/'aligned.csv').open('w',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(['t_s',*[f'target_j{i+1}_deg' for i in range(6)],
                         *[f'actual_j{i+1}_deg' for i in range(6)],*[f'error_j{i+1}_deg' for i in range(6)]])
        for t,r,q,e in zip(qt,ref,actual,error):writer.writerow([t-t0,*r,*q,*e])
    (output/'metrics.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(f'Follower plots and metrics: {output.resolve()}')
    return report


def _read_events(directory):
    with (directory/'events.jsonl').open() as stream:
        return [json.loads(line) for line in stream]


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log_dir')
    parser.add_argument('--joint',type=int,choices=range(1,7),default=4)
    parser.add_argument('--output')
    args=parser.parse_args()
    analyze(args.log_dir,joint=args.joint,output=args.output)
