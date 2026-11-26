"""Compare the saved run and a shared-clock counterfactual, without hardware.

Forecasts and their arrival times stay fixed. Transport/wakeup delays are
interpolated from this recording; future real transport timing is unknown.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from a10_action_fusion_replay import JOINTS, metrics
from fluxvla.engines.utils.action_fusion import ExponentialActionFusion
from fluxvla.engines.utils.chunk_execution import StreamReferenceClock
from fluxvla.engines.utils.trajectory_utils import resample_remaining


class ExperimentalPrefixFusion(ExponentialActionFusion):
    """Rejected candidate: enforce three old samples, taper residual to zero."""
    def __init__(self, config, steps):
        super().__init__(config)
        self.steps=steps

    def fuse(self, targets, *, start_s, dt):
        previous,old_start=self.plan,self.start_s
        plan,info=super().fuse(targets,start_s=start_s,dt=dt)
        if previous is not None:
            x=(start_s-old_start)/dt+np.arange(3)
            length=min(self.steps,len(plan))
            if x[-1] <= len(previous)-1 and length>=4:
                old=self._sample(previous,x)[:,self.joint_columns]
                u=np.linspace(0.,1.,length)
                basis=np.column_stack((u*(1-u)**3,u*u*(1-u)**3))
                correction=basis@np.linalg.solve(basis[1:3],old[1:3]-plan[1:3,self.joint_columns])
                plan[:length,self.joint_columns]+=correction
                plan[:3,self.joint_columns]=old
                self.plan=plan.copy()
        return plan,info


def analyze(directory):
    source=directory/'events.jsonl'
    events=[json.loads(line) for line in source.read_text().splitlines()]
    packets=[e for e in events if e['event']=='sent']
    config=json.loads((directory/'config.json').read_text())['inference']['action_fusion']
    dt=json.loads((directory/'config.json').read_text())['inference']['action_dt']
    records={e['cycle']:e for e in events if e['event']=='action_fusion'}
    predictions={e['cycle']:np.asarray(e['converted_actions']) for e in events if e['event']=='prediction'}
    starts=[e for e in events if e['event']=='execution_start']
    observed=np.array([e['target'] for e in packets])
    raw=np.array([e['unfused_target'] for e in packets])
    wire=np.array([e['udp_send_started_s'] for e in packets])
    refs=np.array([e['scheduled_s'] for e in packets])
    wake=np.maximum(0.,np.array([e['send_started_s'] for e in packets])-refs)
    transport=np.maximum(0.,wire-np.array([e['send_started_s'] for e in packets]))
    finish=np.maximum(0.,np.array([e['send_finished_s'] for e in packets])-wire)
    cycles=np.array([e['cycle'] for e in packets])
    boundary=np.diff(cycles)!=0
    deg_raw=np.rad2deg(raw[:,JOINTS])

    def evaluate(q,t,cc,rr):
        deg=np.rad2deg(q[:,JOINTS])
        reference=np.column_stack([np.interp(t,wire,col) for col in deg_raw.T])
        m=metrics(deg,t,np.diff(cc)!=0,reference)
        velocity=np.diff(deg,axis=0)/np.diff(t)[:,None]
        at=np.flatnonzero(np.diff(cc)!=0)
        at=at[(at>0)&(at<len(velocity)-1)]
        changes=np.max(abs(velocity[at+1]-velocity[at-1]),axis=1)
        m.update(packets=len(q),max_boundary_nearby_velocity_change_p95_deg_s=float(np.percentile(changes,95)),
                 reference_grid_phase_peak_ms=float(abs(((rr-rr[0]+dt/2)%dt)-dt/2).max()*1000),
                 min_wire_interval_ms=float(np.diff(t).min()*1000))
        return m

    # Fixed original publication clocks isolate trajectory-only changes.
    groups={}
    for packet in packets:groups.setdefault(packet['cycle'],[]).append(packet)
    fixed=[];baseline=None
    for steps in (0,6,10):
        f=ExperimentalPrefixFusion(config,steps) if steps else ExponentialActionFusion(config)
        points=[]
        for cycle,rows in groups.items():
            rec=records[cycle]
            plan,_=f.fuse(rec['raw_targets'],start_s=rec['start_s'],dt=dt)
            points.extend(plan[e['frame']-1] for e in rows)
        points=np.asarray(points)
        if steps==0:
            error=float(abs(points-observed).max())
            assert error<1e-12,(directory,error)
            baseline=points
        np.testing.assert_array_equal(points[:,[6,13]],baseline[:,[6,13]])
        fixed.append(dict(experimental_prefix_steps=steps,**evaluate(points,wire,cycles,refs)))

    def schedule(inject_delays):
        clock=StreamReferenceClock(dt)
        f=ExponentialActionFusion(dict(config,continuous_clock=True))
        points=[];times=[];labels=[];references=[];available=starts[0]['monotonic_s']
        skipped=0
        for k,entry in enumerate(starts):
            cycle=entry['cycle']
            ready=max(entry['monotonic_s'],available)
            start=clock.next_slot(ready)
            origin=records[cycle]['reference_time']
            aligned=resample_remaining(predictions[cycle],max(0.,(start-origin)/dt))
            if not len(aligned):continue
            plan,_=f.fuse(aligned,start_s=start,dt=dt)
            until=starts[k+1]['monotonic_s'] if k+1<len(starts) else wire[-1]+1e-6
            index=0
            while index<len(plan):
                deadline=start+index*dt
                wake_delay=float(np.interp(deadline,refs,wake)) if inject_delays else 0.
                send_entered=max(deadline+wake_delay,available)
                if times:send_entered=max(send_entered,times[-1]+dt/2)
                # A replacement cancels an old sleeping publisher. A send
                # already inside its RPC must finish before the new writer.
                if send_entered>=until:break
                advanced=min(len(plan)-1,max(index,int(np.floor((send_entered-start)/dt+1e-8))))
                skipped+=advanced-index;index=advanced;deadline=start+index*dt
                wire_delay=float(np.interp(deadline,refs,transport)) if inject_delays else 0.
                stamp=send_entered+wire_delay
                if stamp>wire[-1]:break
                clock.record(deadline)
                points.append(plan[index]);times.append(stamp);labels.append(cycle);references.append(deadline)
                post=float(np.interp(deadline,refs,finish)) if inject_delays else 0.
                available=stamp+post
                index+=1
        return (np.asarray(points),np.asarray(times),np.asarray(labels),np.asarray(references),skipped)

    variants={}
    arrays={}
    for inject,name in [(False,'ideal_shared_clock'),(True,'recorded_delay_shared_clock')]:
        q,t,cc,rr,skipped=schedule(inject)
        variants[name]=dict(evaluate(q,t,cc,rr),expired_reference_slots_skipped=skipped,
                             kind='counterfactual_fixed_observations_and_arrivals')
        arrays[name]=dict(q=q,t=t,cycles=cc,refs=rr)
    return dict(source=str(source.resolve()),sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                dt=dt,reference_reproduction_max_error=float(abs(baseline-observed).max()),
                observed=evaluate(observed,wire,cycles,refs),fixed_clock_trajectory_candidates=fixed,
                candidates=variants),dict(observed=dict(q=observed,t=wire,cycles=cycles,refs=refs),
                                         raw=raw,**arrays)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs',type=Path,nargs='+')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    reports=[]
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    if 'Noto Sans CJK SC' in {f.name for f in font_manager.fontManager.ttflist}:plt.rcParams['font.family']='Noto Sans CJK SC'
    elif 'Noto Sans CJK JP' in {f.name for f in font_manager.fontManager.ttflist}:plt.rcParams['font.family']='Noto Sans CJK JP'
    plt.rcParams['axes.unicode_minus']=False
    for directory in args.runs:
        report,curves=analyze(directory);reports.append(dict(run=directory.name,**report))
        out=args.output/directory.name;out.mkdir(exist_ok=True)
        observed=curves['observed'];t0=observed['t'][0]
        raw=np.rad2deg(curves['raw'][:,JOINTS])
        boundary=np.diff(observed['cycles'])!=0
        peak=np.flatnonzero(boundary)[np.argmax(np.max(abs(np.diff(raw,axis=0)),axis=1)[boundary])]+1
        center=observed['t'][peak]-t0
        window=(max(0,center-1),center+2)
        axis=int(np.argmax(abs(raw[peak]-raw[peak-1])))
        fig,axes=plt.subplots(3,1,figsize=(14,10),layout='constrained')
        for key,label,color in [('observed','已保存版本：实录发送目标','#167cbd'),
                                 ('recorded_delay_shared_clock','统一时钟：离线调度推演','#d76811')]:
            data=curves[key];q=np.rad2deg(data['q'][:,JOINTS]);t=data['t']-t0
            axes[0].plot(t,q[:,axis],color=color,label=label)
            v=np.diff(q[:,axis])/np.diff(t);vt=(t[1:]+t[:-1])/2
            axes[1].plot(vt,v,color=color,label=label)
            phase=((data['refs']-observed['refs'][0]+report['dt']/2)%report['dt']-report['dt']/2)*1000
            axes[2].plot(t,phase,color=color,label=label,linewidth=.9)
        for ax in axes:ax.grid(alpha=.2);ax.legend()
        for ax in axes[:2]:ax.set_xlim(*window)
        for row in (0,1):
            values=[]
            for key in ('observed','recorded_delay_shared_clock'):
                data=curves[key];t=data['t']-t0;q=np.rad2deg(data['q'][:,JOINTS])[:,axis]
                if row:
                    q=np.diff(q)/np.diff(t);t=(t[1:]+t[:-1])/2
                values.extend(q[(t>=window[0])&(t<=window[1])])
            lo,hi=min(values),max(values);pad=max(.01,(hi-lo)*.1)
            axes[row].set_ylim(lo-pad,hi+pad)
        axes[0].set_ylabel(f'物理{"左" if axis<6 else "右"}臂 J{axis%6+1}目标 / °')
        axes[1].set_ylabel('参考差分速度 / °/s')
        axes[2].set_ylabel('相对首个30Hz网格相位 / ms')
        axes[2].set_xlabel('首次实录发送后原始时间 / s；相位按一个周期折返显示')
        fig.suptitle(directory.name+'\n橙色是固定历史观测及插值通信延迟下的推演，尚非新版实机结果')
        fig.savefig(out/'comparison.png',dpi=150);plt.close(fig)
        np.savez_compressed(out/'curves.npz',**{key+'_'+name:arr for key,data in curves.items()
                           if isinstance(data,dict) for name,arr in data.items()})
    payload=dict(runs=reports,implementation_sha256={name:hashlib.sha256(Path(name).read_bytes()).hexdigest()
        for name in ['fluxvla/engines/operators/a10_dual_operator.py','fluxvla/engines/utils/chunk_execution.py',
                     'fluxvla/engines/utils/action_fusion.py',str(Path(__file__))]},limitations=[
        'Same historical model predictions and observation/inference timestamps; no curve shifts or slowed model clock.',
        'Fixed-clock trajectory comparisons reproduce the saved targets and preserve grippers exactly.',
        'New-clock packet counts/times can change. Delay interpolation is an assumption, not actual new hardware timing.',
        'No device connections, new measured motor feedback, settling or vibration conclusions.'])
    (args.output/'comparison.json').write_text(json.dumps(payload,ensure_ascii=False,indent=2)+'\n')
    lines=['# 跨块统一时钟优化','', '最终启用：连续30Hz参考网格；手臂和夹爪融合增益、偏置、反向确认保持已保存版本。',
           '已发参考时间与实际UDP发送时间分开记录，新块不再用上一包晚发时刻重置参考时钟。',
           '超期参考槽跳过，不追赶补发；严重超时仍停止。模型时间轴、物理映射及下游上限不变。',
           '为避免迟发后的突发，两次发送至少间隔半个控制周期；这不是额外播放缓冲。','',
           '| 输入 | 比较口径 | 目标jerk RMS | 10–15Hz速度波动 | 换块附近速度变化P95 | 目标偏移RMS |',
           '| --- | --- | ---: | ---: | ---: | ---: |']
    for r in reports:
        for label,m in [('已保存版本实录',r['observed']),('新时钟＋实录延迟插值推演',r['candidates']['recorded_delay_shared_clock'])]:
            lines.append(f'| {r["run"]} | {label} | {m["jerk_rms_deg_s3"]:.1f} | '
                f'{m["high_frequency_velocity_rms_deg_s"]:.4f} | '
                f'{m["max_boundary_nearby_velocity_change_p95_deg_s"]:.3f} | {m["correction_rms_deg"]:.3f}° |')
    lines+=['','速度变化P95：在每个换块处，用前一个块末段和新块下一段的差分速度差，取十二轴最大值后统计P95；包含正常加减速，不能等同于严格速度不连续量。',
            '强制前三点一致的轨迹接续候选只用于离线对照，整体jerk/高频波动上升，未放入执行代码。',
            '新版本没有声称严格C1/C2；30Hz采样、预测本身的加减速及真实调度抖动仍会保留残余波动。',
            '通信延迟与线程唤醒耗时从历史记录插值；真实新版运行前，这些改善数字只能作为推演证据。',
            '原始记录指纹、未采用候选、理想时钟诊断和详细指标保存在comparison.json。',
            '回退当前优化：inference.action_fusion.continuous_clock=False；夹爪设置不用改。']
    (args.output/'结论.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps([dict(run=r['run'],observed=r['observed'],selected=r['candidates']['recorded_delay_shared_clock']) for r in reports],ensure_ascii=False,indent=2))


if __name__=='__main__':main()
