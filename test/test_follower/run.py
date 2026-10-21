#!/usr/bin/env python3
"""Run a measured A10 step + action-chunk experiment, then render its curves."""
import argparse
import json
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import numpy as np
from scripts.a10_follower_frequency_test import StateSampler, degrees, run_phase, export_csv
if __package__:
    from .analyze import analyze
else:
    from analyze import analyze


def make_chunk(origin, joint, amplitude, steps):
    """Smooth single positive excursion and return; no model/GPU required."""
    targets = np.repeat(np.asarray(origin,dtype=float)[None],steps,axis=0)
    targets[:,joint-1] += amplitude * .5 * (1-np.cos(np.linspace(0,2*np.pi,steps)))
    return targets


def run_chunk(robot, sampler, log, origin, args):
    targets=make_chunk(origin,args.joint,args.delta_deg,args.chunk_steps*args.chunks)
    phase_start=time.monotonic()
    log.record('phase_start',phase='chunk',start_deg=degrees(sampler.latest().actual_joints_rad),
               target_deg=None,model_hz=args.model_hz,repeat=args.repeat,
               chunk_steps=args.chunk_steps,chunks=args.chunks,chunk_gap_s=args.chunk_gap)
    sent=0
    deadline=phase_start
    dt=1/(args.model_hz*args.repeat)

    def wait_until(stamp):
        while True:
            state=sampler.latest()
            q=degrees(state.actual_joints_rad)
            bounds=np.full(6,2.);bounds[args.joint-1]+=abs(args.delta_deg)
            if np.any(np.abs(np.asarray(q)-origin)>bounds):
                raise RuntimeError('Actual joints left the test excursion envelope')
            if log.dropped or log.write_errors:
                raise RuntimeError('Log lost events; stopping experiment')
            remaining=stamp-time.monotonic()
            if remaining<=0:return
            time.sleep(min(remaining,.005))

    for index,target in enumerate(targets):
        if index%args.chunk_steps==0:
            if index:
                deadline+=args.chunk_gap
                wait_until(deadline)
            log.record('chunk_boundary',chunk=index//args.chunk_steps,action_index=index)
        for repeat in range(args.repeat):
            wait_until(deadline)
            start=time.monotonic()
            if start-deadline>dt:
                raise TimeoutError('Missed chunk send deadline; no catch-up burst')
            try:
                robot.send_joints({'right':np.deg2rad(target)})
            except BaseException as exc:
                log.record('send_error',phase='chunk',action_index=index,repeat_index=repeat,
                           target_deg=target,send_started_s=start,error=repr(exc))
                raise
            finish=time.monotonic()
            log.record('send',phase='chunk',hz=args.model_hz*args.repeat,index=sent,
                       chunk=index//args.chunk_steps,action_index=index,repeat_index=repeat,
                       target_deg=target,send_started_s=start,send_finished_s=finish,
                       scheduled_s=deadline,late_s=start-deadline)
            sent+=1
            deadline+=dt
    wait_until(deadline+args.tail_s)
    log.record('phase_result',phase='chunk',sent=sent,model_steps=len(targets),
               elapsed_s=time.monotonic()-phase_start)


def main(argv=None):
    parser=argparse.ArgumentParser(description='A10 follower单目标+action chunk跟踪测试；会实际运动，结束自动画图')
    parser.add_argument('--mode',choices=['all','step','chunk'],default='all')
    parser.add_argument('--joint',type=int,choices=range(1,7),default=4)
    parser.add_argument('--delta-deg',type=float,default=5.)
    parser.add_argument('--model-hz',type=float,default=30.)
    parser.add_argument('--repeat',type=int,default=1,help='每个模型目标的发送次数；默认30Hz单发')
    parser.add_argument('--chunk-steps',type=int,default=30)
    parser.add_argument('--chunks',type=int,default=4)
    parser.add_argument('--chunk-gap',type=float,default=0.,help='块间不发送新目标的秒数，默认0；可模拟推理空档')
    parser.add_argument('--sample-hz',type=float,default=50.)
    parser.add_argument('--timeout',type=float,default=8.)
    parser.add_argument('--tolerance-deg',type=float,default=.2)
    parser.add_argument('--settle-s',type=float,default=.3)
    parser.add_argument('--tail-s',type=float,default=.5)
    parser.add_argument('--host',default='192.168.0.125')
    parser.add_argument('--ws-port',type=int,default=5999)
    parser.add_argument('--status-ws-port',type=int,default=5888)
    parser.add_argument('--udp-port',type=int,default=9998)
    parser.add_argument('--joint-udp-port',type=int,default=9999)
    parser.add_argument('--shared-port',type=int,default=5870)
    parser.add_argument('--log-dir',default=str(ROOT/'work_dirs/a10/follower_tests'))
    args=parser.parse_args(argv)
    positive=(args.model_hz,args.sample_hz,args.timeout,args.tolerance_deg,args.settle_s,args.tail_s)
    if not all(math.isfinite(x) and x>0 for x in positive):parser.error('Timing options must be positive and finite')
    if not math.isfinite(args.delta_deg) or not .5<=abs(args.delta_deg)<=10:parser.error('delta-deg magnitude must be 0.5..10')
    if args.repeat<1 or args.chunk_steps<2 or args.chunks<1:parser.error('Invalid repeat or chunk dimensions')
    if not 1<=args.model_hz*args.repeat<=240 or args.sample_hz>100:parser.error('Send rate must be 1..240Hz; sample rate <=100Hz')
    if not math.isfinite(args.chunk_gap) or args.chunk_gap<0 or args.tolerance_deg>=abs(args.delta_deg)/2:
        parser.error('Invalid chunk-gap or tolerance')

    from fluxvla.engines.operators.a10_operator import A10Operator
    from fluxvla.engines.communication.a10_shared_server import ConsoleSharedControl
    from fluxvla.engines.utils.a10_debug_log import A10DebugLog
    service=ConsoleSharedControl(('127.0.0.1',args.shared_port),host=args.host,
        ws_port=args.ws_port,status_ws_port=args.status_ws_port,
        udp_port=args.udp_port,joint_udp_port=args.joint_udp_port)
    options=dict(host=args.host,ws_port=args.ws_port,udp_port=args.udp_port,
        joint_udp_port=args.joint_udp_port,command_mode='joint',timeout_s=5.,
        shared_control=dict(host='127.0.0.1',port=args.shared_port,role='inference'))
    robot=A10Operator(**options,allow_motion=True)
    reader=A10Operator(**options,allow_motion=False)
    log=A10DebugLog(role='follower_tracking',root_dir=args.log_dir,image_every_n_cycles=0,
                    queue_size=4096,metadata=vars(args))
    sampler=None
    code=0
    error=None
    stop_confirmed=None
    try:
        print(f'日志：{log.path}',flush=True)
        service.start()
        robot.connect()
        state=robot.get_state()
        if state.actual_joints_rad is None:raise RuntimeError('No actual joint feedback')
        origin=np.asarray(degrees(state.actual_joints_rad))
        target=origin.copy();target[args.joint-1]+=args.delta_deg
        print(f'测试第{args.joint}关节：{origin[args.joint-1]:.3f}° → {target[args.joint-1]:.3f}°；'
              f'模式={args.mode}，模型{args.model_hz:g}Hz，发送{args.model_hz*args.repeat:g}Hz。',flush=True)
        robot.start_follower()
        reader.connect()
        sampler=StateSampler(reader,log,args.sample_hz)
        sampler.start()
        step_args=SimpleNamespace(**vars(args))
        def hold(name,q):
            return run_phase(robot,sampler,log,phase=name,hz=args.model_hz*args.repeat,
                             target_deg=q.tolist(),origin_deg=origin.tolist(),args=step_args)
        hold('baseline',origin)
        if args.mode in ('all','step'):
            hold('step',target)
            hold('step_return',origin)
        if args.mode in ('all','chunk'):
            run_chunk(robot,sampler,log,origin,args)
            hold('chunk_return',origin)
    except BaseException as exc:
        error=repr(exc);code=130 if isinstance(exc,KeyboardInterrupt) else 1
        log.record('test_error',error=error)
        print(f'测试中止：{error}；不自动回位。',file=sys.stderr)
    finally:
        try:
            if robot.follower_stop_pending:
                state=robot.stop_follower()
                stop_confirmed=state.follower_active is False or robot.last_stop_confirmed
                log.record('follower_stop',confirmed=stop_confirmed,raw=state.raw)
        except BaseException as exc:
            stop_confirmed=False;code=1
            log.record('follower_stop',confirmed=False,error=repr(exc))
            print(f'未确认停止：{exc}',file=sys.stderr)
        finally:
            if sampler is not None:sampler.close()
            for closer in (reader.close,robot.close,service.close):
                try:closer()
                except BaseException as exc:
                    code=1;log.record('cleanup_error',error=repr(exc))
            log.close()
            (log.path/'run.json').write_text(json.dumps(dict(completed=code==0,error=error,
                stop_confirmed=stop_confirmed,dropped_events=log.dropped,write_errors=log.write_errors),indent=2))
            export_csv(log.path)
            try:
                report=analyze(log.path,joint=args.joint,tolerance=args.tolerance_deg,settle_s=args.settle_s)
                for phase in report['phases']:
                    if phase['name'] in ('step','chunk'):
                        print(json.dumps(phase,ensure_ascii=False,indent=2),flush=True)
            except Exception as exc:
                print(f'绘图未完成，原始日志已保留：{exc}',file=sys.stderr)
                code=1
    return code


if __name__=='__main__':
    raise SystemExit(main())
