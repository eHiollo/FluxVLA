#!/usr/bin/env python3
"""Bounded 30Hz joint follower tail experiment; default is read-only.

Compare bounded follower target delivery and stop handoff.
Defaults to read-only. Recorded policy shape is limited to 0.5 degrees per axis.
"""
import argparse
import json
from pathlib import Path
import sys
import time
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def profile(origin, axis, delta):
    origin = np.asarray(origin, dtype=float)
    if (origin.shape != (12,) or not np.isfinite(origin).all()
            or type(axis) is not int or not 0 <= axis < 12
            or not np.isfinite(delta) or not 0 < abs(delta) <= .8):
        raise ValueError('Requires finite 12-axis origin and at most 0.8 degree displacement')
    rows = np.tile(origin, (20, 1))
    rows[:, axis] += np.linspace(0, delta, 20)
    return rows




def policy_offsets(log_dir, amplitude=.5):
    """Scale a recorded first policy chunk in policy ee_id order (1, 0).

    Returns degree offsets from its first target, never absolute poses.
    The entire chunk uses one scale, preserving relative axis/waypoint geometry.
    """
    if not np.isfinite(amplitude) or not 0 < amplitude <= .5:
        raise ValueError('Recorded policy replay displacement must be within 0..0.5 degrees')
    events = [json.loads(line) for line in (Path(log_dir)/'events.jsonl').read_text().splitlines()]
    observation = next(e for e in events if e['event'] == 'observation')
    cycle = observation['cycle']
    prediction = next(e for e in events if e['event'] == 'prediction' and e['cycle'] == cycle)
    joints = [0,1,2,3,4,5,7,8,9,10,11,12]
    origin = np.asarray(observation['state'], dtype=float)
    policy = np.asarray(prediction['converted_actions'], dtype=float)
    if (origin.shape != (14,) or policy.ndim != 2 or policy.shape[0] < 20
            or policy.shape[1] != 14 or not np.isfinite(origin).all()
            or not np.isfinite(policy).all()):
        raise ValueError('Recorded policy must contain finite 14-axis observation and twenty targets')
    delta = np.rad2deg(policy[:20, joints]-policy[0, joints])
    size = np.max(np.abs(delta))
    if size <= 1e-8:
        raise ValueError('Recorded policy has no measurable joint displacement')
    offsets = delta*min(1., amplitude/size)
    if np.max(abs(offsets)) > .500000001:
        raise ValueError('Recorded policy exceeds reviewed displacement')
    return offsets


def repeated_packets(rows, repeat, hold_s):
    if type(repeat) is not int or repeat not in (1, 2):
        raise ValueError('repeat must be 1 or 2')
    rows = np.asarray(rows, dtype=float)
    if rows.shape != (20, 12) or not np.isfinite(rows).all():
        raise ValueError('Requires twenty finite 12-axis rows')
    if not np.isfinite(hold_s) or not 0 <= hold_s <= .5:
        raise ValueError('Endpoint hold must be within 0..0.5 seconds')
    policy = np.repeat(rows, repeat, axis=0)
    return np.vstack((policy, np.tile(rows[-1], (round(30*repeat*hold_s), 1))))


def check_envelope(actual, internal, origin, target):
    lo, hi = np.minimum(origin, target)-.15, np.maximum(origin, target)+.15
    for q in (actual, internal):
        q = np.asarray(q)
        if q.shape != (12,) or not np.isfinite(q).all() or np.any(q < lo) or np.any(q > hi):
            raise RuntimeError('Actual/internal target outside bounded test envelope')



def observe_without_targets(read, duration, target, record, clock=time):
    """Only read feedback after the last packet; never send or stop here."""
    samples=[];first_arrival=None;stopped=False
    started=clock.monotonic()
    while clock.monotonic()-started < duration:
        state,q,ct=read(None)
        error=float(np.max(abs(q-target)))
        samples.append(dict(stamp_s=clock.monotonic(),error_deg=error,
            follower_active=state.follower_active,
            internal_error_deg=float(np.max(abs(ct-target))),
            actual_deg=q.tolist(),internal_deg=ct.tolist()))
        if first_arrival is None and error <= .02:
            first_arrival=clock.monotonic()-started
        record('active_no_send_feedback',**samples[-1])
        if state.follower_active is not True:
            stopped=True
            record('follower_stopped_without_request')
            break
        clock.sleep(1/30)
    return samples,first_arrival,stopped


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--repeat', type=int, choices=(1,2), default=1,
                        help='Repeat every 30Hz point; repeat=2 sends 60Hz without speeding up the motion')
    parser.add_argument('--source-log', type=Path, help='Use a recorded policy shape, scaled and translated to current pose')
    parser.add_argument('--amplitude', type=float, default=.5, help='Recorded policy maximum joint displacement, <=0.5 degrees')
    parser.add_argument('--holds', type=float, nargs='+', default=[0., .5],
                        help='Endpoint repetition durations, each within 0..0.5 seconds; at most two trials')
    parser.add_argument('--observe-active-s', type=float, nargs='+', default=[0.],
                        help='After packets cease, observe without stopping or sending; 0..3 seconds')
    args = parser.parse_args(argv)
    if not 1 <= len(args.holds) <= 2 or any(not np.isfinite(h) or not 0 <= h <= .5 for h in args.holds):
        parser.error('--holds requires one or two values within 0..0.5 seconds')
    if (len(args.observe_active_s) not in (1,len(args.holds))
            or any(not np.isfinite(s) or not 0 <= s <= 3 for s in args.observe_active_s)):
        parser.error('--observe-active-s requires one value or one per trial, each within 0..3 seconds')
    observe_times=args.observe_active_s*len(args.holds) if len(args.observe_active_s)==1 else args.observe_active_s
    offsets = policy_offsets(args.source_log, args.amplitude) if args.source_log else None
    from mmengine import Config
    from fluxvla.engines.operators.a10_operator import A10Operator
    from fluxvla.engines.utils.a10_debug_log import A10DebugLog
    cfg = dict(Config.fromfile('configs/a10/a10_dual_arm_operator.py')['operator'])
    cfg.pop('type');cfg.update(allow_motion=args.run, command_mode='joint',
                              joint_follower_filter=False, joint_follower_velocity_percent=4)
    cfg['shared_control'] = dict(cfg['shared_control'], role='console')
    robot = A10Operator(**cfg)
    names = cfg['joint_target_order']
    ids = [cfg['arms'][n]['ee_id'] for n in names]
    axis = ids.index(0)*6+3
    if offsets is not None:
        offsets = np.concatenate([offsets[:, (0 if ee == 1 else 6):(6 if ee == 1 else 12)] for ee in ids], axis=1)
    log = A10DebugLog(role='follower_tail_test', root_dir='work_dirs/a10/follower_tests',
                      image_every_n_cycles=0, metadata=dict(ee_id=('all12' if offsets is not None else 0),
                      joint=('all12' if offsets is not None else 4),
                      delta_deg=(args.amplitude if offsets is not None else .8),
                      observe_active_s=observe_times,
                      follower_percent=4,manual_percent=100,filter=False,send_hz=30*args.repeat,
                      point_repeat=args.repeat, source_log=str(args.source_log) if args.source_log else None,
                      policy_offsets_deg=offsets))
    report = dict(success=False, trials=[], log=str(log.path), motion_requested=args.run)
    reserved = False;original_speed = None;restore_pending = False
    origin = target = None
    guard_lo = guard_hi = None
    def read(expected_active=None):
        started=time.monotonic();st=robot.get_state();info=st.raw['ret_context']['robot_msg']
        actual=np.rad2deg(np.concatenate([st.arms[n].actual_joints_rad for n in names]))
        internal=np.rad2deg(np.concatenate([st.arms[n].target_joints_rad for n in names]))
        log.record('feedback', query_started_s=started, received_monotonic_s=st.received_monotonic_s,
                   actual_deg=actual, internal_deg=internal, robot=info, follower_active=st.follower_active)
        if (st.has_error or info.get('op_mode')!='Manual'
                or (args.run and info.get('activate')!='Enabled')
                or (expected_active is not None and st.follower_active is not expected_active)
                or time.monotonic()-st.received_monotonic_s > .15):
            raise RuntimeError('Required healthy manual follower permission lost')
        if origin is not None:check_envelope(actual,internal,guard_lo,guard_hi)
        return st,actual,internal
    try:
        robot.connect();st,actual,internal=read(False)
        report['before']=dict(robot=st.raw['ret_context']['robot_msg'],actual_deg=actual.tolist())
        motors=robot.read_motor_config();original_speed=robot.get_speed_override()
        report['motor_config']=motors;report['original_speed']=original_speed
        if len(motors)!=12:raise RuntimeError('Expected 12 motor configurations')
        if any(not (0 < m['max_vel'] <= 180.000001 and -180.000001 <= m['min_vel'] < 0
                        and 0 < m['max_acc'] <= 400.000001 and -400.000001 <= m['min_acc'] < 0)
               for m in motors):
            raise RuntimeError('Motor limits exceed the reviewed 180 deg/s, 400 deg/s2 envelope')
        if offsets is not None:
            report['preview']=dict(source_log=str(args.source_log),
                holds_s=args.holds, observe_active_s=observe_times,
                policy_offsets_deg=offsets.tolist(),
                max_displacement_deg=float(np.max(abs(offsets))),
                max_step_deg=float(np.max(abs(np.diff(offsets,axis=0)))),
                max_point_speed_deg_s=float(np.max(abs(np.diff(offsets,axis=0)))*30),
                anchored_targets_deg=(actual[None,:]+offsets).tolist())
        print('Log:',log.path,flush=True)
        print('PRECHECK',json.dumps(report['before']),flush=True)
        if not args.run:
            report['success']=True
            print('Read-only; --run is required for the bounded motion test.',flush=True)
            return 0
        if (st.raw['ret_context']['robot_msg'].get('motion')!='Stopped'
                or st.raw['ret_context']['program_msg']['state']!='Unloaded'
                or np.max(abs(actual-internal))>.03
                or not robot._connection.lease('status').get('available')):
            raise RuntimeError('Requires stopped/unloaded/idle follower with aligned feedback')
        robot.begin_shared_console_operation();reserved=True
        st,actual,internal=read(False)
        if st.raw['ret_context']['robot_msg'].get('motion')!='Stopped':
            raise RuntimeError('Robot moved before reservation')
        restore_pending=True
        if original_speed['percent']!=100:
            robot.set_speed_override(100,expected_mode='Manual',expected_percent=original_speed['percent'])
        for trial_index,hold_s in enumerate(args.holds):
            direction = 1  # Same shape and direction for comparable handoff trials.
            trial = f'trial_{trial_index+1}_hold_{hold_s:g}'
            st,base,ct=read(False)
            if st.raw['ret_context']['robot_msg'].get('motion')!='Stopped' or np.max(abs(base-ct))>.03:
                raise RuntimeError('Test baseline not settled')
            for _ in range(2):
                time.sleep(.04);_,q,_=read(False)
                if max(abs(q-base))>.03:raise RuntimeError('Baseline is moving')
            origin=base.copy()
            rows=(profile(origin,axis,.8*direction) if offsets is None
                  else origin[None,:]+direction*offsets)
            target=rows[-1]
            guard_lo=np.minimum(origin,rows.min(axis=0))
            guard_hi=np.maximum(origin,rows.max(axis=0))
            log.record('trial_profile',trial=trial,origin_deg=origin,targets_deg=rows,
                       max_displacement_deg=float(np.max(abs(rows-origin))))
            for row in rows:
                if any(not m['min_pos']+.2 <= q <= m['max_pos']-.2 for m,q in zip(motors,row)):
                    raise RuntimeError('Test exceeds motor position range')
            robot.start_follower('joint')
            start=time.monotonic();sent=[];wire=[]
            packets=repeated_packets(rows,args.repeat,hold_s)
            hz=30*args.repeat
            for i,row in enumerate(packets):
                deadline=start+i/hz;time.sleep(max(0,deadline-time.monotonic()))
                read(True)
                if time.monotonic()-deadline>.1:raise TimeoutError('Test send deadline missed')
                mapped={n:np.deg2rad(row[j*6:j*6+6]) for j,n in enumerate(names)}
                timing=robot.send_joints(mapped);sent.append(time.monotonic())
                wire.append(timing.get('udp_send_started_s') if isinstance(timing,dict) else None)
                log.record('sent',trial=trial,index=i+1,role='policy' if i<20*args.repeat else 'endpoint_hold',
                           policy_frame=i//args.repeat+1 if i<20*args.repeat else 20,
                           target_deg=row,scheduled_s=deadline,timing=timing)
            time.sleep(max(0,start+len(packets)/hz-time.monotonic()))
            observe_s=observe_times[trial_index]
            active_samples,first_active_arrival_s,stopped_without_request=observe_without_targets(
                read,observe_s,target,lambda event,**fields:log.record(event,trial=trial,**fields))
            stop_started=time.monotonic();st=robot.stop_follower()
            log.record('stop',trial=trial,started_s=stop_started,confirmed=st.follower_active is False)
            if st.follower_active is not False:raise RuntimeError('Stop unconfirmed')
            # Observe for 3s even if the final point was not reached. No new targets.
            samples=[];end=time.monotonic()+3
            while time.monotonic()<end:
                _,q,ct=read(False);samples.append((time.monotonic(),q,ct));time.sleep(1/30)
            q=samples[-1][1];ct=samples[-1][2];d=np.max(abs(rows-q),axis=1);k=int(np.argmin(d))
            ct_distance=np.max(abs(rows-ct),axis=1);ct_k=int(np.argmin(ct_distance))
            result=dict(trial=trial,packets=len(packets),hold_s=hold_s,
                        active_observe_s=observe_s,
                        stopped_without_request=stopped_without_request,
                        first_active_arrival_s=first_active_arrival_s,
                        active_last_feedback=active_samples[-1] if active_samples else None,
                        stop_requested_s=stop_started,
                        last_udp_send_s=wire[-1],
                        actual_udp_hz=(None if any(t is None for t in wire) else (len(wire)-1)/(wire[-1]-wire[0])),
                        terminal_internal_deg=ct.tolist(),
                        nearest_internal_policy_frame=ct_k+1,
                        nearest_internal_policy_frame_error_deg=float(ct_distance[ct_k]),
                        point_repeat=args.repeat,requested_hz=hz,
                        effective_hz=(len(sent)-1)/(sent[-1]-sent[0]),
                        terminal_max_error_deg=float(max(abs(q-target))),
                        internal_max_error_deg=float(max(abs(ct-target))),
                        nearest_policy_frame=k+1,nearest_policy_frame_error_deg=float(d[k]),
                        terminal_actual_deg=q.tolist(),target_deg=target.tolist())
            report['trials'].append(result);print(json.dumps(result),flush=True)
            log.record('trial_result',**result)
            origin=target=None
        report['success']=True
        return 0
    except BaseException as exc:
        report['error']=repr(exc);log.record('error',error=repr(exc));print('TEST ERROR:',repr(exc),flush=True)
        return 130 if isinstance(exc,KeyboardInterrupt) else 1
    finally:
        try:
            if robot.follower_stop_pending:robot.stop_follower()
            if restore_pending:
                st=robot.get_state()
                if (st.has_error or st.follower_active is not False
                        or st.raw['ret_context']['robot_msg'].get('motion')!='Stopped'):
                    raise RuntimeError('Stop not confirmed; speed restoration refused')
                now=robot.get_speed_override()
                if now['mode']!='Manual' or now['percent'] not in (100,original_speed['percent']):
                    raise RuntimeError('Speed changed externally; restoration refused')
                if now['percent']!=original_speed['percent']:
                    robot.set_speed_override(original_speed['percent'],expected_mode='Manual',expected_percent=now['percent'])
                report['speed_restored']=True
        except BaseException as exc:
            report['cleanup_error']=repr(exc);report['success']=False
            print('CLEANUP ERROR:',repr(exc),flush=True)
        finally:
            if reserved:robot.end_shared_console_operation()
            robot.close();log.record('result',**report);log.close()
            (log.path/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
            print('REPORT:',log.path/'report.json',flush=True)

if __name__=='__main__':
    raise SystemExit(main())
