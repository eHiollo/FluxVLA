#!/usr/bin/env python3
"""Export one recorded A10 prediction as a complete follower test sequence."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def export_actions(log_dir, cycle=1, output=None):
    source=Path(log_dir).resolve()
    manifest=json.loads((source/'metadata.json').read_text())
    with (source/'events.jsonl').open() as stream:
        rows=[json.loads(line) for line in stream]
    def event(name):
        return next((r for r in rows if r['event']==name and r.get('cycle')==cycle),None)
    planned,observation=event('planned_actions'),event('observation')
    if planned is None or observation is None:
        raise ValueError(f'No complete prediction/observation for cycle {cycle}; setup may have failed')
    actions=np.asarray(planned['actions'],dtype=np.float32)
    initial=np.asarray(observation['feedback']['qpos'],dtype=np.float32)
    if (actions.ndim!=2 or actions.shape[1]!=7 or not len(actions) or
            not np.isfinite(actions).all() or initial.shape!=(7,) or not np.isfinite(initial).all()):
        raise ValueError('Expected finite absolute actions[T,7] and observed qpos[7]')
    metadata=manifest['metadata']
    dt=float(metadata['action_dt'])
    if not np.isfinite(dt) or dt<=0:raise ValueError('Invalid model action_dt')
    output=Path(output) if output else source/'exported_actions'/f'cycle_{cycle:04d}'
    output.mkdir(parents=True,exist_ok=False)
    joint_rad=actions[:,:6].copy()
    joint_deg=np.rad2deg(joint_rad).astype(np.float64)
    np.save(output/'actions_rad_mm.npy',actions,allow_pickle=False)
    np.save(output/'joints_rad.npy',joint_rad,allow_pickle=False)
    np.save(output/'joints_deg.npy',joint_deg,allow_pickle=False)
    with (output/'actions.csv').open('w',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(['step','time_s',*[f'q{i}_rad' for i in range(1,7)],
                         *[f'q{i}_deg' for i in range(1,7)],'gripper_mm'])
        for i,(action,degrees) in enumerate(zip(actions,joint_deg)):
            writer.writerow([i,i*dt,*action[:6],*degrees,float(action[6])])
    prediction=event('model_output')
    profile=(prediction or {}).get('remote_profile') or {}
    before=event('denormalized_actions_before_clamp')
    if before:
        np.save(output/'actions_before_gripper_clamp_rad_mm.npy',np.asarray(before['actions'],dtype=np.float32),allow_pickle=False)
    information=dict(source_log=str(source),cycle=cycle,source_run_id=manifest['run_id'],
        observation_utc=observation['utc'],initial_qpos_rad_mm=initial.tolist(),
        initial_joints_deg=np.rad2deg(initial[:6]).tolist(),
        instruction=metadata.get('instruction'),model=profile.get('server_debug'),
        steps=len(actions),model_hz=1/dt,action_dt_s=dt,duration_s=len(actions)*dt,
        semantics='Absolute joint targets, NOT deltas. First sample at t=0; each target holds for action_dt_s.',
        columns='6 joint positions followed by gripper opening; radians and millimetres',
        full_prediction=True,selected_steps_in_original_run=len(planned.get('selected_actions',[])),
        dry_run=metadata.get('dry_run'),
        motion_sends_in_log=any(bool(r.get('trajectory',{}).get('joint_packets_sent'))
                                for r in rows if isinstance(r.get('trajectory'),dict)),
        image_files={k:str(source/v) for k,v in observation.get('image_files',{}).items()})
    (output/'metadata.json').write_text(json.dumps(information,ensure_ascii=False,indent=2))
    print(f'Exported {len(actions)} targets at {1/dt:g}Hz: {output.resolve()}')
    return output


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log_dir')
    parser.add_argument('--cycle',type=int,default=1)
    parser.add_argument('--output')
    args=parser.parse_args()
    export_actions(args.log_dir,cycle=args.cycle,output=args.output)
