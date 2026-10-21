#!/usr/bin/env python3
"""Read one A10 log directory without connecting to hardware."""
import argparse
import json
from pathlib import Path


def _maximum(values):
    return max((abs(float(x)) for x in values), default=0.0)


def summarize(directory):
    directory = Path(directory)
    manifest = json.loads((directory / 'metadata.json').read_text())
    summary = dict(log_dir=str(directory.resolve()), role=manifest['role'],
                   run_id=manifest['run_id'], cycles=0, selected_steps=0,
                   stop_reasons=[], errors=[], max_cross_chunk_jump_rad=0.,
                   max_nominal_target_velocity_rad_s=0., max_nominal_target_acceleration_rad_s2=0.,
                   max_local_joint_send_gap_s=0., max_tracking_error_at_observation_rad=0.,
                   sampled_images=0, last_stop_confirmation=None)
    trajectories = {}
    with (directory / 'events.jsonl').open() as stream:
        for line in stream:
            row = json.loads(line)
            event = row['event']
            summary['sampled_images'] += len(row.get('image_files', {}))
            if event == 'model_output':
                info = (row.get('remote_profile') or {}).get('server_debug') or {}
                if info:
                    summary['model_server'] = info
            if event == 'planned_actions':
                summary['max_cross_chunk_jump_rad'] = max(summary['max_cross_chunk_jump_rad'], _maximum(row.get('cross_chunk_target_jump_rad') or []))
                for source, target in (('nominal_target_velocity_rad_s', 'max_nominal_target_velocity_rad_s'),
                                       ('nominal_target_acceleration_rad_s2', 'max_nominal_target_acceleration_rad_s2')):
                    summary[target] = max(summary[target], _maximum(x for step in row.get(source, []) for x in step))
            if event == 'cycle':
                summary['cycles'] += 1
                summary['selected_steps'] = row['result']['steps']
                summary['max_tracking_error_at_observation_rad'] = max(summary['max_tracking_error_at_observation_rad'], _maximum(row['result'].get('tracking_error_at_observation_rad') or []))
                if row.get('trajectory'):
                    trajectories[row['cycle']] = row['trajectory']
            if event == 'run_end':
                summary['stop_reasons'].append(row['reason'])
            if event in ('run_error', 'setup_error', 'server_error'):
                summary['errors'].append(dict(event=event, cycle=row.get('cycle'),
                                               phase=row.get('phase'), error=row['error']))
                if row.get('trajectory') and row.get('trajectory_cycle') is not None:
                    trajectories[row['trajectory_cycle']] = row['trajectory']
            if event == 'follower_stop':
                summary['last_stop_confirmation'] = row['confirmed']
    trace = [row for trajectory in trajectories.values() for row in trajectory.get('trace', [])]
    summary['local_joint_send_successes'] = sum(bool(row['joint_sent']) for row in trace)
    summary['max_local_joint_send_gap_s'] = max((row.get('joint_send_gap_s') or 0. for row in trace), default=0.)
    health = directory / 'summary.json'
    summary['log_health'] = json.loads(health.read_text()) if health.exists() else 'not cleanly closed or still running'
    if manifest['role'] == 'server':
        summary['model'] = manifest['metadata'].get('model')
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log_dir', help='A timestamped client/server log directory')
    args = parser.parse_args()
    print(json.dumps(summarize(args.log_dir), ensure_ascii=False, indent=2))
