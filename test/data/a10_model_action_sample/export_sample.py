"""Export a real prediction and its executed chunk; offline, no robot connection."""
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / 'work_dirs/a10/inference_logs/20261123_171128_5b70c81a_dual_client'
OUT = Path(__file__).resolve().parent
CYCLE = 1  # Zero-based cycle index in the original log.


def main():
    events = []
    identity = None
    with (SOURCE / 'events.jsonl').open() as stream:
        for line in stream:
            event = json.loads(line)
            if event['event'] == 'connected':
                identity = event.get('model_identity')
            if event.get('cycle') == CYCLE:
                events.append(event)
    by_name = {event['event']: event for event in events}
    manifest = json.loads((SOURCE / 'metadata.json').read_text())
    dt = manifest['metadata']['settings']['action_dt']
    raw = np.array(by_name['prediction']['raw_actions'], dtype=np.float64)
    converted = np.array(by_name['prediction']['converted_actions'], dtype=np.float64)
    count = by_name['follower_tail']['policy_points']
    repeat = by_name['follower_tail']['point_repeat']
    selected = converted[:count]
    wire = np.array([event['target'] for event in events if event['event'] == 'sent'])
    initial = np.array(by_name['pre_execution_feedback']['state'])
    joints = [*range(6), *range(7, 13)]
    assert raw.shape == converted.shape == (30, 14)
    assert selected.shape == (20, 14) and wire.shape == (40, 14)
    assert np.isfinite(raw).all() and np.isfinite(converted).all()
    assert np.isfinite(initial).all() and initial.shape == (14,)
    assert np.array_equal(wire, np.repeat(selected, repeat, axis=0))
    assert np.array_equal(raw[:, joints], converted[:, joints])
    assert np.array_equal(converted[:, [6, 13]], np.clip(raw[:, [6, 13]], 0, 1))
    assert by_name['chunk_complete']['physical_arrival_confirmed']
    columns = [*[f'left_j{i}_rad' for i in range(1, 7)], 'left_gripper_closure',
               *[f'right_j{i}_rad' for i in range(1, 7)], 'right_gripper_closure']
    for name, array in [('model_output_raw', raw), ('actions_full', converted),
                        ('actions_executed', selected), ('wire_targets_60hz', wire),
                        ('initial_state', initial)]:
        np.save(OUT / f'{name}.npy', array, allow_pickle=False)
    for name, array, step in [('actions_full', converted, dt),
                               ('actions_executed', selected, dt),
                               ('wire_targets_60hz', wire, dt / repeat)]:
        with (OUT / f'{name}.csv').open('w', newline='') as stream:
            writer = csv.writer(stream, lineterminator='\n')
            writer.writerow(['step', 'time_s', *columns])
            writer.writerows([i, i * step, *row] for i, row in enumerate(array))
    degrees = np.rad2deg(selected[:, joints])
    with (OUT / 'joints_executed_deg.csv').open('w', newline='') as stream:
        writer = csv.writer(stream, lineterminator='\n')
        writer.writerow(['step', 'time_s', *[f'ee{ee}_j{j}_deg' for ee in (1, 0) for j in range(1, 7)]])
        writer.writerows([i, i * dt, *row] for i, row in enumerate(degrees))
    info = dict(
        source_log=str(SOURCE.relative_to(ROOT)), source_run_id=manifest['run_id'],
        source_events_sha256=hashlib.sha256((SOURCE / 'events.jsonl').read_bytes()).hexdigest(),
        cycle_zero_based=CYCLE, model_identity=identity,
        prediction_utc=by_name['prediction']['utc'],
        semantics='Absolute joint position targets in rad; grippers are closure fractions: 0=open, 1=closed.',
        columns=columns, physical_arm_mapping={'left': {'ee_id': 1}, 'right': {'ee_id': 0}},
        gripper_raw_conversion='round(clip(closure, 0, 1) * 9000); not millimetres',
        model_hz=1 / dt, action_dt_s=dt, full_prediction_steps=len(converted),
        full_prediction_nominal_duration_s=len(converted) * dt,
        executed_steps=count, executed_nominal_duration_s=count * dt,
        packet_hz=repeat / dt, point_repeat=repeat,
        time_semantics='CSV time_s is a nominal schedule starting at 0, not measured send time. Duration includes the final target hold interval.',
        initial_state_rad_closure=initial.tolist(),
        observation_state_rad_closure=by_name['observation']['state'],
        initial_joints_deg=np.rad2deg(initial[joints]).tolist(),
        original_run_arrival_confirmed=True,
        executed_joint_span_deg=np.ptp(degrees, axis=0).tolist(),
        max_initial_to_first_joint_change_deg=float(np.rad2deg(np.abs(selected[0, joints] - initial[joints])).max()),
        raw_model_output=raw.tolist(), actions=converted.tolist(),
        source_events_file='source_cycle_events.jsonl',
    )
    (OUT / 'sample.json').write_text(json.dumps(info, ensure_ascii=False, indent=2) + '\n')
    with (OUT / 'source_cycle_events.jsonl').open('w') as stream:
        for event in events:
            stream.write(json.dumps(event, ensure_ascii=False) + '\n')
    # Read back the exported files and verify the portable representations.
    for name, expected in [('actions_full', converted), ('actions_executed', selected),
                            ('wire_targets_60hz', wire)]:
        assert np.array_equal(np.load(OUT / f'{name}.npy'), expected)
        assert np.allclose(np.loadtxt(OUT / f'{name}.csv', delimiter=',', skiprows=1)[:, 2:], expected, rtol=0, atol=1e-14)
    assert np.array_equal(np.array(json.loads((OUT / 'sample.json').read_text())['actions']), converted)
    print(json.dumps(dict(output=str(OUT), cycle=CYCLE, shapes=[list(raw.shape), list(selected.shape), list(wire.shape)],
                          max_joint_span_deg=float(np.ptp(degrees, axis=0).max()),
                          first_target=selected[0].tolist(), initial_state=initial.tolist()), indent=2))


if __name__ == '__main__':
    main()
