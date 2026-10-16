"""Regenerate golden results with the original C++ source, entirely locally.

python test/data/vr_control/generate_reference.py --source-root /path/to/A10_new
Use --json-include /path/to/vendor/rtb-1.0.0/include for a source-only snapshot.
No controller process, networking, or robot SDK is used.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

import numpy as np
from scipy.spatial.transform import Rotation

SOURCE_SHA256 = 'ee282317b9b7b361d8b39a049e63f71f9b62a21c5be6e5e9796b10ffa82b3464'


def matrix(xyz=(0., 0., 0.), rotvec=(0., 0., 0.)):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix()
    result[:3, 3] = xyz
    return result.ravel().tolist()


def command(anchor=1, seq=1, offset=(0.,)*6, active=True, session='test'):
    return dict(type='SET_EE_ANCHOR', session_id=session, anchor_id=anchor,
                sample_sequence=seq, offset=list(offset), active=active, gripper=.5)


def inputs():
    actual = matrix((.2, -.1, .3), (.2, -.3, .8))
    anchors = []
    for cmd in [command(), command(seq=2, offset=(.01, -.02, .03, .1, .2, -.3)),
                command(seq=2, offset=(.2, 0., 0., 0., 0., 0.)), command(seq=1),
                command(anchor=0, seq=9), command(seq=3, active=False),
                command(anchor=2, seq=4, offset=(.02, .01, 0., 0., .2, 0.)),
                command(session='next', offset=(0., 0., 0., .4, -.2, .1))]:
        anchors.append(dict(command=cmd, actual_pm=actual))
    cases = []
    # Nonidentity frame, changing targets, externally held reference, release.
    rows = [dict(operation='engage', actual_pm=actual)]
    for index in range(14):
        rows.append(dict(operation='step', actual_pm=actual,
                         target_pm=matrix((.23, -.12, .32), (.4, -.2, .9)),
                         dt_s=.002, hold=index in (3, 4)))
    rows += [dict(operation='release', actual_pm=actual),
             dict(operation='step', actual_pm=actual, target_pm=matrix(), dt_s=.002)]
    cases.append(dict(fault_cycles=3, rows=rows))
    # Reference/actual separation freezes and faults, then explicit re-engage.
    bad = matrix((.5, 0., 0.))
    rows = [dict(operation='engage', actual_pm=matrix())]
    rows += [dict(operation='step', actual_pm=bad, target_pm=matrix((.1, 0., 0.)), dt_s=.002)
             for _ in range(4)]
    rows += [dict(operation='engage', actual_pm=bad), dict(operation='fault', actual_pm=bad)]
    cases.append(dict(fault_cycles=3, rows=rows))
    # Small angles, near-pi mixed-sign axis, and rotation-only tracking fault.
    for rotvec in [(1e-13, 0., 0.), (2.221441462, -2.221441462, 0.), (.1, .2, -.3)]:
        cases.append(dict(fault_cycles=3, rows=[dict(operation='engage', actual_pm=matrix()),
            dict(operation='step', actual_pm=matrix(), target_pm=matrix(rotvec=rotvec), dt_s=.002),
            dict(operation='step', actual_pm=matrix(rotvec=(0., 0., .8)),
                 target_pm=matrix(rotvec=rotvec), dt_s=.002)]))
    velocities = [dict(current=[.01, -.02, .03], target=v, maximum=.12, maximum_delta=.0008)
                  for v in ([1., 2., -3.], [0., 0., 0.], [-.02, .01, .04])]
    return dict(anchors=anchors, governors=cases, velocities=velocities)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, required=True)
    parser.add_argument('--json-include', type=Path)
    args = parser.parse_args()
    source = args.source_root / 'a10_anchor_protocol.cpp'
    header = args.source_root / 'a10_anchor_protocol.hpp'
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    if digest != SOURCE_SHA256:
        raise SystemExit('C++ source differs from the reviewed controller 97feb31; review before updating golden data')
    directory = Path(__file__).resolve().parent
    include = args.json_include or args.source_root / 'vendor/rtb-1.0.0/include'
    data = inputs()
    with tempfile.TemporaryDirectory(prefix='vr_cpp_reference_') as temporary:
        binary = Path(temporary) / 'reference'
        subprocess.run(['g++', '-std=c++17', '-O2', '-I', str(args.source_root),
                        '-I', str(include), str(source), str(directory / 'reference.cpp'),
                        '-o', str(binary)], check=True, capture_output=True)
        result = subprocess.run([str(binary)], input=json.dumps(data), text=True,
                                capture_output=True, check=True)
    fixture = dict(provenance=dict(controller_commit='97feb3131e48964c092371b266ebb489d8235686',
                   source_file='a10_anchor_protocol.cpp', source_sha256=digest,
                   header_sha256=hashlib.sha256(header.read_bytes()).hexdigest(),
                   generator='Original C++ source + offline harness; no robot SDK'),
                   inputs=data, expected=json.loads(result.stdout))
    output = directory / 'reference.json'
    output.write_text(json.dumps(fixture, ensure_ascii=False, indent=2) + '\n')
    print(f'Wrote {output}')


if __name__ == '__main__':
    main()
