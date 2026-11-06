"""Read the 2x40-frame synthetic dual fixture using standard LeRobot only.

--reader v3: environment with lerobot.datasets.
--reader v21: environment with lerobot.common.datasets.
Does not import OpenPI, load policy weights, train, or access hardware.
"""
import argparse
import json
import os
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reader', choices=('v3', 'v21'), required=True)
    parser.add_argument('--dataset', type=Path, required=True)
    args = parser.parse_args()
    root = args.dataset.resolve()
    if not (root/'SIMULATED_DATA.json').is_file():
        raise ValueError('This check requires the synthetic 2x40-frame dual fixture')
    info = json.loads((root/'meta/info.json').read_text())
    if info.get('a10_recording_schema') != 'a10_dual_v1':
        raise ValueError('Expected a10_dual_v1')
    os.environ['HF_HUB_OFFLINE'] = '1'
    os.environ['HF_DATASETS_OFFLINE'] = '1'
    os.environ['HF_LEROBOT_HOME'] = str(root.parent)
    if args.reader == 'v3':
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        cameras = ('base_0_rgb', 'left_wrist_0_rgb', 'right_wrist_0_rgb')
    else:
        from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
        cameras = ('top', 'left', 'right')
    dataset = LeRobotDataset(root.name, root=root, video_backend='pyav',
                            delta_timestamps={'observation.state': [i/30 for i in range(30)]})
    assert len(dataset) == 80
    for index in (0, 20, 39, 40, 60, 79):
        row = dataset[index]; ep, local = divmod(index, 40)
        expected = []
        for offset in range(30):
            i = min(local+offset,39)
            expected.append([*(np.arange(6)*.05+.1+ep*.5+i*.001), i/39,
                             *(-np.arange(6)*.03-.2-ep*.5-i*.002), 1-i/39])
        np.testing.assert_allclose(row['observation.state'], expected, atol=1e-6)
        np.testing.assert_allclose(row['action'], expected[0], atol=1e-6)
        assert tuple(row['action'].shape) == (14,)
        assert row['task'] == ('Pick up the bottle' if ep==0 else 'Put down the bottle')
        assert int(row['frame_index']) == local
        assert abs(float(row['timestamp']) - local/30) < 1e-5
        for c, camera in enumerate(cameras):
            image = np.asarray(row[f'observation.images.{camera}'])
            assert image.shape == (3,16,16)
            assert abs(float(image.mean())*255 - (30+80*c+10*ep)) < 4
    print(json.dumps(dict(reader=args.reader, schema='a10_dual_v1', frames=len(dataset),
                          checked_indices=[0,20,39,40,60,79], cameras=list(cameras),
                          state_window=[30,14], action=[14],
                          episode_boundary='clamped to last frame, never crosses episode',
                          policy_or_training_executed=False), ensure_ascii=False, indent=2))


if __name__ == '__main__': main()
