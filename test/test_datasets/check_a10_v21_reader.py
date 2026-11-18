"""Opt-in output check using an independently installed official v2.1 reader.

Run in a LeRobot v0.3.3 / datasets<=3.6.0 environment, not the v3 environment:
    python test/test_datasets/check_a10_v21_reader.py --dataset /path/to/export
No model, hardware, network access or dataset writes are needed.
"""
import argparse
import json
import os
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    args = parser.parse_args()
    os.environ['HF_HUB_OFFLINE'] = '1'
    os.environ['HF_DATASETS_OFFLINE'] = '1'
    import numpy as np
    import pyarrow.parquet as pq
    from lerobot.datasets.lerobot_dataset import LeRobotDataset, CODEBASE_VERSION
    if CODEBASE_VERSION != 'v2.1':
        raise ValueError('Use an official v2.1 reader in an isolated dependency environment')
    root = args.dataset.resolve()
    info = json.loads((root / 'meta/info.json').read_text())
    episodes = [json.loads(line) for line in (root / 'meta/episodes.jsonl').read_text().splitlines()]
    dataset = LeRobotDataset('local/a10-v21-check', root=root, video_backend='pyav',
                             delta_timestamps={'observation.state': [i/info['fps'] for i in range(3)]})
    assert len(dataset) == info['total_frames']
    keys = [key for key, feature in info['features'].items() if feature['dtype'] == 'video']
    checked = []
    offset = 0
    selected = {0, min(1, len(episodes)-1), len(episodes)-1}
    for number, episode in enumerate(episodes):
        if number in selected:
            path = root / info['data_path'].format(
                episode_index=episode['episode_index'], episode_chunk=episode['episode_index']//info['chunks_size'])
            expected = pq.read_table(path)
            for local in (0, episode['length']-1):
                index = offset + local
                actual = dataset[index]
                state = [expected['observation.state'][min(local+i, episode['length']-1)].as_py()
                         for i in range(3)]
                np.testing.assert_allclose(actual['observation.state'], state, atol=1e-6)
                np.testing.assert_allclose(actual['action'], expected['action'][local].as_py(), atol=1e-6)
                assert actual['task'] in episode['tasks']
                assert int(actual['episode_index']) == episode['episode_index']
                for key in keys:
                    shape = info['features'][key]['shape']
                    assert tuple(actual[key].shape) == (shape[2], shape[0], shape[1])
                checked.append(index)
        offset += episode['length']
    print(json.dumps(dict(reader=CODEBASE_VERSION, frames=len(dataset), checked_indexes=checked,
                          cameras=keys), ensure_ascii=False))


if __name__ == '__main__':
    main()
