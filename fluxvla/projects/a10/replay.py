"""Offline A10 observations -> FluxVLA service; never opens robot TCP."""

from __future__ import annotations

import argparse
import json
import os
from datetime import datetime
from pathlib import Path

import numpy as np

from fluxvla.datasets.parquet_dataset_v3 import ParquetDatasetV3
from .config import DEFAULT_CONFIG, DEFAULT_SITE, load_project
from .transport import PolicyClient


def _dataset(root: str) -> ParquetDatasetV3:
    return ParquetDatasetV3(
        data_root_path=root,
        action_key='observation.state',
        action_window_size=1,
        window_start_idx=1,
        statistic_name='a10',
        transforms=[dict(
            type='ProcessParquetInputs',
            parquet_keys=['observation.state', 'timestamp', 'actions',
                          'info', 'stats', 'action_masks'],
            video_keys=['observation.images.base_0_rgb',
                        'observation.images.right_wrist_0_rgb'],
            name_mappings={'observation.state': ['states']},
            video_backend='pyav')])


def _select_indices(dataset: ParquetDatasetV3, count: int) -> list[int]:
    episode_index = dataset.dataset['episode_index']
    starts = [index for index in range(len(episode_index) - 1)
              if (index == 0 or episode_index[index] != episode_index[index - 1])
              and episode_index[index] == episode_index[index + 1]]
    if not starts:
        raise ValueError('Dataset contains no episode with two frames')
    locations = np.linspace(0, len(starts) - 1,
                            num=min(count, len(starts)), dtype=int)
    return [starts[int(position)] for position in locations]


def run(cfg: dict, count: int) -> Path:
    dataset = _dataset(cfg['dataset']['root'])
    client = PolicyClient(cfg['inference']['host'], cfg['inference']['port'],
                          cfg['inference']['timeout_s'])
    results = []
    try:
        client.ping()
        statistics = {'a10': dataset.stats[0]['stats']}
        for index in _select_indices(dataset, count):
            sample = dataset.__getitem__(index, statistics)
            images = sample['images']
            observation = {
                'qpos': np.asarray(sample['states'], dtype=np.float32),
                'base_0_rgb': np.asarray(images[0]).transpose(1, 2, 0),
                'right_wrist_0_rgb': np.asarray(images[1]).transpose(1, 2, 0),
                'task_description': sample['task_description'],
            }
            actions = np.asarray(client.predict(observation))
            if actions.ndim == 3 and actions.shape[0] == 1:
                actions = actions[0]
            if (actions.ndim != 2 or actions.shape[1] != 7
                    or not np.isfinite(actions).all()):
                raise ValueError(f'Invalid offline actions: {actions.shape}')
            results.append({'row': index,
                            'episode': int(dataset.dataset[index]['episode_index']),
                            'task': sample['task_description'],
                            'action_shape': list(actions.shape),
                            'first_action': actions[0].tolist()})
            print(f'row={index}: {actions.shape} finite actions', flush=True)
    finally:
        client.close()
    output = (Path(cfg['paths']['outputs']) / 'replays' /
              datetime.now().strftime('%Y%m%d_%H%M%S'))
    output.mkdir(parents=True, exist_ok=True)
    report = output / 'report.json'
    report.write_text(json.dumps({'dataset': cfg['dataset']['root'],
                                  'samples': results}, indent=2,
                                 ensure_ascii=False) + '\n', encoding='utf-8')
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--samples', type=int, default=3)
    args = parser.parse_args()
    if args.samples <= 0:
        raise ValueError('--samples must be positive')
    cfg = load_project(os.environ.get('FLUXVLA_A10_CONFIG', DEFAULT_CONFIG),
                       os.environ.get('FLUXVLA_A10_SITE', DEFAULT_SITE))
    print(f'离线回放报告：{run(cfg, args.samples)}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
