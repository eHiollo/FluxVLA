#!/usr/bin/env python
"""Export labeled A10 episodes into disjoint, reindexed SARM splits.

Parquet data and metadata are copied; videos link to the untouched source.
Both splits use normalization and temporal priors from training episodes only.
"""

import argparse
import hashlib
import json
import os
import random
from pathlib import Path

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq


TASK = ('Pick up the water bottle, put it in the nearby box, close the box lid, '
        'and return to the initial position.')


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def prepare(source, labels_path, output_prefix, seed=42, val_count=6):
    source = Path(source).resolve()
    labels_path = Path(labels_path).resolve()
    output_prefix = Path(output_prefix).resolve()
    info = json.loads((source / 'meta/info.json').read_text())
    if info['codebase_version'] != 'v2.1':
        raise ValueError('Expected LeRobot v2.1')
    episodes = {e['episode_index']: e for e in map(
        json.loads, (source / 'meta/episodes.jsonl').read_text().splitlines())}
    labels = json.loads(labels_path.read_text())
    label_map = {entry['episode_index']: entry['dense'] for entry in labels}
    if len(label_map) != len(labels) or not 0 < val_count < len(labels):
        raise ValueError('Duplicate labels or invalid validation count')
    names = labels[0]['dense']['names']
    for episode_id, dense in label_map.items():
        starts, ends = dense['start_frames'], dense['end_frames']
        if (dense['names'] != names or len(starts) != len(names) or
                len(ends) != len(names) or starts[0] != 0 or
                ends[-1] != episodes[episode_id]['length'] - 1 or
                any(a > b for a, b in zip(starts, ends)) or
                any(a != b + 1 for a, b in zip(starts[1:], ends[:-1]))):
            raise ValueError(f'Invalid annotation: episode {episode_id}')
    shuffled = sorted(label_map)
    random.Random(seed).shuffle(shuffled)
    splits = dict(train=sorted(shuffled[val_count:]), val=sorted(shuffled[:val_count]))
    manifest = dict(source=str(source), labels=str(labels_path), seed=seed,
                    labels_sha256=hashlib.sha256(labels_path.read_bytes()).hexdigest(),
                    excluded_episode_ids=sorted(set(episodes) - set(label_map)),
                    names=names, task_description=TASK, splits={})
    destinations = {key: Path(str(output_prefix) + '_' + key) for key in splits}
    for path in destinations.values():
        if path.exists():
            raise FileExistsError(f'Refusing to overwrite existing split: {path}')
    camera_keys = [key for key, feature in info['features'].items() if feature['dtype'] == 'video']

    def source_table(episode_id):
        return pq.read_table(source / info['data_path'].format(
            episode_index=episode_id, episode_chunk=episode_id // info['chunks_size']))

    # All statistics below are derived exclusively from the training split.
    stats = {}
    for key in ('observation.state', 'action'):
        arrays = [np.asarray(source_table(i)[key].to_pylist(), dtype=np.float64)
                  for i in splits['train']]
        values = np.concatenate(arrays)
        if not np.isfinite(values).all():
            raise ValueError(f'Non-finite training data in {key}')
        stats[key] = dict(mean=values.mean(0).tolist(), std=values.std(0).tolist(),
                          min=values.min(0).tolist(), max=values.max(0).tolist(),
                          q01=np.quantile(values, .01, axis=0).tolist(),
                          q99=np.quantile(values, .99, axis=0).tolist())
    durations = np.zeros(len(names), dtype=np.float64)
    for i in splits['train']:
        dense = label_map[i]
        durations += np.asarray(dense['end_frames']) - np.asarray(dense['start_frames']) + 1
    proportions = dict(zip(names, (durations / durations.sum()).tolist()))

    for split, ids in splits.items():
        destination = destinations[split]
        staging = destination.with_name(destination.name + '.preparing')
        if staging.exists():
            raise FileExistsError(f'Partial export exists: {staging}')
        (staging / 'meta').mkdir(parents=True)
        (staging / 'data/chunk-000').mkdir(parents=True)
        records, mapping, offset = [], [], 0
        for new_id, original_id in enumerate(ids):
            table = source_table(original_id)
            length = len(table)
            if length != episodes[original_id]['length']:
                raise ValueError(f'Parquet length mismatch: episode {original_id}')
            for key, values in (
                ('episode_index', np.full(length, new_id)),
                ('index', np.arange(offset, offset + length)),
                ('frame_index', np.arange(length)),
                ('task_index', np.zeros(length, dtype=np.int64)),
            ):
                column = table.schema.field(key)
                table = table.set_column(table.schema.get_field_index(key), column,
                                         pa.array(values, type=column.type))
            pq.write_table(table, staging / f'data/chunk-000/episode_{new_id:06d}.parquet')
            dense = label_map[original_id]
            records.append(dict(episode_index=new_id, source_episode_index=original_id,
                                tasks=[TASK], length=length,
                                sparse_subtask_names=['task'],
                                sparse_subtask_start_frames=[0],
                                sparse_subtask_end_frames=[length - 1],
                                dense_subtask_names=dense['names'],
                                dense_subtask_start_frames=dense['start_frames'],
                                dense_subtask_end_frames=dense['end_frames']))
            mapping.append(dict(episode_index=new_id, source_episode_index=original_id,
                                length=length))
            for camera in camera_keys:
                original_video = source / info['video_path'].format(
                    episode_index=original_id, episode_chunk=original_id // info['chunks_size'],
                    video_key=camera)
                if not original_video.is_file():
                    raise FileNotFoundError(original_video)
                link = staging / f'videos/chunk-000/{camera}/episode_{new_id:06d}.mp4'
                link.parent.mkdir(parents=True, exist_ok=True)
                link.symlink_to(os.path.relpath(original_video, link.parent))
            offset += length
        split_info = dict(info)
        split_info.update(total_episodes=len(ids), total_frames=offset,
                          total_tasks=1, total_chunks=1, total_videos=len(ids) * len(camera_keys),
                          chunks_size=1000, splits={'train': f'0:{len(ids)}'},
                          data_path='data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet',
                          video_path='videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4')
        write_json(staging / 'meta/info.json', split_info)
        write_json(staging / 'meta/stats.json', stats)
        write_json(staging / 'meta/temporal_proportions_dense.json', proportions)
        write_json(staging / 'meta/temporal_proportions_sparse.json', {'task': 1.0})
        (staging / 'meta/episodes.jsonl').write_text(
            ''.join(json.dumps(e, ensure_ascii=False) + '\n' for e in records), encoding='utf-8')
        (staging / 'meta/tasks.jsonl').write_text(
            json.dumps(dict(task_index=0, task=TASK)) + '\n')
        write_json(staging / 'meta/source_episode_mapping.json', mapping)
        staging.rename(destination)
        manifest['splits'][split] = dict(root=str(destination), episodes=mapping, frames=offset)
        print(f'{split}: {len(ids)} episodes, {offset} frames -> {destination}')
    write_json(Path(str(output_prefix) + '_split.json'), manifest)
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset-root', default='datasets/a10_9_21_sarm')
    parser.add_argument('--labels', default='datasets/a10_stages.json')
    parser.add_argument('--output-prefix', default='datasets/a10_sarm')
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--val-count', type=int, default=6)
    args = parser.parse_args()
    prepare(args.dataset_root, args.labels, args.output_prefix, args.seed, args.val_count)
