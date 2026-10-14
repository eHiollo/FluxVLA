"""Check split export against non-contiguous source IDs and held-out stats."""

import importlib.util
import json
from pathlib import Path

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import pytest


spec = importlib.util.spec_from_file_location(
    'prepare_a10', Path(__file__).resolve().parents[2] /
    'tools/sarm_annotate/prepare_a10_sarm.py')
prepare_a10 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prepare_a10)


def test_split_reindex_labels_and_train_only_statistics(tmp_path):
    source = tmp_path / 'source'
    (source / 'meta').mkdir(parents=True)
    (source / 'data/chunk-000').mkdir(parents=True)
    (source / 'videos/chunk-000/camera').mkdir(parents=True)
    info = dict(codebase_version='v2.1', fps=30, chunks_size=1000,
                features={'camera': {'dtype': 'video'}},
                data_path='data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet',
                video_path='videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4')
    (source / 'meta/info.json').write_text(json.dumps(info))
    episodes = [dict(episode_index=i, length=4) for i in [0, 2, 5, 8]]
    (source / 'meta/episodes.jsonl').write_text('\n'.join(map(json.dumps, episodes)))
    for episode in episodes:
        i = episode['episode_index']
        table = pa.table({'episode_index': [i] * 4, 'frame_index': list(range(4)),
                          'index': list(range(i * 4, i * 4 + 4)), 'task_index': [0] * 4,
                          'observation.state': [[float(i)]] * 4,
                          'action': [[float(i)]] * 4, 'timestamp': np.arange(4) / 30})
        pq.write_table(table, source / f'data/chunk-000/episode_{i:06d}.parquet')
        (source / f'videos/chunk-000/camera/episode_{i:06d}.mp4').write_bytes(b'video')
    labels = tmp_path / 'labels.json'
    labels.write_text(json.dumps([dict(episode_index=i, dense=dict(
        names=['grab', 'place'], start_frames=[0, 2], end_frames=[1, 3])) for i in [0, 5, 8]]))
    source_metadata = (source / 'meta/episodes.jsonl').read_bytes()
    prefix = tmp_path / 'export'
    manifest = prepare_a10.prepare(source, labels, prefix, seed=42, val_count=1)
    assert manifest['excluded_episode_ids'] == [2]
    train = manifest['splits']['train']
    val = manifest['splits']['val']
    train_ids = [x['source_episode_index'] for x in train['episodes']]
    val_ids = [x['source_episode_index'] for x in val['episodes']]
    assert not set(train_ids) & set(val_ids)
    assert set(train_ids + val_ids) == {0, 5, 8}
    for part in [train, val]:
        root = Path(part['root'])
        for mapping in part['episodes']:
            i = mapping['episode_index']
            table = pq.read_table(root / f'data/chunk-000/episode_{i:06d}.parquet')
            assert table['episode_index'].to_pylist() == [i] * 4
            link = root / f'videos/chunk-000/camera/episode_{i:06d}.mp4'
            assert link.is_file()
            assert link.resolve().name == f'episode_{mapping["source_episode_index"]:06d}.mp4'
        stats = json.loads((root / 'meta/stats.json').read_text())
        assert stats['observation.state']['mean'] == [np.mean(train_ids)]
        metadata = [json.loads(line) for line in (root / 'meta/episodes.jsonl').read_text().splitlines()]
        assert metadata[0]['dense_subtask_start_frames'] == [0, 2]
    assert (source / 'meta/episodes.jsonl').read_bytes() == source_metadata
    with pytest.raises(FileExistsError):
        prepare_a10.prepare(source, labels, prefix, seed=42, val_count=1)
