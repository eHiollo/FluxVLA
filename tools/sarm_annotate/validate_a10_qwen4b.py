#!/usr/bin/env python
"""Validate exported Qwen labels and write an inspectable quality summary."""
import json
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq

from annotate_a10_qwen4b import NAMES


def validate(prefix='datasets/a10_sarm_v3'):
    prefix = Path(prefix)
    manifest = json.loads(Path(str(prefix) + '_split.json').read_text())
    combined = json.loads(Path(str(prefix) + '_qwen_4b_labels.json').read_text())
    expected_ids, flags, summaries, all_durations = [], [], {}, []
    sampling_counts = {}
    training_stats = None
    training_priors = None
    for split in ('train', 'valid'):
        root = Path(str(prefix) + '_' + split)
        info = json.loads((root / 'meta/info.json').read_text())
        rows = pq.read_table(root / 'meta/episodes/chunk-000/file-000.parquet').to_pylist()
        data = pq.read_table(root / 'data/chunk-000/file-000.parquet')
        exported = json.loads(Path(str(root) + '_qwen_4b_labels.json').read_text())
        assert info['codebase_version'] == 'v3.0'
        assert len(rows) == len(exported) == info['total_episodes']
        assert sum(r['length'] for r in rows) == len(data) == info['total_frames']
        assert [r['source_episode_index'] for r in rows] == [r['source_episode_index'] for r in manifest['splits'][split]['episodes']]
        for row, label in zip(rows, exported):
            ep, old, n = row['episode_index'], row['source_episode_index'], row['length']
            expected_ids.append(old)
            assert old not in manifest['excluded_episode_ids']
            assert row['dense_subtask_names'] == NAMES == label['dense']['names']
            starts, ends = row['dense_subtask_start_frames'], row['dense_subtask_end_frames']
            assert starts == label['dense']['start_frames'] and ends == label['dense']['end_frames']
            assert len(starts) == len(ends) == 4 and starts[0] == 0 and ends[-1] == n-1
            assert all(a <= b for a,b in zip(starts,ends))
            assert all(a == b+1 for a,b in zip(starts[1:],ends[:-1]))
            assert row['sparse_subtask_start_frames'] == [0] and row['sparse_subtask_end_frames'] == [n-1]
            evidence_path = root / f'meta/qwen4b_annotations/episode_{ep:06d}_qwen_4b.json'
            evidence = json.loads(evidence_path.read_text())
            sampling = str(evidence['sample_fps'])
            sampling_counts[sampling] = sampling_counts.get(sampling, 0) + 1
            assert evidence['source_episode_index'] == old and evidence['start_frames'] == starts
            durations = (np.array(ends)-np.array(starts)+1)/info['fps']
            all_durations.append(durations.tolist())
            reasons = []
            if evidence['response']['uncertain']:
                reasons.append('model_marked_uncertain')
            short = [NAMES[i] for i,d in enumerate(durations) if d < 1.]
            if short:
                reasons.append('stage_shorter_than_1_second')
            if reasons:
                flags.append(dict(split=split, episode_index=ep, source_episode_index=old,
                                  reasons=reasons, short_stages=short,
                                  stage_durations_seconds=durations.tolist()))
            for key, feature in info['features'].items():
                if feature['dtype'] != 'video':
                    continue
                path = root / info['video_path'].format(video_key=key,
                    chunk_index=row[f'videos/{key}/chunk_index'],file_index=row[f'videos/{key}/file_index'])
                assert path.is_file() and not path.is_symlink()
                assert feature['info']['video.codec'] == 'h264'
                assert row[f'videos/{key}/from_timestamp'] == 0
                assert np.isclose(row[f'videos/{key}/to_timestamp'],n/info['fps'])
        stats = json.loads((root / 'meta/stats.json').read_text())
        priors = json.loads((root / 'meta/temporal_proportions_dense.json').read_text())
        if split == 'train':
            training_stats, training_priors = stats, priors
            for key in ('action', 'observation.state'):
                a = np.array(data[key].to_pylist())
                assert np.isfinite(a).all()
                assert np.allclose(a.mean(0),stats[key]['mean'],rtol=1e-5,atol=1e-6)
                assert np.allclose(a.std(0),stats[key]['std'],rtol=1e-5,atol=1e-6)
            expected = np.mean([(np.array(r['dense_subtask_end_frames'])-
                                np.array(r['dense_subtask_start_frames'])+1)/r['length'] for r in rows],axis=0)
            assert np.allclose(expected,list(priors.values()))
        else:
            for key in ('action','observation.state'):
                assert stats[key] == training_stats[key]
            assert priors == training_priors
        summaries[split] = dict(episodes=len(rows),frames=len(data),videos=2*len(rows))
    assert len(expected_ids) == len(set(expected_ids)) == len(combined) == 29
    assert {r['episode_index'] for r in combined} == set(expected_ids)
    result = dict(status='structural_checks_passed', model='Qwen/Qwen3-VL-4B-Instruct',
                  excluded_episode_ids=manifest['excluded_episode_ids'],splits=summaries,
                  review_flags=flags, sampling_fps_episode_counts=sampling_counts,
                  annotation_precision='Model estimates from timestamped frames sampled at 1 or 2 FPS',
                  label_semantics='0-based inclusive frame intervals; model-estimated visual event boundaries',
                  training_only_normalization_and_temporal_priors=True)
    path = Path(str(prefix) + '_qwen_4b_quality.json')
    path.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))
    return result


if __name__ == '__main__':
    validate()
