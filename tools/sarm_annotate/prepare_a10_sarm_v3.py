#!/usr/bin/env python
"""Create standalone v3 train/valid splits without modifying the source."""
import argparse
import copy
import json
import random
import shutil
from pathlib import Path

import av
import numpy as np
import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq

TASK = 'Pick up the water bottle, put it in the nearby box, close the box lid, and return to the initial position.'


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def numeric_stats(values):
    values = np.asarray(values, dtype=np.float64)
    if values.ndim == 1:
        values = values[:, None]
    if not np.isfinite(values).all():
        raise ValueError('Non-finite numeric data')
    result = {key: fn(values, axis=0).tolist() for key, fn in
              [('min', np.min), ('max', np.max), ('mean', np.mean), ('std', np.std)]}
    result['count'] = [len(values)]
    for q in (.01, .1, .5, .9, .99):
        result[f'q{int(q * 100):02d}'] = np.quantile(values, q, axis=0).tolist()
    return result


def prepare(source, prefix, seed=42, valid_count=6, excluded=(2, 12, 25)):
    source, prefix = Path(source).resolve(), Path(prefix).resolve()
    info = json.loads((source / 'meta/info.json').read_text())
    if info['codebase_version'] != 'v3.0':
        raise ValueError('Expected v3.0 source')
    rows = sorted([r for p in sorted((source / 'meta/episodes').rglob('*.parquet'))
                   for r in pq.read_table(p).to_pylist()], key=lambda r: r['episode_index'])
    by_id = {r['episode_index']: r for r in rows}
    if len(by_id) != len(rows) or not set(excluded) <= set(by_id):
        raise ValueError('Duplicate episodes or unknown exclusions')
    ids = sorted(set(by_id) - set(excluded))
    random.Random(seed).shuffle(ids)
    splits = {'train': sorted(ids[valid_count:]), 'valid': sorted(ids[:valid_count])}
    roots = {s: Path(str(prefix) + '_' + s) for s in splits}
    if any(r.exists() for r in roots.values()):
        raise FileExistsError('Output exists; use a different prefix')
    cameras = [k for k, f in info['features'].items() if f['dtype'] == 'video']
    tables = {}
    for row in rows:
        p = source / info['data_path'].format(chunk_index=row['data/chunk_index'],
                                             file_index=row['data/file_index'])
        if p not in tables:
            tables[p] = pq.read_table(p)

    def episode_table(ep):
        row = by_id[ep]
        p = source / info['data_path'].format(chunk_index=row['data/chunk_index'],
                                             file_index=row['data/file_index'])
        table = tables[p]
        table = table.filter(pa.compute.equal(table['episode_index'], ep))
        if len(table) != row['length']:
            raise ValueError(f'Length mismatch: {ep}')
        return table

    training = pa.concat_tables([episode_table(ep) for ep in splits['train']])
    normalization = {k: numeric_stats(training[k].to_pylist())
                     for k in ('action', 'observation.state')}
    manifest = dict(source=str(source), seed=seed, excluded_episode_ids=list(excluded),
                    task_description=TASK, splits={})
    for split, selected in splits.items():
        root = roots[split]
        (root / 'meta/episodes/chunk-000').mkdir(parents=True)
        (root / 'data/chunk-000').mkdir(parents=True)
        records, data, mapping, offset = [], [], [], 0
        image_pixels = {k: [] for k in cameras}
        for new_id, old_id in enumerate(selected):
            old = by_id[old_id]
            table = episode_table(old_id)
            n = len(table)
            for key, values in [('episode_index', np.full(n, new_id)),
                                ('index', np.arange(offset, offset+n)),
                                ('frame_index', np.arange(n)),
                                ('task_index', np.zeros(n, dtype=np.int64))]:
                field = table.schema.field(key)
                table = table.set_column(table.schema.get_field_index(key), field,
                                         pa.array(values, type=field.type))
            row = {k: v for k, v in old.items() if not k.startswith('stats/')
                   and not k.startswith(('dense_', 'sparse_', 'subtask_'))}
            row.update(episode_index=new_id, source_episode_index=old_id,
                       dataset_from_index=offset, dataset_to_index=offset+n,
                       tasks=[TASK])
            row.update({'data/chunk_index': 0, 'data/file_index': 0,
                        'meta/episodes/chunk_index': 0, 'meta/episodes/file_index': 0})
            for k in table.column_names:
                for stat, value in numeric_stats(table[k].to_pylist()).items():
                    row[f'stats/{k}/{stat}'] = value
            for camera in cameras:
                src = source / info['video_path'].format(video_key=camera,
                    chunk_index=old[f'videos/{camera}/chunk_index'],
                    file_index=old[f'videos/{camera}/file_index'])
                # This source stores one complete episode per video.
                if abs(old[f'videos/{camera}/from_timestamp']) > 1e-6:
                    raise ValueError('Expected separate videos, not concatenated segments')
                dst = root / f'videos/{camera}/chunk-000/file-{new_id:03d}.mp4'
                dst.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(src, dst)
                sampled = []
                with av.open(str(dst)) as container:
                    stream = container.streams.video[0]
                    codec = stream.codec_context.name
                    for fi, frame in enumerate(container.decode(stream)):
                        if fi % 60 == 0:
                            sampled.append(frame.to_ndarray(format='rgb24')[::8, ::8] / 255.)
                    count = fi + 1
                    fps = float(stream.average_rate)
                    width, height = stream.width, stream.height
                if count != n or abs(fps - info['fps']) > 1e-6:
                    raise ValueError(f'Video mismatch: {split}/{new_id}/{camera}: {count} vs {n}')
                pix = np.concatenate([a.reshape(-1, 3) for a in sampled])
                image_pixels[camera].append(pix)
                for stat, value in numeric_stats(pix).items():
                    row[f'stats/{camera}/{stat}'] = ([len(sampled)] if stat == 'count'
                        else np.asarray(value).reshape(3, 1, 1).tolist())
                row.update({f'videos/{camera}/chunk_index': 0,
                            f'videos/{camera}/file_index': new_id,
                            f'videos/{camera}/from_timestamp': 0.,
                            f'videos/{camera}/to_timestamp': n / fps})
                feature = info['features'][camera]['info']
                feature.update({'video.codec': codec, 'video.fps': fps,
                                'video.width': width, 'video.height': height,
                                'video.pix_fmt': 'yuv420p'})
            mapping.append(dict(episode_index=new_id, source_episode_index=old_id, length=n))
            records.append(row)
            data.append(table)
            offset += n
            print(f'{split}: episode {new_id} <- source {old_id}, {n} frames', flush=True)
        combined = pa.concat_tables(data)
        pq.write_table(combined, root / 'data/chunk-000/file-000.parquet')
        pq.write_table(pa.Table.from_pylist(records), root / 'meta/episodes/chunk-000/file-000.parquet')
        pd.DataFrame({'task_index': [0], 'task': [TASK]}, index=[TASK]).to_parquet(
            root / 'meta/tasks.parquet')
        out_info = copy.deepcopy(info)
        out_info.update(total_episodes=len(selected), total_frames=offset, total_tasks=1,
                        total_videos=len(selected)*len(cameras), total_chunks=1,
                        splits={'train': f'0:{len(selected)}'})
        write_json(root / 'meta/info.json', out_info)
        stats = {k: numeric_stats(combined[k].to_pylist()) for k in combined.column_names}
        stats.update(normalization)
        for camera in cameras:
            stats[camera] = {k: ([sum((row['length'] + 59)//60 for row in records)] if k == 'count' else
                np.asarray(v).reshape(3,1,1).tolist()) for k, v in
                numeric_stats(np.concatenate(image_pixels[camera])).items()}
        write_json(root / 'meta/stats.json', stats)
        write_json(root / 'meta/source_episode_mapping.json', mapping)
        manifest['splits'][split] = dict(root=str(root), episodes=mapping, frames=offset)
    write_json(Path(str(prefix) + '_split.json'), manifest)
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset-root', default='datasets/a10_sarm_v3')
    parser.add_argument('--output-prefix', default='datasets/a10_sarm_v3')
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--valid-count', type=int, default=6)
    args = parser.parse_args()
    prepare(args.dataset_root, args.output_prefix, args.seed, args.valid_count)
