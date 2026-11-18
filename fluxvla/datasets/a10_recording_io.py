"""Offline validation and OpenPI v2.1 export of versioned A10 single/dual-arm contracts."""
from __future__ import annotations

from contextlib import contextmanager
import fcntl
import json
from pathlib import Path

import av
import numpy as np
import pyarrow as pa
import pyarrow.compute as pc
import pyarrow.parquet as pq

from .episode_writer import (
    aggregate_stats, episode_stats,
    json_write, load_v3, safe_path, sync_dir, table_schema,
)

from .a10_recording_schema import get_schema, schema_from_info

IMAGE_ALIASES = get_schema().image_aliases  # legacy public alias


def checked_schema(root, info):
    spec = schema_from_info(info)
    if spec.dual:
        filename = ('recording_contract.json' if info['codebase_version'] == 'v3.0'
                    else 'source_recording_contract.json')
        contract = json.loads((Path(root) / 'meta' / filename).read_text())
        if any(contract.get(key) != value for key, value in spec.contract().items()):
            raise ValueError('Dual recording contract mismatch (order/units/action semantics)')
        if contract.get('fps') != 30 or contract.get('synthetic') != (Path(root)/'SIMULATED_DATA.json').exists():
            raise ValueError('Dual recording contract fps/synthetic marker mismatch')
        expected_cameras = set(spec.cameras)
        if set(contract.get('cameras', {})) != expected_cameras:
            raise ValueError('Dual recording contract camera mismatch')
        for camera, settings in contract['cameras'].items():
            key = f'observation.images.{camera}'
            if info['codebase_version'] == 'v2.1': key = spec.image_aliases[key]
            if info['features'][key]['shape'] != [settings['height'], settings['width'], 3]:
                raise ValueError('Dual camera dimensions differ from recording contract')
    return spec


@contextmanager
def dataset_read_lock(root):
    # Never create a lock or modify an externally collected source dataset.
    lock = Path(root) / '.recording.lock'
    if not lock.exists():
        yield
        return
    with lock.open('r') as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_SH | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError('Dataset is being recorded; close writer before reading/exporting') from exc
        yield


def read_jsonl(path):
    with Path(path).open(encoding='utf-8') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def write_jsonl(path, rows):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')


def dataset_metadata(root):
    root = Path(root)
    info = json.loads((root / 'meta/info.json').read_text())
    if info['codebase_version'] == 'v3.0':
        return load_v3(root)
    if info['codebase_version'] != 'v2.1':
        raise ValueError('Only LeRobot v3.0 and v2.1 are supported')
    episodes = read_jsonl(root / 'meta/episodes.jsonl')
    tasks = {r['task_index']: r['task'] for r in read_jsonl(root / 'meta/tasks.jsonl')}
    ep_stats = read_jsonl(root / 'meta/episodes_stats.jsonl')
    if [r['episode_index'] for r in ep_stats] != [e['episode_index'] for e in episodes]:
        raise ValueError('Episode statistics do not match episodes')
    for ep, statistics in zip(episodes, ep_stats):
        ep.update({f'stats/{k}/{s}': v for k, st in statistics['stats'].items() for s, v in st.items()})
    return info, episodes, tasks


def data_path(root, info, ep):
    args = dict(episode_index=ep['episode_index'], episode_chunk=ep['episode_index']//info['chunks_size'])
    if info['codebase_version'] == 'v3.0':
        args.update(chunk_index=ep['data/chunk_index'], file_index=ep['data/file_index'])
    return safe_path(root, info['data_path'].format(**args))


def video_segment(root, info, ep, key):
    args = dict(video_key=key, episode_index=ep['episode_index'], episode_chunk=ep['episode_index']//info['chunks_size'])
    start = 0.
    if info['codebase_version'] == 'v3.0':
        args.update(chunk_index=ep[f'videos/{key}/chunk_index'], file_index=ep[f'videos/{key}/file_index'])
        start = ep[f'videos/{key}/from_timestamp']
        end = ep[f'videos/{key}/to_timestamp']
    else:
        end = ep['length']/info['fps']
    return safe_path(root, info['video_path'].format(**args)), start, end


def video_index(path, *, progress=None):
    with av.open(str(path)) as video:
        stream = video.streams.video[0]
        dimensions = (stream.height, stream.width, 3)
        codec = stream.codec_context.name
        stream.thread_type = 'AUTO'
        stream.codec_context.thread_count = 4
        timestamps = []
        for frame in video.decode(video=0):
            if frame.pts is None or frame.time_base is None:
                raise ValueError(f'Missing video timestamp: {path}')
            timestamps.append(float(frame.pts * frame.time_base))
            if progress is not None and len(timestamps) % 300 == 0:
                progress(dict(path=str(path), frames=len(timestamps), total_frames=stream.frames))
        timestamps = np.asarray(timestamps)
    if not len(timestamps) or not np.isfinite(timestamps).all() or np.any(np.diff(timestamps) <= 0):
        raise ValueError(f'Invalid video timestamps: {path}')
    return timestamps, dimensions, codec


def validate_dataset(root, *, check_videos=True, _locked=False, progress=None):
    """Validate every episode; optionally decode videos for frame/timestamp checks."""
    if not _locked:
        with dataset_read_lock(root):
            return validate_dataset(root, check_videos=check_videos, _locked=True, progress=progress)
    root = Path(root)
    if (root / '.recording/active').exists():
        raise ValueError('Unfinished recording/commit exists; recover it before validation')
    info, episodes, tasks = dataset_metadata(root)
    if info['fps'] != 30:
        raise ValueError('Expected A10 30 fps')
    features = info['features']; version = info['codebase_version']
    spec = checked_schema(root, info); aliases = spec.image_aliases
    keys = list(aliases if version == 'v3.0' else aliases.values())
    expected = table_schema(spec.name).names
    if set(features) != set(expected) | set(keys):
        raise ValueError('Unexpected feature names for the selected A10 schema')
    for key, names in (('observation.state', list(spec.state_names)), ('action', list(spec.action_names))):
        if features[key]['shape'] != [len(names)] or features[key]['dtype'] != 'float32' or features[key]['names'] != names:
            raise ValueError(f'Invalid feature contract: {key}')
    for key in keys:
        feature = features[key]
        if feature['dtype'] != 'video' or len(feature['shape']) != 3 or feature['shape'][-1] != 3:
            raise ValueError(f'Invalid camera feature: {key}')
    if [e['episode_index'] for e in episodes] != list(range(info['total_episodes'])):
        raise ValueError('Episode indexes/count are inconsistent')
    if sorted(tasks) != list(range(info['total_tasks'])) or any(not isinstance(t, str) or not t.strip() for t in tasks.values()):
        raise ValueError('Invalid task metadata')
    videos = {}; cached_path = None; table = None; total = 0; data_counts = {}
    for ep in episodes:
        eid, count = ep['episode_index'], ep['length']
        if progress is not None:
            progress(dict(episode=eid, total_episodes=len(episodes)))
        if count < 2: raise ValueError(f'Episode {eid} has fewer than two frames')
        path = data_path(root, info, ep)
        if path != cached_path:
            table = pq.read_table(path); cached_path = path
        data_counts[path] = table.num_rows
        rows = table.filter(pc.equal(table['episode_index'], eid)).to_pylist()
        if len(rows) != count or set(table.column_names) != set(table_schema(spec.name).names):
            raise ValueError(f'Episode {eid}: data columns/frame count mismatch')
        for field in table_schema(spec.name):
            actual = table.schema.field(field.name).type
            if pa.types.is_fixed_size_list(field.type):
                if not (pa.types.is_list(actual) or pa.types.is_fixed_size_list(actual)) or actual.value_type != pa.float32():
                    raise ValueError(f'Invalid column type: {field.name}')
            elif actual != field.type: raise ValueError(f'Invalid column type: {field.name}')
        for k, length in (('observation.state', len(spec.state_names)), ('action', len(spec.action_names))):
            values = np.asarray([r[k] for r in rows])
            if values.shape != (count, length) or not np.isfinite(values).all(): raise ValueError(f'Invalid {k} in episode {eid}')
        if spec.dual:
            for r in rows: spec.validate_values(r['observation.state'], r['action'])
        if [r['frame_index'] for r in rows] != list(range(count)) or [r['index'] for r in rows] != list(range(total,total+count)):
            raise ValueError(f'Episode {eid}: frame/global indexes mismatch')
        if not np.allclose([r['timestamp'] for r in rows], np.arange(count)/30, atol=1e-4, rtol=0):
            raise ValueError(f'Episode {eid}: nonuniform timestamps')
        if any(r['task_index'] not in tasks for r in rows) or set(ep['tasks']) != {tasks[r['task_index']] for r in rows}:
            raise ValueError(f'Episode {eid}: task mismatch')
        if version == 'v3.0' and (ep['dataset_from_index'] != total or ep['dataset_to_index'] != total+count):
            raise ValueError(f'Episode {eid}: dataset range mismatch')
        statistics = episode_stats(ep)
        for key in features:
            st = statistics.get(key, {})
            valid_count = (isinstance(st.get('count'), list) and len(st['count']) == 1
                           and 0 < st['count'][0] <= count) if key in keys else st.get('count') == [count]
            if not valid_count or not all(k in st for k in ('min','max','mean','std')):
                raise ValueError(f'Episode {eid}: missing/invalid statistics for {key}')
            shape = (3,1,1) if key in keys else tuple(features[key]['shape'])
            if any(not np.isfinite(np.asarray(v)).all() for v in st.values()):
                raise ValueError(f'Episode {eid}: nonfinite statistics')
            if any(np.asarray(st[k]).shape != shape for k in ('min','max','mean','std')):
                raise ValueError(f'Episode {eid}: invalid statistics shape: {key}')
        for key in keys:
            path, start, end = video_segment(root, info, ep, key)
            if not path.is_file() or not np.isfinite([start,end]).all() or start < 0 or abs(end-start-count/30) > 1e-4:
                raise ValueError(f'Episode {eid}: video path/duration mismatch')
            if check_videos:
                if path not in videos:
                    if progress is not None:
                        progress(dict(path=str(path), frames=0))
                    videos[path] = video_index(path, progress=progress)
                stamps, dimensions, codec = videos[path]
                segment = stamps[(stamps >= start-1e-5) & (stamps < end-1e-5)]
                if len(segment) != count or not np.allclose(segment-start, np.arange(count)/30, atol=1e-4, rtol=0):
                    raise ValueError(f'Episode {eid}: video frames/timestamps mismatch: {key}')
                if list(dimensions) != features[key]['shape'] or av.Codec(codec,'r').id != av.Codec(features[key]['info']['video.codec'],'r').id:
                    raise ValueError(f'Episode {eid}: video size/codec metadata mismatch: {key}')
        total += count
    global_stats = json.loads((root/'meta/stats.json').read_text())
    merged = aggregate_stats([episode_stats(ep) for ep in episodes])
    if set(global_stats) != set(merged):
        raise ValueError('Global statistics features mismatch')
    for key, st in merged.items():
        for name in ('count','min','max','mean','std'):
            actual = np.asarray(global_stats[key].get(name))
            if actual.shape != np.asarray(st[name]).shape or not np.allclose(actual, st[name], atol=1e-5, rtol=1e-4):
                raise ValueError(f'Global statistics mismatch: {key}/{name}')
    if total != info['total_frames'] or sum(data_counts.values()) != total:
        raise ValueError('Dataset total_frames mismatch or unreferenced rows in data files')
    return dict(schema=spec.name, version=version, episodes=len(episodes), frames=total, fps=30,
                videos_checked=check_videos, synthetic=(root/'SIMULATED_DATA.json').exists())


def export_openpi_dataset(source, destination, *, video_mode='auto', resume=False, progress=None):
    """Export a verified v2.1 copy; see a10_dataset_export for resumable staging."""
    from .a10_dataset_export import export_openpi_dataset as export
    return export(source, destination, video_mode=video_mode, resume=resume, progress=progress)
