"""A10 v3 -> LeRobot v2.1 export with immutable input and resumable staging.

Layout follows huggingface/lerobot v0.3.3 datasets/utils.py. V3 shard offsets
follow v0.4.3 datasets/v30/convert_dataset_v21_to_v30.py in reverse.
"""
from __future__ import annotations

from collections import Counter
import copy
import ctypes
import fcntl
from fractions import Fraction
import hashlib
import json
import os
from pathlib import Path
import shutil

import av
import pyarrow as pa
import pyarrow.compute as pc
import pyarrow.parquet as pq

from .a10_recording_io import (
    checked_schema, data_path, dataset_read_lock, validate_dataset, video_segment, write_jsonl,
)
from .episode_writer import (
    aggregate_stats, episode_stats, json_write, load_v3, sync_dir, table_schema,
)


def _digest(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        while block := stream.read(8 * 1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def _publish(stage, destination):
    # Linux renameat2(RENAME_NOREPLACE) also protects a destination created
    # between the final existence check and rename, including an empty dir.
    libc = ctypes.CDLL(None, use_errno=True)
    rename = getattr(libc, 'renameat2', None)
    if rename is None:
        raise RuntimeError('此平台不支持原子且禁止覆盖的目录发布；已保留暂存目录')
    rename.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    rename.restype = ctypes.c_int
    if rename(-100, os.fsencode(stage), -100, os.fsencode(destination), 1) != 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error), str(destination))


def _source_snapshot(source, paths):
    records = {}
    for path in sorted(paths):
        st = path.stat()
        rel = path.relative_to(source).as_posix()
        records[rel] = [st.st_size, st.st_mtime_ns]
        if rel.startswith('meta/') or path.name == 'SIMULATED_DATA.json':
            records[rel].append(_digest(path))
    return records


def _video_plans(source, info, episodes, aliases, video_mode):
    jobs = []
    for ep in episodes:
        for key in aliases:
            path, start, end = video_segment(source, info, ep, key)
            jobs.append(dict(ep=ep, key=key, path=path, start=start, end=end))
    references = Counter(job['path'] for job in jobs)
    can_copy = dict.fromkeys(aliases, video_mode == 'auto')
    for job in jobs:
        key, ep, path = job['key'], job['ep'], job['path']
        with av.open(str(path)) as video:
            stream = video.streams.video[0]
            declared = info['features'][key]
            if ([stream.height, stream.width, 3] != declared['shape']
                    or stream.codec_context.codec.id != av.Codec(declared['info']['video.codec'], 'r').id):
                raise ValueError(f'Source video size/codec metadata mismatch: {path}')
            duration = float(stream.duration * stream.time_base) if stream.duration is not None else None
            complete = (references[path] == 1 and abs(job['start']) < 1e-5
                        and stream.frames == ep['length'] and duration is not None
                        and abs(duration - job['end']) < 1e-4)
            can_copy[key] = can_copy[key] and complete
    # Each feature has one codec declaration: never mix copied AV1 with H.264
    # under the same camera key when some clips need splitting.
    for job in jobs:
        job['mode'] = 'copy' if can_copy[job['key']] else 'h264'
    return jobs, can_copy


def _transcode(job, target, fps, notify):
    count = 0
    expected = job['ep']['length']
    with av.open(str(job['path'])) as src, av.open(str(target), 'w') as dst:
        source_stream = src.streams.video[0]
        source_stream.thread_type = 'AUTO'
        source_stream.codec_context.thread_count = 4
        if job['start'] > 0:
            # Seek to a preceding keyframe, then select by decoded timestamps.
            src.seek(int(job['start'] / source_stream.time_base), stream=source_stream,
                     backward=True, any_frame=False)
        stream = dst.add_stream('libx264', rate=fps)
        stream.width, stream.height = source_stream.width, source_stream.height
        stream.pix_fmt = 'yuv420p'
        stream.options = {'crf': '18', 'preset': 'ultrafast'}
        stream.codec_context.thread_count = 2
        for frame in src.decode(video=0):
            if frame.pts is None or frame.time_base is None:
                raise ValueError(f'Missing source video timestamp: {job["path"]}')
            stamp = float(frame.pts * frame.time_base)
            if stamp < job['start'] - 1e-5:
                continue
            if stamp >= job['end'] - 1e-5:
                break
            if abs(stamp - job['start'] - count / fps) > 1e-4:
                raise ValueError(f'Nonuniform source video timestamps: {job["path"]}')
            image = frame.to_ndarray(format='rgb24')
            output_frame = av.VideoFrame.from_ndarray(image, format='rgb24')
            output_frame.pts = count
            output_frame.time_base = Fraction(1, fps)
            count += 1
            for packet in stream.encode(output_frame):
                dst.mux(packet)
            if count % 300 == 0:
                notify('transcode', frames=count, total_frames=expected)
        for packet in stream.encode():
            dst.mux(packet)
    if count != expected:
        raise ValueError(f'Export video frame count mismatch: {count} != {expected}: {job["path"]}')


def export_openpi_dataset(source, destination, *, video_mode='auto', resume=False, progress=None):
    """Publish a fully validated v2.1 copy; leave checkpoints on interruption."""
    source, destination = Path(source).resolve(), Path(destination).resolve()
    if video_mode not in ('auto', 'h264'):
        raise ValueError('video_mode must be auto or h264')
    if destination == source or destination.is_relative_to(source) or source.is_relative_to(destination):
        raise ValueError('Export source/destination must be separate directories')
    if destination.exists():
        raise FileExistsError(f'输出目录已存在，不会覆盖：{destination}；请指定新的 --output')
    stage = destination.parent / f'.{destination.name}.export'
    if (stage.is_symlink() or stage.resolve() == source or stage.resolve().is_relative_to(source)
            or source.is_relative_to(stage.resolve())):
        raise ValueError('暂存目录不能位于源目录内或使用软链接；请选择其他 --output')
    checkpoint = stage / '.export-state.json'

    def notify(phase, **details):
        if progress is not None:
            progress(dict(phase=phase, **details))

    with dataset_read_lock(source):
        header = json.loads((source / 'meta/info.json').read_text())
        if header.get('codebase_version') != 'v3.0':
            raise ValueError('输入必须是干净的 LeRobot v3.0 数据集；meta/info.json 当前标记为 '
                             f'{header.get("codebase_version")}。请勿在源目录内转换或混放 v2.1 文件。')
        info, episodes, tasks = load_v3(source)
        if not episodes:
            raise ValueError('Export requires a nonempty v3.0 dataset')
        spec = checked_schema(source, info)
        aliases = spec.image_aliases
        expected_data = {data_path(source, info, ep) for ep in episodes}
        expected_videos = {video_segment(source, info, ep, key)[0] for ep in episodes for key in aliases}
        actual = set((source / 'data').rglob('*.parquet')) | set((source / 'videos').rglob('*.mp4'))
        extras = actual - expected_data - expected_videos
        if extras:
            example = min(extras).relative_to(source)
            raise ValueError(f'输入含有未引用或混合格式文件：{example}；请使用独立、完整的 v3.0 源目录。')
        notify('validate_source', episodes=len(episodes))
        # Validate numeric data, metadata, statistics and paths first. Video
        # decoding is checked during slicing and in the final output pass.
        validate_dataset(source, check_videos=False, _locked=True,
                         progress=lambda event: notify('validate_source', **event))
        jobs, can_copy = _video_plans(source, info, episodes, aliases, video_mode)
        paths = expected_data | expected_videos | set((source / 'meta').rglob('*'))
        paths = {p for p in paths if p.is_file()}
        if (source / 'SIMULATED_DATA.json').is_file():
            paths.add(source / 'SIMULATED_DATA.json')
        snapshot = _source_snapshot(source, paths)
        identity = dict(format=1, source=str(source), video_mode=video_mode, source_files=snapshot)
        destination.parent.mkdir(parents=True, exist_ok=True)
        # Lock the staging directory, not the destination or the source.
        try:
            stage.mkdir()
        except FileExistsError:
            if not resume:
                raise FileExistsError(f'存在未完成的导出：{stage}；使用相同命令加 --resume 继续')
            if not checkpoint.is_file():
                raise ValueError(f'暂存目录缺少有效检查点，请保留它并选择新的 --output：{stage}')
        with (stage / '.export.lock').open('a+') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise RuntimeError('该输出已有导出任务运行') from exc
            if checkpoint.exists():
                if not resume:
                    raise FileExistsError(f'存在未完成的导出：{stage}；请加 --resume')
                state = json.loads(checkpoint.read_text())
                if state.get('identity') != identity:
                    raise ValueError('断点的源文件或转换参数已变化；请指定新的 --output')
            else:
                if any(p.name != '.export.lock' for p in stage.iterdir()):
                    raise ValueError(f'暂存目录缺少有效检查点，请保留它并选择新的 --output：{stage}')
                state = dict(identity=identity, completed={})
                json_write(checkpoint, state)

            def verified(target):
                rel = target.relative_to(stage).as_posix()
                saved = state['completed'].get(rel)
                return (saved is not None and target.is_file()
                        and target.stat().st_size == saved['size'] and _digest(target) == saved['sha256'])

            def record(target):
                state['completed'][target.relative_to(stage).as_posix()] = dict(
                    size=target.stat().st_size, sha256=_digest(target))
                json_write(checkpoint, state)

            # Explicit v2.1 metadata fields, rather than leaking v3 shard settings.
            out = {key: copy.deepcopy(info[key]) for key in
                   ('robot_type', 'total_episodes', 'total_frames', 'total_tasks', 'fps', 'splits', 'features')}
            out.update(codebase_version='v2.1', a10_recording_schema=spec.name,
                       total_videos=len(jobs), total_chunks=(len(episodes) + 999) // 1000, chunks_size=1000,
                       data_path='data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet',
                       video_path='videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4')
            for feature in out['features'].values():
                feature.pop('fps', None)
            for old, new in aliases.items():
                out['features'][new] = out['features'].pop(old)
                if not can_copy[old]:
                    out['features'][new]['info'].update({'video.codec': 'h264', 'video.pix_fmt': 'yuv420p'})
            all_stats, summaries = [], []
            cached_path = None
            for number, ep in enumerate(episodes, 1):
                path = data_path(source, info, ep)
                if path != cached_path:
                    table = pq.read_table(path)
                    cached_path = path
                rows = table.filter(pc.equal(table['episode_index'], ep['episode_index'])).to_pylist()
                target = data_path(stage, out, ep)
                if not verified(target):
                    target.parent.mkdir(parents=True, exist_ok=True)
                    pq.write_table(pa.Table.from_pylist(rows, schema=table_schema(spec.name)), target)
                    record(target)
                statistics = episode_stats(ep)
                for old, new in aliases.items():
                    statistics[new] = statistics.pop(old)
                all_stats.append(statistics)
                summaries.append(dict(episode_index=ep['episode_index'], length=ep['length'], tasks=ep['tasks']))
                notify('data', completed=number, total=len(episodes))
            counts = dict(copied=0, transcoded=0, resumed=0)
            for number, job in enumerate(jobs, 1):
                target, _, _ = video_segment(stage, out, job['ep'], aliases[job['key']])
                details = dict(completed=number - 1, total=len(jobs), episode=job['ep']['episode_index'],
                               camera=aliases[job['key']], mode=job['mode'])
                notify('video', **details)
                if verified(target):
                    counts['resumed'] += 1
                else:
                    target.parent.mkdir(parents=True, exist_ok=True)
                    if job['mode'] == 'copy':
                        shutil.copy2(job['path'], target)
                    else:
                        _transcode(job, target, info['fps'],
                                   lambda phase, **event: notify(phase, **details, **event))
                    record(target)
                counts['copied' if job['mode'] == 'copy' else 'transcoded'] += 1
                notify('video_done', **dict(details, completed=number))
            json_write(stage / 'meta/info.json', out)
            write_jsonl(stage / 'meta/episodes.jsonl', summaries)
            write_jsonl(stage / 'meta/tasks.jsonl', [dict(task_index=k, task=v) for k, v in sorted(tasks.items())])
            write_jsonl(stage / 'meta/episodes_stats.jsonl',
                        [dict(episode_index=ep['episode_index'], stats=st) for ep, st in zip(episodes, all_stats)])
            json_write(stage / 'meta/stats.json', aggregate_stats(all_stats))
            json_write(stage / 'meta/export_manifest.json', dict(
                source=str(source), schema=spec.name, camera_aliases=aliases, video_mode=video_mode,
                action_source=spec.contract()['action_source'], raw_action_preserved=True,
                action_time_offset_frames=0 if spec.dual else None, training_actions='observation.state',
                videos=counts, image_statistics='Source image statistics retained; recompute training norm stats.'))
            for rel in ('SIMULATED_DATA.json', 'meta/recording_contract.json', 'meta/xlevr_diagnostics'):
                src = source / rel
                dst = stage / ('meta/source_recording_contract.json' if rel == 'meta/recording_contract.json' else rel)
                if src.is_dir():
                    shutil.copytree(src, dst, dirs_exist_ok=True)
                elif src.is_file():
                    dst.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(src, dst)
            notify('validate_output')
            report = validate_dataset(stage, progress=lambda event: notify('validate_output', **event))
            if _source_snapshot(source, paths) != snapshot:
                raise RuntimeError('源数据在转换期间改变，输出未发布；请检查录制是否关闭')
            if destination.exists():
                raise FileExistsError(f'输出目录已出现，不会覆盖：{destination}')
            for path in stage.rglob('*'):
                if path.is_file():
                    with path.open('rb') as stream:
                        os.fsync(stream.fileno())
            _publish(stage, destination)
            (destination / '.export-state.json').unlink()
            (destination / '.export.lock').unlink()
            sync_dir(destination.parent)
            notify('complete', **counts)
            return dict(report, **counts)
