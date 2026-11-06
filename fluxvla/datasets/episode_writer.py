"""A10 single/dual-arm LeRobot v3 writer, adapted from projects/a10/recording.py.

No robot/VR dependency. One producer, a bounded encoder queue, one staged
episode and an explicit recoverable commit. The wire protocol is NOT `action`.
"""
from __future__ import annotations

import copy
import fcntl
import hashlib
import json
import os
from pathlib import Path
import queue
import shutil
import threading
import time
from fractions import Fraction

import av
import numpy as np
import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq

from .a10_recording_schema import get_schema

# Backward-compatible names used by existing single-arm callers/tests.
_LEGACY = get_schema()
CAMERAS = _LEGACY.cameras
STATE_NAMES = list(_LEGACY.state_names)
ACTION_NAMES = list(_LEGACY.action_names)
CONTRACT = _LEGACY.contract()


def json_write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
    os.replace(temporary, path)
    sync_dir(path.parent)


def sync_dir(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def safe_path(root, relative):
    root = Path(root).resolve()
    result = (root / relative).resolve()
    if not result.is_relative_to(root):
        raise ValueError(f'Path escapes dataset: {relative}')
    return result


def stats(values):
    values = np.asarray(values, dtype=np.float64)
    if values.ndim == 1:
        values = values[:, None]
    return {**{k: v.tolist() for k, v in dict(mean=values.mean(axis=0),
            std=values.std(axis=0), min=values.min(axis=0), max=values.max(axis=0)).items()},
            **{f'q{q:02d}': np.quantile(values, q / 100, axis=0).tolist()
               for q in (1, 10, 50, 90, 99)}, 'count': [len(values)]}


def aggregate_stats(episodes):
    """Merge exact counts/moments; quantiles stay per episode (not averaged)."""
    result = {}
    for key in set().union(*(s.keys() for s in episodes)) if episodes else []:
        items = [s[key] for s in episodes if key in s]
        n = np.array([s['count'][0] for s in items], dtype=float)
        means = np.array([s['mean'] for s in items]); stds = np.array([s['std'] for s in items])
        weights = n.reshape((-1,) + (1,) * (means.ndim - 1)) / n.sum()
        mean = (means * weights).sum(axis=0)
        var = ((stds**2 + (means - mean)**2) * weights).sum(axis=0)
        result[key] = dict(mean=mean.tolist(), std=np.sqrt(np.maximum(var, 0)).tolist(),
                           min=np.min([s['min'] for s in items], axis=0).tolist(),
                           max=np.max([s['max'] for s in items], axis=0).tolist(), count=[int(n.sum())])
    return result


def table_schema(schema="a10_single_v1"):
    spec = get_schema(schema)
    return pa.schema([('observation.state', pa.list_(pa.float32(), len(spec.state_names))),
                      ('action', pa.list_(pa.float32(), len(spec.action_names))), ('timestamp', pa.float32()),
                      *[(k, pa.int64()) for k in ('frame_index', 'episode_index', 'task_index', 'index')]])


def load_v3(root):
    root = Path(root)
    info = json.loads((root / 'meta/info.json').read_text())
    episodes = [r for p in sorted((root / 'meta/episodes').rglob('*.parquet'))
                for r in pq.read_table(p).to_pylist()]
    episodes.sort(key=lambda r: r['episode_index'])
    tasks_frame = pd.read_parquet(root / 'meta/tasks.parquet')
    if 'task' in tasks_frame.columns:
        tasks = dict(zip(tasks_frame.task_index.astype(int), tasks_frame.task))
    else:
        tasks = {int(row.task_index): str(name) for name, row in tasks_frame.iterrows()}
    return info, episodes, tasks


class EpisodeWriter:
    """Own-format resume only. Existing legacy datasets can be validated/exported.

    append() copies input and never waits for encoding. Overflow/invalid input
    poisons the episode; save cannot silently publish a partial recording.
    close() preserves unfinished files. recover=True archives unfinished input
    or rolls forward a checksummed commit; it never removes a saved episode.
    """
    def __init__(self, root, cameras, fps=30, *, resume=False, recover=False,
                 queue_size=32, max_episode_frames=18000, synthetic=False,
                 schema="a10_single_v1", device_metadata=None):
        self.schema = get_schema(schema)
        self.root = Path(root).resolve(); self.cameras = copy.deepcopy(cameras)
        if type(fps) is not int or fps != 30:
            raise ValueError('A10 recording requires fps=30')
        if set(cameras) != set(self.schema.cameras):
            raise ValueError(f'Expected cameras {self.schema.cameras}')
        for c in cameras.values():
            if any(type(c.get(k)) is not int or c[k] <= 0 or c[k] % 2 for k in ('width', 'height')):
                raise ValueError('Camera dimensions must be positive even integers')
        if type(queue_size) is not int or queue_size < 1 or type(max_episode_frames) is not int or max_episode_frames < 2:
            raise ValueError('Invalid queue size / episode frame limit')
        if type(synthetic) is not bool or (recover and not resume):
            raise ValueError('synthetic must be bool; recovery requires resume')
        if not resume and self.root.exists() and any(self.root.iterdir()):
            raise FileExistsError('Use an empty dataset directory; resume must be explicit')
        if resume and not (self.root / 'meta/recording_contract.json').is_file():
            raise ValueError('Resume requires this writer\'s recording contract; legacy datasets are read-only inputs')
        self.root.mkdir(parents=True, exist_ok=True)
        self._lock = (self.root / '.recording.lock').open('a+')
        try:
            fcntl.flock(self._lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            self._lock.close(); raise RuntimeError('Dataset is already in use')
        self.fps = fps; self.queue_size = queue_size; self.max_episode_frames = max_episode_frames
        self.active = None; self._thread = None; self._closed = False; self._error = None
        self._stage = self.root / '.recording/active'
        try:
            contract = dict(self.schema.contract(), cameras=self.cameras, fps=fps, synthetic=synthetic, video_codec='h264')
            if self.schema.dual:
                contract['device_metadata'] = copy.deepcopy(device_metadata or {})
            elif device_metadata is not None:
                raise ValueError('device_metadata is only supported by the dual schema')
            if resume:
                saved = json.loads((self.root / 'meta/recording_contract.json').read_text())
                if saved != contract: raise ValueError('Recording contract differs (units/cameras/fps/synthetic)')
                if (self.root / 'SIMULATED_DATA.json').exists() != synthetic:
                    raise ValueError('Synthetic marker does not match recording contract')
                if self._stage.exists():
                    if not recover: raise RuntimeError('Unfinished episode/commit found; inspect then use recover=True')
                    if (self._stage / 'commit.json').exists(): self._publish()
                    else:
                        self._stage.rename(self._stage.with_name(f'recovered-{time.time_ns()}'))
                self.info, self.episodes, self.tasks = load_v3(self.root)
                from .a10_recording_io import validate_dataset
                validate_dataset(self.root, check_videos=False, _locked=True)
            else:
                self.info = self._new_info(); self.episodes = []; self.tasks = {}
                json_write(self.root / 'meta/recording_contract.json', contract)
                if synthetic: json_write(self.root / 'SIMULATED_DATA.json', {'simulation': True, 'purpose': 'pipeline test'})
                self._write_metadata(self.root, self.info, self.tasks, {})
        except BaseException:
            self._lock.close(); raise

    def _new_info(self):
        features = {'observation.state': dict(dtype='float32', shape=[len(self.schema.state_names)], names=list(self.schema.state_names)),
                    'action': dict(dtype='float32', shape=[len(self.schema.action_names)], names=list(self.schema.action_names))}
        for key in ('timestamp', 'frame_index', 'episode_index', 'index', 'task_index'):
            features[key] = dict(dtype='float32' if key == 'timestamp' else 'int64', shape=[1], names=None)
        for name, c in self.cameras.items():
            features[f'observation.images.{name}'] = dict(dtype='video', shape=[c['height'], c['width'], 3],
                names=['height', 'width', 'channels'], info={'video.height': c['height'], 'video.width': c['width'],
                'video.codec': 'h264', 'video.pix_fmt': 'yuv420p', 'video.fps': self.fps,
                'video.channels': 3, 'video.is_depth_map': False, 'has_audio': False})
        return dict(**({'a10_recording_schema': self.schema.name} if self.schema.dual else {}),
                    codebase_version='v3.0', robot_type='a10_follower', total_episodes=0, total_frames=0,
                    total_tasks=0, chunks_size=1000, data_files_size_in_mb=100, video_files_size_in_mb=200,
                    fps=self.fps, features=features, splits={'train': '0:0'},
                    data_path='data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet',
                    video_path='videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4')

    @staticmethod
    def _write_metadata(root, info, tasks, statistics):
        root = Path(root); (root / 'meta').mkdir(parents=True, exist_ok=True)
        pd.DataFrame({'task_index': list(tasks)}, index=list(tasks.values())).astype({'task_index': 'int64'}).to_parquet(root / 'meta/tasks.parquet')
        json_write(root / 'meta/stats.json', statistics)
        json_write(root / 'meta/info.json', info)

    def start(self, task):
        if self._closed or self.active is not None or self._stage.exists():
            raise RuntimeError('Writer closed or previous episode unfinished')
        if not isinstance(task, str) or not task.strip(): raise ValueError('Task must be nonempty text')
        self.task = task.strip(); self._stage.mkdir(parents=True); self.active = self._stage
        json_write(self.active / 'episode.json', {'task': self.task, 'started_ns': time.time_ns()})
        self._queue = queue.Queue(self.queue_size); self._error = None; self._count = 0
        self._rows = []; self._image_stats = {}; self._last_capture = None
        self._thread = threading.Thread(target=self._worker, name='a10-episode-encoder', daemon=True)
        self._thread.start()

    def append(self, state, action, images, *, capture_monotonic_s=None, diagnostics=None):
        if self.active is None or self._closed or self._thread is None: raise RuntimeError('Start an episode first')
        if self._error is not None: raise RuntimeError('Episode failed; discard or preserve it') from self._error
        try:
            state = np.array(state, dtype=np.float32, copy=True); action = np.array(action, dtype=np.float32, copy=True)
            self.schema.validate_values(state, action)
            if set(images) != set(self.schema.cameras): raise ValueError('All configured RGB images are required')
            frames = {}
            for name, c in self.cameras.items():
                img = np.asarray(images[name])
                if img.dtype != np.uint8 or img.shape != (c['height'], c['width'], 3): raise ValueError(f'Invalid image: {name}')
                frames[name] = img.copy()
            if self._count >= self.max_episode_frames: raise ValueError('Episode frame limit reached')
            if capture_monotonic_s is not None:
                if not np.isfinite(capture_monotonic_s) or capture_monotonic_s < 0 or (self._last_capture is not None and capture_monotonic_s <= self._last_capture):
                    raise ValueError('Capture times must be finite and strictly increasing')
            detail = copy.deepcopy(diagnostics or {})
            json.dumps(detail, allow_nan=False)
            self._queue.put_nowait((state, action, frames, capture_monotonic_s, detail))
            self._count += 1
            if capture_monotonic_s is not None:
                self._last_capture = capture_monotonic_s
        except Exception as exc:
            self._error = exc; raise

    def _encode_frame(self, name, image, stream, container, index):
        frame = av.VideoFrame.from_ndarray(image, format='rgb24')
        frame.pts = index; frame.time_base = Fraction(1, self.fps)
        for packet in stream.encode(frame): container.mux(packet)

    def _worker(self):
        streams = {}; sidecar = None; finished = False
        try:
            for name, c in self.cameras.items():
                container = av.open(str(self.active / f'{name}.mp4'), 'w'); streams[name] = (container, None)
                stream = container.add_stream('libx264', rate=self.fps); streams[name] = (container, stream)
                stream.width, stream.height = c['width'], c['height']; stream.pix_fmt = 'yuv420p'
                stream.options = {'crf': '18', 'preset': 'ultrafast'}
                self._image_stats[name] = dict(sum=np.zeros(3), square=np.zeros(3), min=np.ones(3), max=np.zeros(3), pixels=0)
            sidecar = (self.active / 'diagnostics.jsonl').open('w')
            while True:
                item = self._queue.get()
                if item is None:
                    finished = True
                    break
                if self._error is not None: continue
                state, action, images, stamp, details = item; index = len(self._rows)
                for name, image in images.items():
                    container, stream = streams[name]; self._encode_frame(name, image, stream, container, index)
                    pixels = image.reshape(-1, 3).astype(np.float64) / 255.; s = self._image_stats[name]
                    s['sum'] += pixels.sum(axis=0); s['square'] += (pixels**2).sum(axis=0)
                    s['min'] = np.minimum(s['min'], pixels.min(axis=0)); s['max'] = np.maximum(s['max'], pixels.max(axis=0)); s['pixels'] += len(pixels)
                self._rows.append(dict(**{'observation.state': state.tolist(), 'action': action.tolist()},
                                       timestamp=index / self.fps, frame_index=index))
                sidecar.write(json.dumps(dict(frame_index=index, capture_monotonic_s=stamp, details=details), allow_nan=False) + '\n')
            if self._error is None:
                for container, stream in streams.values():
                    for packet in stream.encode(): container.mux(packet)
        except Exception as exc:
            self._error = exc
            # Drain until producer finalizes; bounded queue must not deadlock save/close.
            if not finished:
                while self._queue.get() is not None: pass
        finally:
            for container, _ in streams.values():
                try: container.close()
                except Exception as exc: self._error = self._error or exc
            if sidecar is not None: sidecar.close()

    def _finish(self):
        if self._thread is not None:
            self._queue.put(None); self._thread.join(); self._thread = None

    def save(self):
        if self.active is None or self._closed: raise RuntimeError('No active episode')
        if self._count < 2: raise ValueError('At least two frames are required')
        self._finish()
        if self._error: raise RuntimeError('Episode failed; no data committed') from self._error
        if len(self._rows) != self._count: raise RuntimeError('Incomplete encoder output')
        eid = len(self.episodes); count = self._count; start = self.info['total_frames']; tasks = dict(self.tasks)
        tid = next((k for k,v in tasks.items() if v == self.task), len(tasks)); tasks[tid] = self.task
        rows = [dict(r, episode_index=eid, task_index=tid, index=start+i) for i,r in enumerate(self._rows)]
        chunk, file = divmod(eid, 1000); data_rel = f'data/chunk-{chunk:03d}/file-{file:03d}.parquet'
        ep_rel = f'meta/episodes/chunk-{chunk:03d}/file-{file:03d}.parquet'
        ep = dict(episode_index=eid, length=count, tasks=[self.task], dataset_from_index=start, dataset_to_index=start+count,
                  **{'data/chunk_index':chunk, 'data/file_index':file, 'meta/episodes/chunk_index':chunk, 'meta/episodes/file_index':file})
        commit = self.active / 'publish'; commit.mkdir(exist_ok=True)
        (commit / data_rel).parent.mkdir(parents=True, exist_ok=True)
        pq.write_table(pa.Table.from_pylist(rows, schema=table_schema(self.schema.name)), commit / data_rel)
        statistics = {k: stats([r[k] for r in rows]) for k in rows[0]}
        for name in self.schema.cameras:
            key = f'observation.images.{name}'; rel = f'videos/{key}/chunk-{chunk:03d}/file-{file:03d}.mp4'
            # Decode completed video before committing it, including all timestamps.
            with av.open(str(self.active / f'{name}.mp4')) as video:
                stamps = [float(f.pts*f.time_base) for f in video.decode(video=0)]
            if len(stamps) != count or not np.allclose(stamps, np.arange(count)/self.fps, atol=1e-4):
                raise RuntimeError(f'Video frame/time mismatch: {name}')
            (commit / rel).parent.mkdir(parents=True, exist_ok=True)
            shutil.move(self.active / f'{name}.mp4', commit / rel)
            ep.update({f'videos/{key}/chunk_index':chunk, f'videos/{key}/file_index':file,
                       f'videos/{key}/from_timestamp':0., f'videos/{key}/to_timestamp':count/self.fps})
            s = self._image_stats[name]; mean = s['sum']/s['pixels']; std = np.sqrt(np.maximum(s['square']/s['pixels']-mean**2,0))
            statistics[key] = {k: np.asarray(v).reshape(3,1,1).tolist() for k,v in dict(mean=mean,std=std,min=s['min'],max=s['max']).items()}
            statistics[key]['count'] = [count]
        ep.update({f'stats/{k}/{n}':v for k,s in statistics.items() for n,v in s.items()})
        (commit / ep_rel).parent.mkdir(parents=True, exist_ok=True)
        pq.write_table(pa.Table.from_pylist([ep]), commit / ep_rel)
        diagnostic_rel = f'meta/xlevr_diagnostics/episode_{eid:06d}.jsonl'
        (commit / diagnostic_rel).parent.mkdir(parents=True, exist_ok=True)
        shutil.move(self.active / 'diagnostics.jsonl', commit / diagnostic_rel)
        all_stats = [episode_stats(e) for e in self.episodes] + [statistics]
        info = dict(self.info, total_episodes=eid+1, total_frames=start+count, total_tasks=len(tasks), splits={'train':f'0:{eid+1}'})
        self._write_metadata(commit, info, tasks, aggregate_stats(all_stats))
        files = sorted(p.relative_to(commit).as_posix() for p in commit.rglob('*') if p.is_file() and p.name!='info.json') + ['meta/info.json']
        entries = []
        for rel in files:
            dest = self.root / rel
            if dest.exists() and rel not in ('meta/tasks.parquet','meta/stats.json','meta/info.json'):
                raise FileExistsError(f'Will not overwrite saved data: {dest}')
            with (commit / rel).open('rb') as stream: os.fsync(stream.fileno())
            entries.append(dict(path=rel, sha256=digest(commit/rel), old_sha256=digest(dest) if dest.exists() else None))
        for directory in sorted((p for p in commit.rglob('*') if p.is_dir()), key=lambda p: len(p.parts), reverse=True):
            sync_dir(directory)
        sync_dir(commit)
        json_write(self.active / 'commit.json', entries)
        sync_dir(self._stage.parent)
        sync_dir(self.root)
        self._publish()
        self.info, self.episodes, self.tasks = load_v3(self.root); self.active = None
        return eid

    def _publish(self):
        entries = json.loads((self._stage/'commit.json').read_text())
        for entry in entries:
            rel = entry['path']; src = safe_path(self._stage/'publish', rel); dest = safe_path(self.root, rel)
            if src.exists():
                if digest(src) != entry['sha256']: raise RuntimeError('Staged commit checksum mismatch')
                old = digest(dest) if dest.exists() else None
                if old not in (entry['old_sha256'], entry['sha256']): raise RuntimeError('Destination changed during commit')
                dest.parent.mkdir(parents=True, exist_ok=True); os.replace(src,dest); sync_dir(dest.parent)
            elif not dest.is_file() or digest(dest) != entry['sha256']:
                raise RuntimeError('Cannot recover incomplete commit')
        shutil.rmtree(self._stage); sync_dir(self._stage.parent)

    def discard(self):
        if self.active is None: return
        self._finish()
        if (self.active/'commit.json').exists(): raise RuntimeError('Commit started; close and recover instead of discarding')
        shutil.rmtree(self.active); self.active = None; self._error = None

    def close(self):
        if not self._closed:
            self._finish(); self._lock.close(); self._closed = True

    def __enter__(self): return self
    def __exit__(self, *_exc): self.close()


def episode_stats(episode):
    result = {}
    for key,value in episode.items():
        if key.startswith('stats/'):
            feature,name = key[6:].rsplit('/',1); result.setdefault(feature,{})[name] = value
    return result
