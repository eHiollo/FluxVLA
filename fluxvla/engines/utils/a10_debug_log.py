"""Bounded asynchronous A10 evidence logs; never perform disk I/O per target."""

import json
import hashlib
import os
import subprocess
import logging
from pathlib import Path
import queue
import sys
import threading
import time
import uuid
from datetime import datetime, timezone

import numpy as np


def _copy(value):
    if hasattr(value, 'detach'):
        return value.detach().cpu().numpy().copy()
    if isinstance(value, np.ndarray):
        return value.copy()
    if isinstance(value, dict):
        return {str(k): _copy(v) for k, v in value.items()
                if str(k).lower() not in ('password', 'api_key', 'access_token', 'secret')}
    if isinstance(value, (list, tuple)):
        return [_copy(v) for v in value]
    return value


def _json(value):
    if isinstance(value, np.ndarray):
        return _json(value.tolist())
    if isinstance(value, np.generic):
        return _json(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {str(k): _json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json(v) for v in value]
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    return str(value)


class _LogHandler(logging.Handler):
    def __init__(self, recorder):
        super().__init__(logging.INFO)
        self.recorder = recorder
        self.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(name)s %(message)s'))

    def emit(self, record):
        self.recorder.record('console', text=self.format(record))


class A10DebugLog:
    def __init__(self, *, role, root_dir='work_dirs/a10/inference_logs',
                 metadata=None, image_every_n_cycles=5, queue_size=128, **unused):
        if type(image_every_n_cycles) is not int or image_every_n_cycles < 0:
            raise ValueError('image_every_n_cycles must be a nonnegative integer')
        if type(queue_size) is not int or queue_size <= 0:
            raise ValueError('queue_size must be a positive integer')
        self.run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ') + '_' + uuid.uuid4().hex[:8]
        self.path = Path(root_dir).expanduser().resolve() / (self.run_id + '_' + role)
        self.path.mkdir(parents=True)
        self.image_every_n_cycles = image_every_n_cycles
        self.dropped = 0
        self.write_errors = 0
        self.closed = False
        self.metadata = _copy(dict(metadata or {}))
        source_root = Path(__file__).resolve().parents[3]
        source_hashes = {}
        for name in ('fluxvla/engines/runners/a10_inference_runner.py',
                     'fluxvla/engines/operators/a10_operator.py',
                     'fluxvla/engines/runners/serving/zmq_server.py',
                     'fluxvla/datasets/a10_openpi_inference.py',
                     'fluxvla/transforms/a10_openpi.py'):
            source = source_root / name
            if source.is_file():
                source_hashes[name] = hashlib.sha256(source.read_bytes()).hexdigest()
        try:
            revision = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=source_root,
                                      capture_output=True, text=True, timeout=2).stdout.strip()
        except (OSError, subprocess.TimeoutExpired):
            revision = None
        self.metadata['runtime'] = dict(python=sys.version, numpy=np.__version__,
                                        torch=getattr(sys.modules.get('torch'), '__version__', None),
                                        pid=os.getpid(), git_revision=revision,
                                        source_sha256=source_hashes)
        manifest = dict(schema_version=1, run_id=self.run_id, role=role,
                        started_utc=datetime.now(timezone.utc).isoformat(),
                        clock='monotonic_s is host-local; feedback times are receive times',
                        units=dict(joints='rad', gripper='mm', time='s'),
                        image_every_n_cycles=image_every_n_cycles, metadata=self.metadata)
        (self.path / 'metadata.json').write_text(json.dumps(_json(manifest), ensure_ascii=False, indent=2))
        self._queue = queue.Queue(maxsize=queue_size)
        self._thread = threading.Thread(target=self._write_loop, name='a10-debug-writer', daemon=True)
        self._thread.start()
        self._handler = _LogHandler(self)
        logging.getLogger().addHandler(self._handler)

    def record(self, event, *, images=None, **data):
        if self.closed:
            return
        item = dict(event=event, utc=datetime.now(timezone.utc).isoformat(),
                    monotonic_s=time.monotonic(), **_copy(data))
        try:
            self._queue.put_nowait((item, _copy(images)))
        except queue.Full:
            self.dropped += 1

    def _write_loop(self):
        try:
            self._drain()
        except Exception as exc:
            self.write_errors += 1
            sys.stderr.write(f'A10 debug writer failed: {exc}\n')

    def _drain(self):
        with (self.path / 'events.jsonl').open('a', buffering=1) as events, \
                (self.path / 'console.log').open('a', buffering=1) as console:
            while True:
                item = self._queue.get()
                if item is None:
                    break
                event, images = item
                try:
                    if images:
                        from PIL import Image
                        directory = self.path / 'frames'
                        directory.mkdir(exist_ok=True)
                        paths = {}
                        for name, pixels in images.items():
                            filename = f"{event.get('cycle', 0):06d}_{name}.png"
                            Image.fromarray(pixels).save(directory / filename)
                            paths[name] = 'frames/' + filename
                        event['image_files'] = paths
                    if event['event'] == 'console':
                        console.write(event['text'] + '\n')
                    else:
                        events.write(json.dumps(_json(event), ensure_ascii=False, allow_nan=False) + '\n')
                except Exception as exc:
                    self.write_errors += 1
                    if self.write_errors == 1:
                        sys.stderr.write(f'A10 debug write failed: {exc}\n')
            summary = dict(run_id=self.run_id, closed_utc=datetime.now(timezone.utc).isoformat(),
                           dropped_events=self.dropped, write_errors=self.write_errors,
                           queue_remaining=self._queue.qsize())
            (self.path / 'summary.json').write_text(json.dumps(summary, indent=2))

    def close(self):
        if self.closed:
            return
        self.closed = True
        logging.getLogger().removeHandler(self._handler)
        self._handler.close()
        try:
            self._queue.put(None, timeout=1)
        except queue.Full:
            sys.stderr.write('A10 debug writer backlog; log drain not confirmed\n')
            return
        self._thread.join(timeout=5)
        if self._thread.is_alive():
            sys.stderr.write('A10 debug log drain not confirmed\n')


def create_a10_debug_log(config, *, role, metadata=None):
    if not config or not config.get('enabled', True):
        return None
    options = dict(config)
    options.pop('enabled', None)
    try:
        recorder = A10DebugLog(role=role, metadata=metadata, **options)
    except Exception as exc:
        logging.getLogger(__name__).warning('A10 debug logging unavailable: %s', exc)
        return None
    logging.getLogger(__name__).info('A10 debug log directory: %s', recorder.path)
    return recorder
