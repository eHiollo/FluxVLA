"""Boundary, persistence, video HTTP and SARM consumer checks."""

import importlib.util
import io
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]


def load_module(name, filename):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'tools/sarm_annotate' / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


annotator = load_module('manual_annotator', 'manual_video_annotator.py')


@pytest.fixture
def store(tmp_path):
    dataset = tmp_path / 'dataset'
    meta = dataset / 'meta'
    meta.mkdir(parents=True)
    (meta / 'info.json').write_text(json.dumps(dict(
        codebase_version='v2.1', fps=30, chunks_size=1000,
        features={'observation.images.base': {'dtype': 'video'}},
        video_path='videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4')))
    (meta / 'episodes.jsonl').write_text('\n'.join(json.dumps(dict(
        episode_index=i, length=8, tasks=['pick_and_place'])) for i in range(2)))
    videos = dataset / 'videos/chunk-000/observation.images.base'
    videos.mkdir(parents=True)
    for i in range(2):
        (videos / f'episode_{i:06d}.mp4').write_bytes(b'0123456789')
    return annotator.AnnotationStore(dataset, tmp_path / 'stages.json')


def test_boundaries_and_completion(store):
    store.update('setup', {'names': ['接近', '抓取', '放置']})
    for cuts in ([2], [2, 2], [5, 2], [-1, 4], [2, 7], [1, 2, 3]):
        with pytest.raises(ValueError):
            store.update('save', {'episode_index': 0, 'cuts': cuts})
    store.update('save', {'episode_index': 0, 'cuts': [0, 6]})
    dense = json.loads(store.output.read_text())[0]['dense']
    assert dense == dict(names=['接近', '抓取', '放置'],
                         start_frames=[0, 1, 7], end_frames=[0, 6, 7])


def test_resume_reset_and_stage_changes(store):
    store.update('setup', {'names': ['抓取', '放置']})
    store.update('save', {'episode_index': 0, 'cuts': [3]})
    store.update('draft', {'episode_index': 1, 'cuts': [4], 'frame': 5})
    resumed = annotator.AnnotationStore(store.root, store.output)
    assert resumed.current == 1
    assert resumed.drafts['1'] == dict(cuts=[4], frame=5)
    resumed.update('reset', {'episode_index': 1})
    assert len(json.loads(store.output.read_text())) == 1
    with pytest.raises(ValueError):
        resumed.update('setup', {'names': ['新阶段']})
    resumed.update('setup', {'names': ['新阶段'], 'reset': True})
    resumed.update('save', {'episode_index': 0, 'cuts': []})
    assert resumed.annotations[0]['dense']['end_frames'] == [7]


def test_skip_resume_and_restore(store):
    store.update('setup', {'names': ['抓取', '放置']})
    store.update('save', {'episode_index': 0, 'cuts': [3]})
    store.update('save', {'episode_index': 1, 'cuts': [4]})
    store.update('skip', {'episode_index': 1})
    assert [entry['episode_index'] for entry in json.loads(store.output.read_text())] == [0]
    resumed = annotator.AnnotationStore(store.root, store.output)
    assert resumed.state()['skipped'] == [1]
    resumed.update('draft', {'episode_index': 1, 'cuts': [], 'frame': 0})
    assert '1' not in resumed.drafts
    with pytest.raises(ValueError):
        resumed.update('save', {'episode_index': 1, 'cuts': [4]})
    resumed.update('reset', {'episode_index': 1})
    assert resumed.state()['skipped'] == []
    resumed.update('save', {'episode_index': 1, 'cuts': [4]})
    assert len(resumed.annotations) == 2
    resumed.update('skip', {'episode_index': 1})
    resumed.update('setup', {'names': ['完整任务'], 'reset': True})
    assert resumed.state()['skipped'] == []


def test_skip_incomplete_episode(store):
    store.update('setup', {'names': ['抓取', '搬运', '放置']})
    store.update('draft', {'episode_index': 0, 'cuts': [2], 'frame': 3})
    store.update('skip', {'episode_index': 0})
    assert store.state()['skipped'] == [0]
    assert json.loads(store.output.read_text()) == []
    assert '0' not in store.drafts


def test_sarm_writer_compatibility(store):
    writer = load_module('manual_writer', 'write_manual_stages.py')
    store.update('setup', {'names': ['抓取', '放置']})
    for i in range(2):
        store.update('save', {'episode_index': i, 'cuts': [3]})
    sparse, dense, updated = writer._rewrite_jsonl(
        store.root / 'meta/episodes.jsonl', writer._load_spec(store.output), 'auto', None)
    records = [json.loads(line) for line in
               (store.root / 'meta/episodes.jsonl').read_text().splitlines()]
    assert updated == 2
    assert records[0]['dense_subtask_start_frames'] == [0, 4]
    assert records[1]['dense_subtask_end_frames'] == [3, 7]
    assert records[0]['sparse_subtask_names'] == ['task']
    assert dict(dense) == {'抓取': 8, '放置': 8}


def test_http_video_ranges_and_errors(store):
    # Exercise the HTTP parser and handler with an in-memory connection,
    # respecting the repository's no-network unit-test fixture.
    class Connection:
        def __init__(self, request):
            self.request = io.BytesIO(request)
            self.response = bytearray()

        def makefile(self, *_):
            return self.request

        def sendall(self, data):
            self.response.extend(data)

    def request(method, path, headers='', body=b''):
        raw = (f'{method} {path} HTTP/1.0\r\nHost: localhost\r\n'
               f'Content-Length: {len(body)}\r\n{headers}\r\n').encode() + body
        connection = Connection(raw)
        annotator.make_handler(store)(connection, ('127.0.0.1', 0), None)
        return bytes(connection.response).split(b'\r\n\r\n', 1)

    video = '/video?episode=0&camera=observation.images.base'
    headers, body = request('GET', video, 'Range: bytes=2-5\r\n')
    assert b'206' in headers.splitlines()[0]
    assert b'Content-Range: bytes 2-5/10' in headers
    assert body == b'2345'
    assert request('GET', video, 'Range: bytes=-3\r\n')[1] == b'789'
    assert b'416' in request('GET', video, 'Range: bytes=20-30\r\n')[0]
    assert '空格' in request('GET', '/')[1].decode()
    data = json.dumps({'names': ['抓取', '放置']}).encode()
    assert json.loads(request('POST', '/api/setup', body=data)[1])['names'] == ['抓取', '放置']
    assert b'403' in request('POST', '/api/setup', 'Origin: http://foreign.example\r\n', data)[0]


def review_fixture(store, tmp_path):
    store.update('setup', {'names': ['抓取', '放置']})
    for i in range(2):
        store.update('save', {'episode_index': i, 'cuts': [3]})
    meta = tmp_path / 'validation/meta'
    meta.mkdir(parents=True)
    (meta / 'info.json').write_text(json.dumps({'fps': 30}))
    records = []
    predictions = []
    for validation_id, original_id in enumerate([1, 0]):
        dense = store.annotations[original_id]['dense']
        records.append(dict(episode_index=validation_id, source_episode_index=original_id,
                            length=8, dense_subtask_names=dense['names'],
                            dense_subtask_start_frames=dense['start_frames'],
                            dense_subtask_end_frames=dense['end_frames']))
        for frame in range(8):
            predictions.append(dict(episode_index=validation_id,
                                    source_episode_index=original_id,
                                    frame_index=frame, current_index=validation_id * 8 + frame,
                                    frame_alignment='center',
                                    pred_progress=frame / 7 if original_id else 1 - frame / 7))
    (meta / 'episodes.jsonl').write_text('\n'.join(map(json.dumps, records)))
    (meta / 'temporal_proportions_dense.json').write_text(
        json.dumps({'抓取': .4, '放置': .6}))
    path = tmp_path / 'predictions.jsonl'
    path.write_text('\n'.join(map(json.dumps, predictions)))
    return meta.parent, path, predictions


def test_review_mapping_reference_and_read_only(store, tmp_path):
    root, path, _ = review_fixture(store, tmp_path)
    before = store.output.read_bytes(), store.session_path.read_bytes()
    review = annotator.ReviewStore(store.root, store.output, path, root)
    assert review.state()['review'] is True
    assert review.review_data[1]['predicted'][0] == 0
    assert review.review_data[0]['predicted'][0] == 1
    review.update('select', {'episode_index': 1})
    for action in ['setup', 'draft', 'save', 'reset', 'skip']:
        with pytest.raises(PermissionError):
            review.update(action, {'episode_index': 1})
    assert before == (store.output.read_bytes(), store.session_path.read_bytes())
    spec = importlib.util.spec_from_file_location(
        'review_sarm_utils', ROOT / 'fluxvla/datasets/utils/sarm_utils.py')
    utils = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(utils)
    dense = store.annotations[1]['dense']
    for frame, actual in enumerate(review.review_data[1]['reference']):
        encoded = utils.find_stage_and_tau(frame, 8, dense['names'], dense['start_frames'],
                                           dense['end_frames'], dense['names'], None, True)
        expected = utils.normalize_stage_tau(encoded, temporal_proportions=[.4, .6])
        assert actual == pytest.approx(expected)


@pytest.mark.parametrize('invalid', ['legacy', 'duplicate', 'missing', 'nonfinite', 'wrong_id', 'wrong_index'])
def test_review_rejects_misaligned_or_incomplete_predictions(store, tmp_path, invalid):
    root, path, rows = review_fixture(store, tmp_path)
    if invalid == 'legacy':
        rows[0].pop('frame_alignment')
    elif invalid == 'duplicate':
        rows.append(rows[0])
    elif invalid == 'missing':
        rows.pop()
    elif invalid == 'nonfinite':
        rows[0]['pred_progress'] = float('nan')
    elif invalid == 'wrong_id':
        rows[0]['source_episode_index'] = 0
    else:
        rows[0]['current_index'] = 1
    path.write_text('\n'.join(map(json.dumps, rows)))
    with pytest.raises(ValueError):
        annotator.ReviewStore(store.root, store.output, path, root)


def test_legacy_review_gaps_and_last_frame_selection(store, tmp_path):
    root, path, rows = review_fixture(store, tmp_path)
    for row in rows:
        for key in ['frame_index', 'source_episode_index', 'frame_alignment']:
            row.pop(key)
    path.write_text('\n'.join(map(json.dumps, reversed(rows))))
    review = annotator.ReviewStore(store.root, store.output, path, root, legacy_frame_offset=2)
    assert review.state()['frame_alignment'] == 'window_end'
    assert review.review_data[1]['predicted'][:2] == [None, None]
    assert review.review_data[1]['predicted'][2] == 0
    assert review.review_data[1]['predicted'][-1] == 1
    assert review.review_data[0]['predicted'][-1] == 0
