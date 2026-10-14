#!/usr/bin/env python
"""Compare Qwen and human A10 annotations at identical original episode IDs."""
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'sarm_runs/a10_qwen4b'
BOUNDARIES = ['Start putting bottle in box', 'Start closing lid', 'Start returning home']


def compare():
    manual_path = ROOT / 'datasets/a10_stages.json'
    qwen_path = ROOT / 'datasets/a10_sarm_v3_qwen_4b_labels.json'
    manual_rows, qwen_rows = (json.loads(p.read_text()) for p in (manual_path, qwen_path))
    manual = {e['episode_index']: e['dense'] for e in manual_rows}
    qwen = {e['episode_index']: e for e in qwen_rows}
    assert len(manual) == len(manual_rows) and len(qwen) == len(qwen_rows)
    assert set(manual) == set(qwen)
    fps = json.loads((ROOT / 'datasets/a10_sarm_v3/meta/info.json').read_text())['fps']
    names = manual_rows[0]['dense']['names']
    errors, records, intersections, total_frames = [], [], [], 0
    ious, manual_durations, qwen_durations = [], [], []
    video_matches = []
    for ep in sorted(manual):
        m, q = manual[ep], qwen[ep]['dense']
        assert m['names'] == q['names'] == names
        assert m['start_frames'][0] == q['start_frames'][0] == 0
        assert m['end_frames'][-1] == q['end_frames'][-1]
        n = m['end_frames'][-1] + 1
        ms, me, qs, qe = (np.array(x, dtype=int) for x in
                          (m['start_frames'],m['end_frames'],q['start_frames'],q['end_frames']))
        for s,e in [(ms,me),(qs,qe)]:
            assert len(s) == len(e) == 4 and np.all(s <= e)
            assert np.array_equal(s[1:],e[:-1]+1)
        error = (qs[1:]-ms[1:])/fps
        overlap = np.maximum(0, np.minimum(me,qe)-np.maximum(ms,qs)+1)
        union = np.maximum(me,qe)-np.minimum(ms,qs)+1
        accuracy = float(overlap.sum()/n)
        errors.append(error); ious.append(overlap/union)
        manual_durations.append((me-ms+1)/fps); qwen_durations.append((qe-qs+1)/fps)
        intersections.append(int(overlap.sum())); total_frames += n
        record = dict(source_episode_index=ep, split=qwen[ep]['split'],
            frame_count=n, manual_start_seconds=(ms/fps).tolist(),
            qwen_start_seconds=(qs/fps).tolist(), signed_boundary_errors_seconds=error.tolist(),
            mean_absolute_boundary_error_seconds=float(np.abs(error).mean()),
            max_absolute_boundary_error_seconds=float(np.abs(error).max()),
            frame_stage_agreement=accuracy, stage_iou=(overlap/union).tolist(),
            qwen_review_required=qwen[ep].get('review_required',False))
        records.append(record)
        # Human metadata refers to the old v2 source. Ensure the converted
        # split presents the exact same camera video, rather than another ID.
        old = ROOT / f'datasets/a10_9_21_sarm/videos/chunk-000/observation.images.base_0_rgb/episode_{ep:06d}.mp4'
        new = ROOT / 'datasets' / qwen[ep]['split'] / f'videos/observation.images.base_0_rgb/chunk-000/file-{qwen[ep]["split_episode_index"]:03d}.mp4'
        video_matches.append(hashlib.sha256(old.read_bytes()).digest() == hashlib.sha256(new.read_bytes()).digest())
    if not all(video_matches):
        raise ValueError('Some source videos differ; compare decoded frames before interpreting label differences')
    errors = np.array(errors); absolute = np.abs(errors)
    def metrics(values):
        v = np.asarray(values)
        return dict(mean_absolute_error_seconds=float(np.abs(v).mean()),
            median_absolute_error_seconds=float(np.median(np.abs(v))),
            p90_absolute_error_seconds=float(np.quantile(np.abs(v),.9)),
            maximum_absolute_error_seconds=float(np.abs(v).max()),
            mean_signed_error_seconds=float(v.mean()),
            within_1_second=float(np.mean(np.abs(v)<=1)),
            within_2_seconds=float(np.mean(np.abs(v)<=2)),
            greater_than_5_seconds=int(np.sum(np.abs(v)>5)))
    result = dict(reference='Human annotations; differences do not by themselves establish which label is correct',
        sources=dict(manual=str(manual_path.relative_to(ROOT)),qwen=str(qwen_path.relative_to(ROOT))),
        episodes=len(records),boundary_count=int(errors.size),fps=fps,total_frames=total_frames,
        exact_source_camera_video_matches=sum(video_matches),overall=metrics(errors),
        frame_stage_agreement=float(sum(intersections)/total_frames),
        mean_episode_frame_stage_agreement=float(np.mean([r['frame_stage_agreement'] for r in records])),
        transitions=[dict(name=names[i+1],**metrics(errors[:,i])) for i in range(3)],
        stage_mean_iou=dict(zip(names,np.mean(ious,axis=0).tolist())),
        stage_total_manual_seconds=dict(zip(names,np.sum(manual_durations,axis=0).tolist())),
        stage_total_qwen_seconds=dict(zip(names,np.sum(qwen_durations,axis=0).tolist())),
        episodes_with_any_boundary_over_2_seconds=int(np.sum(absolute.max(axis=1)>2)),
        episodes_with_any_boundary_over_5_seconds=int(np.sum(absolute.max(axis=1)>5)),
        worst_episodes=sorted(records,key=lambda r:r['mean_absolute_boundary_error_seconds'],reverse=True)[:5],
        per_episode=records)
    OUT.mkdir(parents=True,exist_ok=True)
    output_path=OUT/'comparison_qwen_4b_manual.json'
    output_path.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    fig,axes=plt.subplots(1,2,figsize=(13,5),layout='constrained')
    for j,color in enumerate(['#377eb8','#ff9933','#6d55a3']):
        axes[0].scatter([r['source_episode_index'] for r in records],errors[:,j],
                        label=BOUNDARIES[j],color=color,s=34,alpha=.8)
    axes[0].axhline(0,color='black',linewidth=.8)
    axes[0].axhspan(-2,2,color='#77a077',alpha=.12)
    axes[0].set(xlabel='Original episode index',ylabel='Qwen - human boundary (seconds)',
                title='Positive values mean Qwen transitions later')
    axes[0].legend(fontsize=8)
    axes[1].boxplot([absolute[:,j] for j in range(3)],tick_labels=['Put in box','Close lid','Return home'],showmeans=True)
    axes[1].set(ylabel='Absolute boundary error (seconds)',title='Boundary differences across 29 identical videos')
    fig.savefig(OUT/'comparison_qwen_4b_manual.png',dpi=160)
    plt.close(fig)
    # A companion notebook reuses the exact check implementation, with no
    # duplicated or separately maintained definitions of the metrics.
    code="import sys\nsys.path.insert(0, 'tools/sarm_annotate')\nfrom compare_a10_qwen4b_manual import compare\nresult = compare()\n"
    nb=dict(nbformat=4,nbformat_minor=5,metadata={'kernelspec':{'name':'python3','display_name':'Python 3','language':'python'}},cells=[
        dict(cell_type='markdown',metadata={},source=['# Qwen 4B vs human A10 labels\nRun from the FluxVLA root using the fluxvla Python environment. Compare by original episode ID; positive errors mean Qwen is later.']),
        dict(cell_type='code',metadata={},execution_count=None,source=code.splitlines(True),outputs=[])])
    (OUT/'comparison_qwen_4b_manual.ipynb').write_text(json.dumps(nb,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='per_episode'},ensure_ascii=False,indent=2))
    return result


if __name__=='__main__':
    compare()
