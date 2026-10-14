#!/usr/bin/env python
"""Show progress predictions at human task events, without rerunning SARM."""
import json
from pathlib import Path

import av
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
COLORS = ['#377eb8', '#e89e40', '#bd78b4', '#41b7b5']


def main():
    root = ROOT / 'datasets/a10_sarm_val'
    prediction = ROOT / 'sarm_runs/a10/validation_center/progress.jsonl'
    output = ROOT / 'sarm_runs/a10/validation_center/event_comparison'
    output.mkdir(exist_ok=True)
    available = {f.name for f in font_manager.fontManager.ttflist}
    for name in ('Noto Sans CJK SC','Noto Sans CJK JP','WenQuanYi Micro Hei'):
        if name in available:
            plt.rcParams['font.family'] = [name,'DejaVu Sans'];break
    plt.rcParams['axes.unicode_minus'] = False
    info = json.loads((root/'meta/info.json').read_text())
    episodes = [json.loads(x) for x in (root/'meta/episodes.jsonl').read_text().splitlines()]
    predictions = [json.loads(x) for x in prediction.read_text().splitlines()]
    priors = json.loads((ROOT/'datasets/a10_sarm_train/meta/temporal_proportions_dense.json').read_text())
    boundaries = np.r_[0,np.cumsum(list(priors.values()))]
    summaries=[]
    for e in episodes:
        ep=e['episode_index'];fps=info['fps'];n=e['length']
        records=sorted([r for r in predictions if r['episode_index']==ep],key=lambda r:r['frame_index'])
        assert len(records)==n and [r['frame_index'] for r in records]==list(range(n))
        assert all(r['frame_alignment']=='center' and r['source_episode_index']==e['source_episode_index'] for r in records)
        pred=np.array([r['pred_progress'] for r in records]);reference=np.empty(n)
        starts,ends=e['dense_subtask_start_frames'],e['dense_subtask_end_frames']
        for k,(s,z) in enumerate(zip(starts,ends)):
            tau=np.linspace(0,1,z-s+1) if z>s else np.ones(1)
            reference[s:z+1]=boundaries[k]+tau*(boundaries[k+1]-boundaries[k])
        anchors=starts+[n-1]
        path=root/info['video_path'].format(episode_index=ep,episode_chunk=0,video_key='observation.images.base_0_rgb')
        frames={}
        with av.open(str(path)) as c:
            for i,f in enumerate(c.decode(video=0)):
                if i in anchors:frames[i]=f.to_image()
        fig=plt.figure(figsize=(17,8),layout='constrained');grid=fig.add_gridspec(2,5,height_ratios=[2.1,1.2])
        ax=fig.add_subplot(grid[0,:]);times=np.arange(n)/fps
        for k,(s,z) in enumerate(zip(starts,ends)):
            ax.axvspan(s/fps,(z+1)/fps,color=COLORS[k],alpha=.12)
            ax.text((s+z)/2/fps,1.04,e['dense_subtask_names'][k],ha='center',fontsize=10)
        ax.plot(times,pred,color='#1c638b',label='SARM 预测',linewidth=1.6)
        ax.plot(times,reference,color='#388c45',linestyle='--',label='人工阶段边界 + 时间插值参考',linewidth=1.5)
        for k,index in enumerate(anchors):
            ax.scatter(index/fps,pred[index],color='#1c638b',s=40,zorder=4)
            ax.axvline(index/fps,color='grey',alpha=.5,linewidth=.7)
        ax.set(xlim=(0,n/fps),ylim=(-.04,1.12),xlabel='视频时间（秒）',ylabel='Progress（阶段加权后的进度）',
               title=f'验证集 {ep} / 原始集 {e["source_episode_index"]}：把预测和实际阶段事件放在一起看')
        ax.legend(loc='upper left');ax.grid(alpha=.2)
        titles=['开始','抓取→搬运','放瓶→关盖','关盖→归位','最后一帧']
        points=[]
        for k,index in enumerate(anchors):
            a=fig.add_subplot(grid[1,k]);a.imshow(frames[index]);a.axis('off')
            a.set_title(f'{titles[k]} · {index/fps:.2f}s\n预测 {pred[index]:.1%} / 参考 {boundaries[k]:.1%}',fontsize=11)
            points.append(dict(event=titles[k],frame_index=index,seconds=index/fps,
                               predicted_progress=float(pred[index]),reference_progress=float(boundaries[k])))
        fig.savefig(output/f'episode_{ep:06d}_events.png',dpi=130);plt.close(fig)
        summaries.append(dict(episode_index=ep,source_episode_index=e['source_episode_index'],events=points,
                              mean_absolute_progress_difference=float(np.abs(pred-reference).mean())))
    (output/'event_comparison.json').write_text(json.dumps(dict(predictions=str(prediction.relative_to(ROOT)),
        reference_definition='Linear time interpolation inside human stages, weighted by training stage-duration priors; not independently labeled physical completion or success probability',
        stage_progress_breakpoints=boundaries.tolist(),episodes=summaries),ensure_ascii=False,indent=2)+'\n')
    print(f'Saved {len(summaries)} event comparison plots to {output}')


if __name__=='__main__':
    main()
