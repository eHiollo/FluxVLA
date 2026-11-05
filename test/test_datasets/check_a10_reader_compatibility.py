"""Opt-in smoke test in the real LeRobot/OpenPI environment, using synthetic data.

Run with --reader v3 or --reader openpi --openpi-repo /path/to/openpi.
Input must be the 2 x 40-frame fixture from test_a10_recording.make_fixture.
No weights/training or hardware are started; norm stats go to a new temp folder.
"""
import argparse
import dataclasses
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reader', choices=('v3','openpi'), required=True)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--openpi-repo', type=Path)
    args = parser.parse_args()
    root = args.dataset.resolve()
    if not (root/'SIMULATED_DATA.json').is_file(): raise ValueError('Synthetic fixture required')
    os.environ['HF_HUB_OFFLINE']='1'; os.environ['HF_DATASETS_OFFLINE']='1'
    os.environ['JAX_PLATFORMS']='cpu'
    os.environ['HF_LEROBOT_HOME']=str(root.parent)
    if args.reader=='v3':
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        image_keys=['observation.images.base_0_rgb','observation.images.right_wrist_0_rgb']
    else:
        from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
        image_keys=['observation.images.top','observation.images.right']
    dataset=LeRobotDataset(root.name,root=root,video_backend='pyav',
        delta_timestamps={'observation.state':[i/30 for i in range(30)]})
    assert len(dataset)==80
    for index in (0,39,40,79):
        row=dataset[index]; local=index%40
        expected=np.array([[min(local+t,39)/100]*6+[40+min(local+t,39)/100] for t in range(30)])
        np.testing.assert_allclose(row['observation.state'],expected,atol=1e-5)
        assert tuple(row['action'].shape)==(8,)
        assert row['task']==('Pick up the bottle' if index<40 else 'Put down the bottle')
        for key in image_keys: assert tuple(row[key].shape)==(3,16,16)
        # Different image brightness proves the two camera aliases weren't swapped.
        assert float(row[image_keys[1]].mean())>float(row[image_keys[0]].mean())+.2
    report=dict(reader=args.reader,frames=len(dataset),boundaries_checked=[0,39,40,79],horizon=30)
    if args.reader=='openpi':
        if args.openpi_repo is None: parser.error('--openpi-repo required')
        sys.path.insert(0,str(args.openpi_repo/'src'))
        import openpi.training.config as config
        import openpi.training.data_loader as dl
        import openpi.shared.normalize as normalize
        cfg=config.get_config('pi05_a10_finetune')
        # Only alter the test's in-memory dataset choice; repository configs remain untouched.
        factory=dataclasses.replace(cfg.data,repo_id=root.name,
            base_config=config.DataConfig(prompt_from_task=True))
        output=Path(tempfile.mkdtemp(prefix='a10_openpi_norm_',dir='/tmp'))
        data=factory.create(output,cfg.model)
        raw=dl.create_torch_dataset(data,30,cfg.model)
        assert raw[0]['prompt']=='Pick up the bottle'
        transformed=dl.TransformedDataset(raw,[*data.repack_transforms.inputs,*data.data_transforms.inputs])
        for index in (0,39,40,79):
            sample=transformed[index]
            assert sample['state'].shape==(7,) and sample['actions'].shape==(30,7)
            np.testing.assert_allclose(sample['actions'][0,:6],0,atol=1e-6)
            np.testing.assert_allclose(sample['actions'][0,-1],sample['state'][-1],atol=1e-6)
            if index==0: np.testing.assert_allclose(sample['actions'][10,:6],.1,atol=1e-6)
            assert not sample['image_mask']['left_wrist_0_rgb']
            assert sample['image_mask']['base_0_rgb'] and sample['image_mask']['right_wrist_0_rgb']
        spec=importlib.util.spec_from_file_location('compute_norm_stats',args.openpi_repo/'scripts/compute_norm_stats.py')
        module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        loader,batches=module.create_torch_dataloader(data,30,8,cfg.model,0,max_frames=80)
        running={k:normalize.RunningStats() for k in ('state','actions')}
        n=0
        for batch in loader:
            for key in running: running[key].update(np.asarray(batch[key]))
            n+=1
        assert n==batches==10
        norm={k:v.get_statistics() for k,v in running.items()}
        normalize.save(output,norm)
        for st in norm.values():
            assert st.mean.shape==(7,) and np.isfinite(st.mean).all()
            assert np.isfinite(st.q01).all() and np.isfinite(st.q99).all()
        # Also exercise the actual model input transforms (tokenization, resizing and padding), no model execution.
        full=dl.transform_dataset(raw,dataclasses.replace(data,norm_stats=norm))[0]
        assert full['state'].shape==(32,) and full['actions'].shape==(30,32)
        assert full['image']['base_0_rgb'].shape==(224,224,3)
        report.update(norm_batches=n,norm_stats=str(output/'norm_stats.json'),padded_action_shape=[30,32],
            prompt_behavior='Current OpenPI A10 Repack drops task prompt; model uses default_prompt')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__': main()
