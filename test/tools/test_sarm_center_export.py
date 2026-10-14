"""Center-frame export must preserve global/local/source index correspondence."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
import torch

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('sarm_infer', ROOT / 'scripts/infer_sarm_progress.py')
infer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(infer)


def test_metadata_has_local_frame_zero_for_later_episode():
    dataset = SimpleNamespace(
        _get_dataset_index=lambda index: 0,
        episode_ranges={(0, 0): (0, 942), (0, 1): (942, 1891)},
        episodes_meta=[{0: {'source_episode_index': 7}, 1: {'source_episode_index': 11}}])
    assert infer._prediction_frame_metadata(dataset, 1, 942) == {
        'frame_index': 0, 'source_episode_index': 11, 'frame_alignment': 'center'}
    assert infer._prediction_frame_metadata(dataset, 1, 1890)['frame_index'] == 948
    with pytest.raises(ValueError):
        infer._prediction_frame_metadata(dataset, 1, 1891)


def test_center_is_current_even_at_episode_edges():
    # Load utility by file to avoid importing optional VLA runtimes.
    spec = importlib.util.spec_from_file_location('sarm_utils_direct', ROOT / 'fluxvla/datasets/utils/sarm_utils.py')
    utils = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(utils)
    for current in [942, 943, 1062, 1300, 1890]:
        indices, _ = utils.compute_absolute_indices(current, 942, 1891, 8, 30)
        assert int(indices[4]) == current


def test_main_passes_shared_center_to_model_and_visuals():
    # Execute real main orchestration with a fake runtime, no checkpoint or GPU.
    captured = {}
    class Model:
        n_obs_steps = 8
        def load_state_dict(self, *args, **kwargs): pass
        def to(self, *args): return self
        def eval(self): return self
        def predict_progress(self, **kwargs):
            captured['selected'] = kwargs['frame_index']
            return torch.tensor([0.4]), torch.tensor([[0.2, 0.8]])
    class Dataset:
        n_obs_steps=8
        _get_dataset_index=staticmethod(lambda i: 0)
        episode_ranges={(0, 0): (0, 1)}
        episodes_meta=[{0: {'source_episode_index': 7}}]
        def __len__(self): return 1
    batch = dict(images=torch.zeros(1, 9, 1, 3, 2, 2), states=torch.zeros(1,9,1),
                 text_input_ids=torch.zeros(1,1),text_attention_mask=torch.ones(1,1),
                 lengths=torch.tensor([9]),episode_index=torch.tensor([0]),
                 current_index=torch.tensor([0]),task_description=['task'],
                 dense_targets=torch.arange(9).reshape(1,9))
    class Config(dict):
        __getattr__ = dict.__getitem__
        @staticmethod
        def fromfile(path):
            return Config(inference_dataset={}, train_dataloader=SimpleNamespace(dataset={}),
                          runner=SimpleNamespace(collator={}), model={})
    args = SimpleNamespace(num_workers=0,batch_size=1,max_batches=None,config='unused',cfg_options=None,
                           ckpt_path='unused',device='cpu',head_mode='dense',num_visualizations=1,
                           visualize_only=False,output_path='unused',output_dir='unused',num_display_frames=1)
    from unittest.mock import patch
    with patch.object(infer,'parse_args',return_value=args), patch.object(infer,'Config',Config), \
         patch.object(infer,'_register_sarm_runtime_modules',return_value={
             'build_collator_from_cfg':lambda _:None,'build_dataset_from_cfg':lambda _:Dataset(),
             'build_vla_from_cfg':lambda _:Model()}), \
         patch.object(infer.torch,'load',return_value={}), \
         patch.object(infer,'DataLoader',return_value=[batch]), \
         patch.object(infer,'_resolve_head_modes',return_value=['dense']), \
         patch.object(infer,'_visual_episode_indices',return_value={0}), \
         patch.object(infer,'_to_numpy_image',side_effect=lambda seq,index: captured.update(image=index)), \
         patch.object(infer,'_normalize_target_value',side_effect=lambda model,mode,value:value), \
         patch.object(infer,'_write_jsonl',side_effect=lambda path,records:captured.update(records=records)), \
         patch.object(infer,'_visualize_sarm_predictions',side_effect=lambda **kwargs:captured.update(viz=kwargs)):
        infer.main()
    assert captured['selected'] == captured['image'] == 4
    record = captured['viz']['viz_records_by_mode']['dense'][0][0]
    assert record['gt_progress'] == 4
    assert record['stage_probs'].tolist() == pytest.approx([0.2, 0.8])
    assert captured['records'][0]['frame_index'] == 0
