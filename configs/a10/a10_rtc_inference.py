"""Native A10 RTC-trained 50k checkpoint; plain PyTorch by default.

Server: python -m fluxvla.engines.runners.serving.serve --config
configs/a10/a10_rtc_inference.py --from-config --dtype fp32 --acceleration off
This switches trained weights, not the synchronous runner to async RTC.
"""

_base_ = []
import json
import os
from pathlib import Path
from mmengine import Config

_root = Path('{{fileDirname}}').resolve().parents[1]
_checkpoint = os.environ.get('A10_RTC_CHECKPOINT', str(
    _root / 'checkpoints/FluxVLA-A10-RTC-50k'))
_export = json.loads((Path(_checkpoint) / 'config.json').read_text())
_architecture = _export['model']
_architecture.update(pretrained_name_or_path=None, use_lora=False,
                     enable_mixed_precision_training=False, num_steps=10)
inference_model = dict(type='NativeA10Policy', checkpoint_dir=_checkpoint,
                       model_config=_architecture)
_accelerated_architecture = Config(_architecture).to_dict()
_accelerated_architecture.update(type='A10PI05FlowMatchingInference',
                                 num_views=2, triton_max_prompt_len=128)
_accelerated_architecture['llm_backbone']['type'] = 'ConditionGemmaInferenceModel'
_accelerated_architecture['llm_expert']['type'] = 'ConditionGemmaInferenceModel'
_accelerated_architecture['vision_backbone']['type'] = 'SigLIPViTBackboneInference'
for _key in ('projector', 'action_in_proj', 'action_out_proj', 'time_mlp_in', 'time_mlp_out'):
    _accelerated_architecture[_key]['type'] = 'LinearProjectorInference'
inference_model_accelerated = dict(type='NativeA10Policy', checkpoint_dir=_checkpoint,
                                   model_config=_accelerated_architecture)
_stats = str(Path(_checkpoint) / 'dataset_statistics.json')
inference = Config.fromfile(str(_root / 'configs/a10/a10_remote_inference.py')).inference.to_dict()
inference.pop('remote_inference')
inference.update(model_from_config=True, task_suite_name='a10',
                 instruction=_export['_TASK'],
                 dataset=dict(type='A10RTCInferenceDataset', norm_stats=_stats,
                              tokenizer_path=str(Path(_checkpoint) / 'tokenizer')),
                 denormalize_action=dict(type='A10RTCDenormalizeAction', norm_stats=_stats),
                 mixed_precision_dtype='float32', enable_mixed_precision=False,
                 keep_params_fp32=True)
