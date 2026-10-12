"""Inference only: original A10 OpenPI export, no training/site YAML needed.

Local (after operator integration):
  python scripts/inference.py --config configs/a10/a10_openpi_inference.py
Model server only (no robot connection):
  python -m fluxvla.engines.runners.serving.serve \
    --config configs/a10/a10_openpi_inference.py --from-config --dtype fp32
Then use a10_remote_inference.py on the robot side.
Set A10_OPENPI_CHECKPOINT / A10_OPENPI_TOKENIZER to override local paths.
"""

_base_ = []
import os
from pathlib import Path
from mmengine import Config

_root = Path('{{fileDirname}}').resolve().parents[1]
_checkpoint = os.environ.get('A10_OPENPI_CHECKPOINT', str(
    _root / 'checkpoints/a10_openpi_90000'))
_tokenizer = os.environ.get('A10_OPENPI_TOKENIZER', str(
    _root / 'checkpoints/tokenizers/paligemma_tokenizer.model'))
_stats = str(Path(_checkpoint) / 'assets/dataset/a10_vr_9_21_4/norm_stats.json')
_architecture = Config.fromfile(str(
    _root / 'configs/pi05/pi05_paligemma_aloha_full_finetune.py')).model.to_dict()
_architecture.update(pretrained_name_or_path=None, n_action_steps=30,
                     ori_action_dim=7, num_steps=10,
                     enable_mixed_precision_training=False)
inference_model = dict(type='OpenPIA10Policy', checkpoint_dir=_checkpoint,
                       model_config=_architecture)
inference = dict(
    type='A10InferenceRunner', model_from_config=True,
    dataset=dict(type='OpenPIA10InferenceDataset', norm_stats=_stats,
                 tokenizer_path=_tokenizer),
    denormalize_action=dict(type='OpenPIA10DenormalizeAction', norm_stats=_stats),
    operator=dict(type='A10Operator', host='127.0.0.1',
                  command_mode='joint', allow_motion=False),
    task_suite_name='a10', instruction='Reach the yellow lemon',
    action_chunk=30, execute_horizon=10, action_dt=1.0 / 30.0,
    max_publish_step=30, gripper_mm_range=(0., 85.), dry_run=True, max_joint_step_rad=None,
    mixed_precision_dtype='float32', enable_mixed_precision=False,
    keep_params_fp32=True,
)
