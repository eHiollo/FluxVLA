"""Build the native A10 PI0.5 config from the central workflow settings."""

from __future__ import annotations

import copy
import os
from pathlib import Path

from mmengine import Config

from .config import DEFAULT_CONFIG, DEFAULT_SITE, PROJECT_ROOT, load_project


def _project() -> dict:
    return load_project(os.environ.get('FLUXVLA_A10_CONFIG', DEFAULT_CONFIG),
                        os.environ.get('FLUXVLA_A10_SITE', DEFAULT_SITE))


def _base_config() -> dict:
    path = (PROJECT_ROOT / 'configs/pi05/'
            'pi05_paligemma_aloha_full_finetune.py')
    return Config.fromfile(str(path)).to_dict()


def build_config() -> dict:
    """Return a FluxVLA train/serve config without OpenPI checkpoint guesses."""
    cfg = _project()
    model_cfg = cfg['model']
    base = _base_config()
    model = copy.deepcopy(base['model'])
    model['n_action_steps'] = model_cfg['action_horizon']
    model['ori_action_dim'] = cfg['robot']['action_dim']
    model['loss_action_dim'] = model_cfg['padded_action_dim']
    model['max_action_dim'] = model_cfg['padded_action_dim']
    model['pretrained_name_or_path'] = model_cfg['base_checkpoint']

    tokenizer_path = model_cfg['tokenizer_path']
    if not tokenizer_path and model_cfg['base_checkpoint']:
        base_path = Path(model_cfg['base_checkpoint'])
        tokenizer_path = str(base_path.parent if base_path.suffix else base_path)
    if not tokenizer_path:
        # Config inspection is still possible before installing base weights.
        tokenizer_path = str(PROJECT_ROOT / 'checkpoints/pi05_base')
    transforms = [
        dict(type='ProcessParquetInputs',
             parquet_keys=['observation.state', 'timestamp', 'actions',
                           'info', 'stats', 'action_masks'],
             video_keys=['observation.images.base_0_rgb',
                         'observation.images.right_wrist_0_rgb'],
             name_mappings={'observation.state': ['states'],
                            'actions': ['actions']},
             video_backend='pyav'),
        dict(type='A10CameraSlots'),
        dict(type='RelativeActions', mask=[True] * 6 + [False]),
        dict(type='NormalizeStatesAndActions', state_key='proprio',
             action_key='action', norm_type='quantile',
             output_dtype='float32'),
        dict(type='PreparePromptWithState'),
        dict(type='ProcessPrompts', max_len=128,
             tokenizer=dict(type='PretrainedTokenizer',
                            model_path=tokenizer_path)),
        dict(type='PadStatesAndActions',
             model_action_dim=model_cfg['padded_action_dim']),
        dict(type='ResizeImagesWithPad', height=224, width=224,
             backend='pil'),
        dict(type='SimpleNormalizeImages'),
        dict(type='OpenPIImageAugment', base_camera_indices=(0, )),
    ]
    train_dataloader = dict(
        per_device_batch_size=model_cfg['batch_size'],
        per_device_num_workers=2,
        dataset=dict(
            type='DistributedRepeatingDataset',
            reshuffle_each_epoch=True,
            statistic_name='a10',
            auto_compute_statistics={'profile': 'a10'},
            statistic_keys=['observation.state'],
            name_mappings={'observation.state': ['proprio', 'action']},
            datasets=[dict(
                type='ParquetDatasetV3',
                data_root_path=[cfg['dataset']['root']],
                action_key='observation.state',
                statistic_name='a10',
                action_window_size=model_cfg['action_horizon'],
                window_start_idx=1,
                transforms=transforms)]))
    runner = copy.deepcopy(base['runner'])
    # Thor has one GPU; use the existing DDP runner rather than FSDP sharding.
    runner['type'] = 'DDPTrainRunner'
    for key in ('sharding_strategy', 'fsdp_wrap_policy', 'change_key_name'):
        runner.pop(key, None)
    runner['ema_decay'] = model_cfg['ema_decay']
    runner['max_steps'] = model_cfg['train_steps']
    runner['max_epochs'] = None
    runner['grad_accumulation_steps'] = model_cfg['grad_accumulation_steps']
    runner['tokenizer']['model_path'] = tokenizer_path
    runner['collator'] = dict(
        type='DictCollator',
        keys=['states', 'timestamp', 'images', 'img_masks',
              'lang_tokens', 'lang_masks', 'actions', 'action_masks'],
        meta_keys=['task_description', 'prompt', 'info', 'stats'])
    runner['metric']['active_trackers'] = ('jsonl', )
    runner['enable_gradient_checkpointing'] = True
    return dict(model=model, inference_model=copy.deepcopy(model),
                train_dataloader=train_dataloader, runner=runner)
