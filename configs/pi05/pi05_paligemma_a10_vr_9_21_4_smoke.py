"""Short A10 pi0.5 fine-tuning run aligned with the local OpenPI recipe.

Run with ``bash scripts/train_a10_smoke.sh``. The dataset and OpenPI
normalization assets are read in place; all training artifacts go to /data.
"""

import json
import os

_base_ = ['./pi05_paligemma_ur3_full_finetune.py']

_DATA_ROOT = os.environ.get(
    'A10_DATA_ROOT', '/home/allen/Allen/openpi/dataset/a10_vr_9_21_4')
_OPENPI_STATS = os.environ.get(
    'A10_OPENPI_STATS',
    '/data/allen/openpi/checkpoints/pi05_a10_finetune/'
    'a10_vr_9_21_4_h30/99999/assets/dataset/a10_vr_9_21_4/norm_stats.json')
_BASE_MODEL = os.environ.get(
    'FLUXVLA_PI05_BASE', '/data/allen/fluxvla/pretrained/pi05_base')
_TASK = (
    'Grasp the bottle, place it into the box beside it, close the lid, '
    'and return the robot arm to its initial position.')

_norm = json.loads(
    __import__('pathlib').Path(_OPENPI_STATS).read_text(
        encoding='utf-8'))['norm_stats']

model = dict(
    n_action_steps=30,
    pretrained_name_or_path=os.path.join(_BASE_MODEL, 'model.safetensors'),
    name_mapping={
        'llm_expert.embed_tokens':
        'paligemma_with_expert.gemma_expert.lm_head'},
    strict_mapping=True,
    freeze_llm_backbone=True,
    freeze_vision_backbone=True,
    freeze_projector=True,
    use_lora=True,
    lora_rank=16,
    lora_alpha=16,
    lora_expert_rank=32,
    lora_dropout=0.0,
    lora_target_modules=(
        r'.*(llm_backbone|llm_expert).*'
        r'(q_proj|k_proj|v_proj|o_proj|gate_proj|up_proj|down_proj)'),
    # OpenPI trains only the PaliGemma and expert LoRA parameters.
    modules_to_save=None,
)

train_dataloader = dict(
    per_device_batch_size=1,
    per_device_num_workers=1,
    dataset=dict(
        _delete_=True,
        type='DistributedRepeatingDataset',
        dataset_statistics={
            'a10': {
                'proprio': _norm['state'],
                'action': _norm['actions'],
            }
        },
        statistic_name='a10',
        statistic_keys=['observation.state'],
        datasets=dict(
            type='ParquetDataset',
            data_root_path=_DATA_ROOT,
            action_key='observation.state',
            action_window_size=30,
            window_start_idx=0,
            supervise_terminal_padding=True,
            statistic_name='a10',
            transforms=[
                dict(
                    type='ProcessParquetInputs',
                    parquet_keys=[
                        'observation.state', 'timestamp', 'actions',
                        'info', 'stats', 'action_masks'],
                    video_keys=[
                        'observation.images.base_0_rgb',
                        'observation.images.right_wrist_0_rgb'],
                    name_mappings={
                        'observation.state': ['states'],
                        'actions': ['actions']}),
                dict(type='PrepareA10OpenPIInputs', task_description=_TASK),
                dict(type='RelativeActions', mask=[True] * 6 + [False]),
                dict(
                    type='NormalizeStatesAndActions',
                    action_dim=None,
                    state_dim=None,
                    state_key='proprio',
                    action_key='action',
                    norm_type='mean_std',
                    output_dtype='float32'),
                dict(type='PreparePromptWithState'),
                dict(
                    type='ProcessPrompts',
                    max_len=128,
                    tokenizer=dict(
                        type='PretrainedTokenizer', model_path=_BASE_MODEL)),
                dict(type='PadStatesAndActions', model_action_dim=32),
                dict(
                    type='ResizeImagesWithPad',
                    height=224, width=224, backend='pil'),
                dict(type='SimpleNormalizeImages'),
            ])))

runner = dict(
    _delete_=True,
    type='DDPTrainRunner',
    max_epochs=None,
    max_steps=2,
    save_iter_interval=2,
    max_keep_ckpts=1,
    grad_accumulation_steps=8,
    seed=42,
    optimizer=dict(
        type='AdamW', lr=2.5e-5, betas=(0.9, 0.95),
        eps=1e-8, weight_decay=1e-10),
    max_grad_norm=1.0,
    collator=dict(
        type='DictCollator',
        keys=[
            'states', 'timestamp', 'images', 'img_masks',
            'lang_tokens', 'lang_masks', 'actions', 'action_masks'],
        meta_keys=['task_description', 'prompt', 'info', 'stats']),
    sampler=None,
    lr_scheduler=dict(
        type='linear-warmup+cosine-decay',
        schedule_style='openpi',
        warmup_steps=1000,
        decay_steps=30000,
        min_lr=2.5e-6),
    tokenizer=dict(type='PretrainedTokenizer', model_path=_BASE_MODEL),
    metric=dict(
        type='VLAMetric', active_trackers=('jsonl',),
        run_dir='work_dirs', window_size=1),
    enable_gradient_checkpointing=True,
    static_graph=False,
    enable_mixed_precision_training=True,
    mixed_precision_dtype='bf16',
    keep_params_fp32=False,
)
