"""A10 bottle/box SARM: held-out episodes, two cameras, 30 fps."""

pretrained_name_or_path = './checkpoints/clip-vit-base-patch32'
data_root_path = './datasets/a10_sarm_train'
video_keys = ['observation.images.base_0_rgb', 'observation.images.right_wrist_0_rgb']

current_transforms = [
    dict(type='DecodeLeRobotVideoSequence', video_keys=video_keys, backend='pyav'),
    dict(type='ResizeImages', height=224, width=224, preserve_leading_dims=True),
    dict(type='NormalizeImages', means=[0.48145466, 0.4578275, 0.40821073],
         stds=[0.26862954, 0.26130258, 0.27577711],
         preserve_leading_dims=True, scale_to_unit_interval=True),
    dict(type='NormalizeStatesAndActions', state_key='observation.state',
         action_key=None, state_dim=32, norm_type='mean_std', output_dtype='float32'),
    dict(type='TokenizeText', tokenizer=dict(type='PretrainedTokenizer',
         model_path=pretrained_name_or_path), max_length=77),
]

current_collator = dict(type='DictCollator', keys=[
    'images', 'states', 'lengths', 'episode_index', 'current_index',
    'sparse_targets', 'dense_targets', 'text_input_ids', 'text_attention_mask'],
    meta_keys=['task_description'])

model = dict(
    type='SARMRewardModel', annotation_mode='dense_only',
    data_root_path=data_root_path, n_obs_steps=8, frame_gap=30, max_rewind_steps=4,
    llm_backbone=dict(type='SARMBackbone', pretrained_name_or_path=pretrained_name_or_path,
                      hidden_dim=768, num_heads=12, num_layers=8, max_state_dim=32,
                      dropout=0.1, num_cameras=2, freeze_clip_backbone=True),
    freeze_llm_backbone=False)

train_dataloader = dict(
    per_device_batch_size=16, per_device_num_workers=4,
    dataset=dict(type='SARMDataset', data_root_path=data_root_path, video_keys=video_keys,
                 annotation_mode='dense_only', n_obs_steps=8, frame_gap=30,
                 max_rewind_steps=4, rewind_probability=0.8,
                 state_key='observation.state', training=True, transforms=current_transforms))

runner = dict(
    type='DDPTrainRunner', max_steps=5000, max_epochs=None, seed=42,
    optimizer=dict(type='AdamW', lr=5e-5, weight_decay=1e-3), max_grad_norm=1.0,
    collator=current_collator, sampler='distributed', save_iter_interval=100,
    max_keep_ckpts=5,
    metric=dict(type='VLAMetric', active_trackers=('jsonl',), run_dir='./sarm_runs/a10',
                window_size=20),
    lr_scheduler=dict(type='linear-warmup+cosine-decay', warmup_ratio=0.03),
    enable_gradient_checkpointing=False, enable_mixed_precision_training=True,
    mixed_precision_dtype='bf16')

inference_dataset = dict(
    type='SARMDataset', data_root_path='./datasets/a10_sarm_val', video_keys=video_keys,
    annotation_mode='dense_only', n_obs_steps=8, frame_gap=30, max_rewind_steps=0,
    rewind_probability=0.0, state_key='observation.state', training=False,
    transforms=current_transforms)
