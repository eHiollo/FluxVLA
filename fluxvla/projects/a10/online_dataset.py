"""Prepare A10 live observations with the same transforms as training."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import torch

from fluxvla.engines import DATASETS, build_transform_from_cfg


@DATASETS.register_module()
class A10InferenceDataset:
    """Convert raw RGB cameras and 7D state into a PI0.5 batch."""

    def __init__(self, norm_stats: str | dict, tokenizer_path: str,
                 statistic_name: str = 'a10', model_path: str | None = None):
        del model_path
        if isinstance(norm_stats, str):
            with Path(norm_stats).open(encoding='utf-8') as stream:
                norm_stats = json.load(stream)
        self.stats = norm_stats[statistic_name]
        self.last_raw_state = None
        self.transforms = [build_transform_from_cfg(item) for item in [
            dict(type='A10CameraSlots'),
            dict(type='NormalizeStatesAndActions', state_key='proprio',
                 action_key=None, norm_type='quantile',
                 output_dtype='float32'),
            dict(type='PreparePromptWithState'),
            dict(type='ProcessPrompts', max_len=128,
                 tokenizer=dict(type='PretrainedTokenizer',
                                model_path=tokenizer_path)),
            dict(type='PadStatesAndActions', model_action_dim=32),
            dict(type='ResizeImagesWithPad', height=224, width=224,
                 backend='pil'),
            dict(type='SimpleNormalizeImages'),
        ]]

    def __call__(self, observation: dict) -> dict:
        state = np.asarray(observation['qpos'], dtype=np.float32).reshape(-1)
        if state.shape != (7,) or not np.isfinite(state).all():
            raise ValueError('A10 observation qpos must be finite 7D')
        self.last_raw_state = state.copy()
        images = []
        for key in ('base_0_rgb', 'right_wrist_0_rgb'):
            image = np.asarray(observation[key])
            if image.ndim != 3 or image.shape[-1] != 3 or image.dtype != np.uint8:
                raise ValueError(f'{key} must be HWC uint8 RGB')
            images.append(np.ascontiguousarray(image.transpose(2, 0, 1)))
        sample = dict(states=state, images=images,
                      task_description=str(observation['task_description']),
                      stats=self.stats)
        for transform in self.transforms:
            sample = transform(sample)
        return dict(
            images=torch.as_tensor(np.asarray(sample['images']))[None],
            img_masks=torch.as_tensor(sample['img_masks'])[None],
            lang_tokens=torch.as_tensor(sample['lang_tokens'])[None],
            lang_masks=torch.as_tensor(sample['lang_masks'])[None],
            states=torch.as_tensor(sample['states'], dtype=torch.float32)[None])
