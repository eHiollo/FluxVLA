"""A10 camera-slot and action transforms shared by training and serving."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from fluxvla.engines import TRANSFORMS


@TRANSFORMS.register_module()
class A10CameraSlots:
    """Place base and right-wrist images in PI0.5's three image slots."""

    def __call__(self, sample: dict) -> dict:
        images = sample['images']
        if len(images) != 2:
            raise ValueError(f'A10 expects two images, got {len(images)}')
        base, right = (np.asarray(image) for image in images)
        if base.shape != right.shape or base.ndim != 3:
            raise ValueError('A10 images must have matching CHW shapes')
        sample['images'] = [base, np.zeros_like(base), right]
        sample['img_masks'] = np.array([True, False, True])
        return sample


@TRANSFORMS.register_module()
class A10DenormalizeAction:
    """Invert quantile normalization, then add current joints to deltas."""

    def __init__(self, norm_stats: str | dict,
                 statistic_name: str = 'a10') -> None:
        if isinstance(norm_stats, str):
            with Path(norm_stats).open(encoding='utf-8') as stream:
                norm_stats = json.load(stream)
        entry = norm_stats.get(statistic_name, norm_stats)
        action = entry['action']
        self.low = np.asarray(action['q01'], dtype=np.float32)
        self.high = np.asarray(action['q99'], dtype=np.float32)
        if self.low.shape != (7,) or self.high.shape != (7,):
            raise ValueError('A10 action statistics must contain seven values')

    def __call__(self, data: dict) -> np.ndarray:
        normalized = np.asarray(data['action'], dtype=np.float32)
        state = np.asarray(data['state'], dtype=np.float32).reshape(-1)
        if normalized.ndim not in (2, 3) or normalized.shape[-1] < 7:
            raise ValueError(f'Expected [..., T, >=7] actions, got {normalized.shape}')
        if state.shape != (7,) or not np.isfinite(state).all():
            raise ValueError('A10 current state must be finite 7D')
        actions = (normalized[..., :7] + 1.0) * 0.5 * (
            self.high - self.low) + self.low
        actions[..., :6] += state[:6]
        if not np.isfinite(actions).all():
            raise ValueError('A10 model produced nonfinite actions')
        return np.ascontiguousarray(actions, dtype=np.float32)
