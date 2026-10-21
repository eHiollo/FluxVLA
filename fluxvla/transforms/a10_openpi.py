"""Inference-only adapters for the original OpenPI A10 normalization assets."""

import json
from pathlib import Path

import numpy as np

from fluxvla.engines.utils.root import TRANSFORMS


def load_a10_stats(norm_stats):
    if isinstance(norm_stats, (str, Path)):
        with Path(norm_stats).open() as stream:
            norm_stats = json.load(stream)
    source = norm_stats['norm_stats']
    result = {}
    for target, key in [('proprio', 'state'), ('action', 'actions')]:
        result[target] = {}
        for quantile in ('q01', 'q99'):
            values = np.asarray(source[key][quantile], dtype=np.float64)
            if values.shape != (7,) or not np.isfinite(values).all():
                raise ValueError(f'{key}.{quantile} must contain seven finite values')
            result[target][quantile] = values
        if np.any(result[target]['q99'] < result[target]['q01']):
            raise ValueError(f'{key} has reversed quantiles')
    return result


@TRANSFORMS.register_module()
class OpenPIA10DenormalizeAction:
    """Unnormalize once; add observation joints, never accumulate deltas."""

    def __init__(self, norm_stats):
        stats = load_a10_stats(norm_stats)['action']
        self.low, self.high = stats['q01'], stats['q99']

    def normalize_absolute(self, actions, state):
        """Encode retained absolute targets relative to the NEW observation."""
        actions = np.array(actions, dtype=np.float32, copy=True)
        actions[:, :6] -= np.asarray(state, dtype=np.float32)[:6]
        return np.ascontiguousarray(2. * (actions - self.low) /
                                    (self.high - self.low + 1e-6) - 1., dtype=np.float32)

    def __call__(self, data):
        action = np.asarray(data['action'], dtype=np.float32)
        state = np.asarray(data['state'], dtype=np.float32)
        if action.ndim == 3 and action.shape[0] == 1:
            action = action[0]
        if action.ndim != 2 or not len(action) or action.shape[1] < 7:
            raise ValueError('Expected nonempty [T,D>=7] actions')
        if state.shape != (7,) or not np.isfinite(state).all():
            raise ValueError('Expected finite original state[7]')
        action = (action[:, :7] + 1.) / 2. * (self.high - self.low + 1e-6) + self.low
        action[:, :6] += state[:6]
        if not np.isfinite(action).all():
            raise ValueError('Nonfinite A10 action')
        return np.ascontiguousarray(action, dtype=np.float32)


@TRANSFORMS.register_module()
class A10RTCDenormalizeAction:
    """Undo native RTC training mean/std and restore absolute joint targets."""

    def __init__(self, norm_stats):
        if isinstance(norm_stats, (str, Path)):
            norm_stats = json.loads(Path(norm_stats).read_text())
        stats = norm_stats['a10']['action']
        self.mean = np.asarray(stats['mean'], dtype=np.float32)
        self.std = np.asarray(stats['std'], dtype=np.float32)
        if (self.mean.shape != (7,) or self.std.shape != (7,)
                or not np.isfinite(self.mean).all() or not np.isfinite(self.std).all()
                or np.any(self.std < 0)):
            raise ValueError('Expected finite seven-dimensional action mean/std')

    def normalize_absolute(self, actions, state):
        if np.any(self.std <= 0):
            raise ValueError('RTC guidance requires positive action standard deviations')
        actions = np.array(actions, dtype=np.float32, copy=True)
        actions[:, :6] -= np.asarray(state, dtype=np.float32)[:6]
        return np.ascontiguousarray((actions - self.mean) / self.std, dtype=np.float32)

    def __call__(self, data):
        action = np.asarray(data['action'], dtype=np.float32)
        state = np.asarray(data['state'], dtype=np.float32)
        if action.ndim == 3 and action.shape[0] == 1:
            action = action[0]
        if action.ndim != 2 or not len(action) or action.shape[1] < 7:
            raise ValueError('Expected nonempty [T,D>=7] actions')
        if state.shape != (7,) or not np.isfinite(state).all():
            raise ValueError('Expected finite original state[7]')
        action = action[:, :7] * self.std + self.mean
        action[:, :6] += state[:6]
        if not np.isfinite(action).all():
            raise ValueError('Nonfinite A10 action')
        return np.ascontiguousarray(action, dtype=np.float32)
