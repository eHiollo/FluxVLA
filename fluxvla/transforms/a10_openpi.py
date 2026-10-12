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
