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


@TRANSFORMS.register_module(name="A10NativeDenormalizeAction")
@TRANSFORMS.register_module()
class A10RTCDenormalizeAction:
    """Configurable inverse normalization and joint deltas; preserve old RTC defaults."""

    def __init__(self, norm_stats, state_dim=7, statistic_name="a10", norm_type="mean_std", relative_mask=None):
        if isinstance(norm_stats, (str, Path)):
            norm_stats = json.loads(Path(norm_stats).read_text())
        if state_dim not in (7, 14) or norm_type not in ("mean_std", "quantile"):
            raise ValueError("A10 requires state_dim 7/14 and mean_std/quantile normalization")
        self.state_dim, self.norm_type = state_dim, norm_type
        self.relative_mask = np.asarray(
            relative_mask if relative_mask is not None else ([True] * 6 + [False]) * (state_dim // 7), dtype=bool
        )
        if self.relative_mask.shape != (state_dim,):
            raise ValueError("Invalid relative mask")
        stats = norm_stats[statistic_name]["action"]
        names = ("q01", "q99") if norm_type == "quantile" else ("mean", "std")
        first, second = (np.asarray(stats[k], dtype=np.float32) for k in names)
        if (
            first.shape != (state_dim,)
            or second.shape != (state_dim,)
            or not np.isfinite(first).all()
            or not np.isfinite(second).all()
        ):
            raise ValueError("Invalid action statistics")
        if norm_type == "quantile":
            if np.any(second < first):
                raise ValueError("Reversed quantiles")
            self.low, self.high = first, second
        else:
            if np.any(second < 0):
                raise ValueError("Negative standard deviation")
            self.mean, self.std = first, second

    def normalize_absolute(self, actions, state):
        actions = np.array(actions, dtype=np.float32, copy=True)
        actions[:, self.relative_mask] -= np.asarray(state, dtype=np.float32)[self.relative_mask]
        if self.norm_type == "quantile":
            return np.ascontiguousarray(2 * (actions - self.low) / (self.high - self.low + 1e-6) - 1, dtype=np.float32)
        if np.any(self.std <= 0):
            raise ValueError("RTC guidance requires positive action standard deviations")
        return np.ascontiguousarray((actions - self.mean) / self.std, dtype=np.float32)

    def __call__(self, data):
        action = np.asarray(data["action"], dtype=np.float32)
        state = np.asarray(data["state"], dtype=np.float32)
        if action.ndim == 3 and action.shape[0] == 1:
            action = action[0]
        if action.ndim != 2 or not len(action) or action.shape[1] < self.state_dim:
            raise ValueError(f"Expected nonempty [T,D>={self.state_dim}] actions")
        if state.shape != (self.state_dim,) or not np.isfinite(state).all():
            raise ValueError(f"Expected finite original state[{self.state_dim}]")
        action = action[:, : self.state_dim]
        if self.norm_type == "quantile":
            action = (action + 1) / 2 * (self.high - self.low + 1e-6) + self.low
        else:
            action = action * self.std + self.mean
        action[:, self.relative_mask] += state[self.relative_mask]
        if not np.isfinite(action).all():
            raise ValueError("Nonfinite A10 action")
        return np.ascontiguousarray(action, dtype=np.float32)
