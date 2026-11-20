"""Follower block arrival checks using fresh actual joint feedback."""

import numpy as np

JOINTS = [0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12]
DEFAULTS = dict(enabled=False, arrival_timeout_s=30., poll_hz=30.,
                position_tolerance_deg=.2, speed_tolerance_deg_s=.5,
                stable_duration_s=.3, max_feedback_gap_s=.1)


def tail_config(value=None):
    value = dict(value or {})
    unknown = value.keys() - DEFAULTS.keys()
    if unknown:
        raise ValueError(f'Unknown follower_tail options: {sorted(unknown)}')
    cfg = dict(DEFAULTS, **value)
    for key in ('enabled',):
        if type(cfg[key]) is not bool:
            raise ValueError(f'follower_tail.{key} must be bool')
    for key, val in cfg.items():
        if key in ('enabled',):
            continue
        if key == 'speed_tolerance_deg_s' and val is None:
            continue
        if (isinstance(val, bool) or not isinstance(val, (int, float)) or not np.isfinite(val)
                or val < 0 or (val == 0 and key != 'stable_duration_s')):
            raise ValueError(f'follower_tail.{key} must be positive and finite (stable duration may be zero)')
    if cfg['max_feedback_gap_s'] < 1 / cfg['poll_hz']:
        raise ValueError('follower_tail feedback gap must allow the polling period')
    return cfg




class ArrivalMonitor:
    """Require fresh, strictly advancing actual-position samples during an arrival gate."""

    def __init__(self, target, started_s, config):
        self.target = np.asarray(target, dtype=float)
        if self.target.shape != (12,) or not np.isfinite(self.target).all():
            raise ValueError('Arrival target requires twelve finite joints')
        self.started_s, self.cfg = started_s, config
        self.previous = None
        self.stable_since = None

    def observe(self, actual, stamp, now):
        actual = np.asarray(actual, dtype=float)
        if (actual.shape != (12,) or not np.isfinite(actual).all()
                or not np.isfinite(stamp) or not 0 <= now - stamp <= self.cfg['max_feedback_gap_s']):
            raise RuntimeError('Invalid/stale arrival feedback')
        if stamp < self.started_s:
            raise RuntimeError('Arrival feedback predates arrival gate')
        if self.previous is not None and stamp <= self.previous[0]:
            if stamp < self.previous[0]:
                raise RuntimeError('Arrival feedback timestamp moved backwards')
            return False, None, None
        error = float(np.rad2deg(np.abs(actual - self.target)).max())
        speed = None
        if self.previous is not None:
            gap = stamp - self.previous[0]
            if gap <= self.cfg['max_feedback_gap_s']:
                speed = float(np.rad2deg(np.abs(actual - self.previous[1])).max() / gap)
        self.previous = stamp, actual.copy()
        speed_limit = self.cfg['speed_tolerance_deg_s']
        good = (error <= self.cfg['position_tolerance_deg']
                and (speed_limit is None or (speed is not None and speed <= speed_limit)))
        if self.cfg['stable_duration_s'] == 0:
            return good, error, speed
        if good:
            if self.stable_since is None:
                self.stable_since = stamp
        else:
            self.stable_since = None
        return (good and stamp - self.stable_since >= self.cfg['stable_duration_s'], error, speed)
