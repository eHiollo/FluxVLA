"""Receive-time measurements; never schedule or modify robot targets."""

import threading

import numpy as np


JOINTS = [0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12]


class TailExecutionTrace:
    def __init__(self, log):
        self.log = log
        self.lock = threading.Lock()
        self.tail = None
        self.previous = None

    def arm(self, cycle, target, sent_s):
        with self.lock:
            self.tail = dict(cycle=cycle, target=np.asarray(target)[JOINTS].copy(),
                             last_send_finished_s=sent_s, samples=0,
                             first_in_tolerance_s=None, stable_since=None,
                             settled_s=None, last_error_deg=None)

    def observe(self, actual, stamp, *, query_started_s=None, controller_target=None):
        actual = np.asarray(actual, dtype=float)
        if actual.shape != (12,) or not np.isfinite(actual).all() or not np.isfinite(stamp):
            raise ValueError('Trace requires twelve finite actual joints and a receive timestamp')
        with self.lock:
            speed = None
            gap = None
            if self.previous is not None and stamp > self.previous[0]:
                gap = stamp - self.previous[0]
                speed = float(np.rad2deg(np.abs(actual - self.previous[1])).max()
                              / gap)
            duplicate = self.previous is not None and stamp <= self.previous[0]
            if self.previous is None or stamp > self.previous[0]:
                self.previous = (stamp, actual.copy())
            self.log('feedback', query_started_s=query_started_s,
                     received_monotonic_s=stamp, actual_joints_rad=actual,
                     controller_target_joints_rad=controller_target,
                     sampled_max_speed_deg_s=speed)
            tail = self.tail
            if tail is None or stamp < tail['last_send_finished_s'] or duplicate:
                return
            tail['samples'] += 1
            error = float(np.rad2deg(np.abs(actual - tail['target'])).max())
            tail['last_error_deg'] = error
            elapsed = stamp - tail['last_send_finished_s']
            if gap is None or gap > .1:
                tail['stable_since'] = None
            if error <= .2 and tail['first_in_tolerance_s'] is None:
                tail['first_in_tolerance_s'] = elapsed
            if error <= .2 and speed is not None and speed <= .5:
                if tail['stable_since'] is None:
                    tail['stable_since'] = stamp
                if stamp - tail['stable_since'] >= .3 and tail['settled_s'] is None:
                    tail['settled_s'] = elapsed
            else:
                tail['stable_since'] = None

    def finish(self, reason, stamp):
        with self.lock:
            tail, self.tail = self.tail, None
            if tail is None:
                return
            self.log('tail_execution', cycle=tail['cycle'], reason=reason,
                     last_send_finished_s=tail['last_send_finished_s'],
                     observation_window_s=max(0., stamp - tail['last_send_finished_s']),
                     samples=tail['samples'], first_in_tolerance_s=tail['first_in_tolerance_s'],
                     settled_s=tail['settled_s'], last_error_deg=tail['last_error_deg'],
                     censored=tail['settled_s'] is None,
                     tolerance_deg=.2, speed_tolerance_deg_s=.5, stable_duration_s=.3,
                     clock='host_receive_monotonic', physical_arrival_confirmed=tail['settled_s'] is not None)
