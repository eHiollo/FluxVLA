"""Shared chunk timing for robot runners; operators own the publishing clock."""

import math

import numpy as np

from .trajectory_utils import resample_remaining


class StreamReferenceClock:
    """One nominal reference grid across replacements; never follow send jitter."""

    def __init__(self, dt):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('StreamReferenceClock requires positive finite dt')
        self.dt = float(dt)
        self.reset()

    def reset(self):
        self.origin_s = self.last_s = None

    def next_slot(self, now):
        if not math.isfinite(now):
            raise ValueError('StreamReferenceClock requires finite now')
        if self.origin_s is None:
            self.origin_s = float(now)
        minimum = self.origin_s if self.last_s is None else self.last_s+self.dt
        tick = max(0, math.ceil((max(now, minimum)-self.origin_s)/self.dt-1e-8))
        return self.origin_s+tick*self.dt

    def record(self, reference_s):
        tick = (reference_s-self.origin_s)/self.dt
        if (not math.isfinite(reference_s) or abs(tick-round(tick)) > 1e-6
                or (self.last_s is not None and reference_s-self.last_s < self.dt-1e-8)):
            raise ValueError('Stream reference must advance on its original grid')
        self.last_s = float(reference_s)


def select_action_chunk(actions, context, previous, *, now, dt,
                        async_execution, execute_horizon):
    """Use the observation/inference time axis when replacing a moving chunk.

    Synchronous execution publishes the selected horizon. Asynchronous execution
    publishes the full remaining prediction in the background; the horizon says
    when the runner observes again, rather than when the operator must stop.
    """
    if async_execution and previous is not None:
        context.action_timestamp = context.inference_start
        offset = max(0., (now - context.action_timestamp) / dt)
        selected = resample_remaining(actions, offset)
    else:
        context.action_timestamp = now
        offset = 0.
        selected = actions
        if not async_execution and execute_horizon is not None:
            selected = selected[:execute_horizon]
    context.execution_offset = offset
    context.actions = np.array(actions, copy=True)
    return selected


def remaining_action_prefix(actions, *, action_timestamp, now, dt):
    """Align the old prediction with the next observation, including fractions."""
    return resample_remaining(actions, max(0., (now - action_timestamp) / dt))
