"""Shared chunk timing for robot runners; operators own the publishing clock."""

import numpy as np

from .trajectory_utils import resample_remaining


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
