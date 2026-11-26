"""Bounded exponential fusion of predictions for the same future times.

This fuses overlapping forecasts, not neighboring samples along a trajectory.
No model-specific state or inference interface is required.
"""

import math

import numpy as np


def fusion_config(value=None):
    config = dict(enabled=False, method='position', alpha=.25, tail_steps=8, handover_steps=0,
                  position_gain=6., correction_tau_s=.15, correction_fraction=1.,
                  correction_floor_deg_s=3., correction_max_deg_s=30., endpoint_steps=12,
                  gripper_alpha=None, continuous_clock=False, persistent_sender=False)
    value = dict(value or {})
    if value.keys() - config.keys():
        raise ValueError(f'Unknown action_fusion options: {sorted(value.keys() - config.keys())}')
    config.update(value)
    if type(config['enabled']) is not bool:
        raise ValueError('action_fusion.enabled must be boolean')
    if type(config['continuous_clock']) is not bool:
        raise ValueError('action_fusion.continuous_clock must be boolean')
    if type(config['persistent_sender']) is not bool:
        raise ValueError('action_fusion.persistent_sender must be boolean')
    if config['method'] not in ('position', 'velocity'):
        raise ValueError('action_fusion.method must be position or velocity')
    alpha = config['alpha']
    if isinstance(alpha, bool) or not isinstance(alpha, (int, float)) or not math.isfinite(alpha) or not 0 < alpha <= 1:
        raise ValueError('action_fusion.alpha must be within (0, 1]')
    if type(config['tail_steps']) is not int or config['tail_steps'] < 2:
        raise ValueError('action_fusion.tail_steps must be an integer >= 2')
    if (type(config['handover_steps']) is not int or
            (config['handover_steps'] != 0 and not 2 <= config['handover_steps'] <= 30)):
        raise ValueError('action_fusion.handover_steps must be 0 or an integer within 2..30')
    for name in ('position_gain', 'correction_tau_s', 'correction_fraction',
                 'correction_floor_deg_s', 'correction_max_deg_s'):
        value = config[name]
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or value <= 0):
            raise ValueError(f'action_fusion.{name} must be positive and finite')
    if config['correction_floor_deg_s'] > config['correction_max_deg_s']:
        raise ValueError('action_fusion correction floor cannot exceed its maximum')
    if type(config['endpoint_steps']) is not int or config['endpoint_steps'] < 3:
        raise ValueError('action_fusion.endpoint_steps must be an integer >= 3')
    grip_alpha = config['gripper_alpha']
    if grip_alpha is not None and (isinstance(grip_alpha, bool)
            or not isinstance(grip_alpha, (int, float)) or not math.isfinite(grip_alpha)
            or not 0 < grip_alpha <= 1):
        raise ValueError('action_fusion.gripper_alpha must be None or within (0, 1]')
    return config


class ExponentialActionFusion:
    """Keep one fused future plan, replacing it at every prediction arrival."""

    def __init__(self, config=None, joint_columns=(0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12)):
        self.config = fusion_config(config)
        self.joint_columns = np.asarray(joint_columns, dtype=int)
        self.reset()

    def reset(self):
        self.plan = None
        self.start_s = self.dt = None
        self.motion_velocity = self.correction_velocity = None
        self.endpoint_start_s = None

    @staticmethod
    def _sample(plan, coordinates):
        x = np.clip(coordinates, 0., len(plan)-1)
        lo = np.floor(x).astype(int)
        hi = np.minimum(lo+1, len(plan)-1)
        fraction = np.asarray(x-lo)[..., None]
        return plan[lo]*(1-fraction) + plan[hi]*fraction

    def _old_weights(self, count, alpha):
        weight = np.full(count, 1-alpha)
        if self.config['handover_steps']:
            u = np.minimum(np.arange(count)/(self.config['handover_steps']-1), 1.)
            gain = u**3*(10-15*u+6*u*u)
            weight = 1-alpha*gain
        taper = min(count, self.config['tail_steps'])
        u = np.linspace(0., 1., taper) if taper > 1 else np.ones(1)
        weight[-taper:] *= 1-u*u*(3-2*u)
        return weight

    def _velocity_plan(self, targets, old, coordinates, old_weight, start_s, dt):
        """Blend motion, then track absolute position with a persistent state.

        A time-varying POSITION blend adds g'*(new-old) to velocity. Blending
        velocities first avoids that term. The bounded correction state carries
        across updates; its slew does not restart at every incoming chunk.
        Endpoint recovery stays in the future tail and is never fed back into
        the cached motion estimate.
        """
        columns, config = self.joint_columns, self.config
        joint = targets[:, columns]
        motion = np.gradient(joint, dt, axis=0) if len(joint) > 1 else np.zeros_like(joint)
        correction = np.zeros_like(motion)
        fused = targets.copy()
        info = dict(endpoint_start_s=None, endpoint_recovery_rad=0., short_horizon=False,
                    late_endpoint_join=False)
        if old is not None and len(targets) >= 3:
            count = len(old)
            previous_motion = self._sample(self.motion_velocity, coordinates)
            motion[:count] = (old_weight[:, None]*previous_motion
                              + (1-old_weight[:, None])*motion[:count])
            fused[0, columns] = old[0, columns]
            correction[0] = self._sample(self.correction_velocity, coordinates[0])
            if self.endpoint_start_s is not None and start_s >= self.endpoint_start_s:
                info['late_endpoint_join'] = True
                # A late update can arrive during endpoint recovery. Join the
                # applied velocity, including recovery, before leaving it.
                applied = np.gradient(self.plan[:, columns], self.dt, axis=0)
                delta = self._sample(applied, coordinates[0])-motion[0]-correction[0]
                length = min(len(targets), max(3, config['handover_steps']))
                u = np.linspace(0., 1., length)
                motion[:length] += (1-u**3*(10-15*u+6*u*u))[:, None]*delta
            decay = math.exp(-dt/config['correction_tau_s'])
            floor = math.radians(config['correction_floor_deg_s'])
            maximum = math.radians(config['correction_max_deg_s'])
            for i in range(1, len(targets)):
                predicted = fused[i-1, columns] + .5*dt*(motion[i-1]+motion[i])
                cap = np.minimum(maximum, np.maximum(
                    floor, config['correction_fraction']*np.maximum(abs(motion[i-1]), abs(motion[i]))))
                desired = np.clip(config['position_gain']*(joint[i]-predicted), -cap, cap)
                correction[i] = decay*correction[i-1] + (1-decay)*desired
                fused[i, columns] = predicted + .5*dt*(correction[i-1]+correction[i])
            length = min(config['endpoint_steps'], len(targets))
            u = np.linspace(0., 1., length)
            residual = joint[-1]-fused[-1, columns]
            fused[-length:, columns] += (u**3*(10-15*u+6*u*u))[:, None]*residual
            fused[-1, columns] = joint[-1]
            info.update(endpoint_start_s=float(start_s+(len(targets)-length)*dt),
                        endpoint_recovery_rad=float(abs(residual).max()))
        elif old is not None:
            # A one/two-point remainder has no room for a smooth splice. Keep
            # its endpoint exact; the existing raw/fused step checks still run.
            info['short_horizon'] = True
        self.motion_velocity, self.correction_velocity = motion.copy(), correction.copy()
        self.endpoint_start_s = info['endpoint_start_s']
        info['max_position_correction_velocity_rad_s'] = float(abs(correction).max())
        return fused, info

    def fuse(self, targets, *, start_s, dt):
        targets = np.array(targets, dtype=float, copy=True)
        if (targets.ndim != 2 or not len(targets) or not np.isfinite(targets).all()
                or targets.shape[1] <= int(self.joint_columns.max())):
            raise ValueError('action_fusion requires a finite nonempty joint/gripper trajectory')
        if not math.isfinite(start_s) or not math.isfinite(dt) or dt <= 0:
            raise ValueError('action_fusion requires finite start_s and positive dt')
        if self.start_s is not None and start_s < self.start_s:
            raise ValueError('action_fusion cannot reuse a plan from a future task')
        info = dict(enabled=self.config['enabled'], applied=False, overlap_steps=0,
                    method=self.config['method'],
                    alpha=self.config['alpha'], tail_steps=self.config['tail_steps'],
                    handover_steps=self.config['handover_steps'],
                    start_s=float(start_s), history_expired=False,
                    gripper_alpha=self.config['gripper_alpha'], gripper_fusion_applied=False,
                    continuous_clock=self.config['continuous_clock'],
                    persistent_sender=self.config['persistent_sender'])
        fused = targets.copy()
        old, x, old_weight = None, None, None
        if self.config['enabled'] and self.plan is not None:
            offset = (start_s - self.start_s) / self.dt
            coordinates = offset + np.arange(len(targets)) * dt / self.dt
            valid = (coordinates >= 0.) & (coordinates <= len(self.plan) - 1 + 1e-9)
            count = int(valid.sum())
            if count:
                rows = np.flatnonzero(valid)
                x = np.clip(coordinates[rows], 0., len(self.plan)-1)
                old = self._sample(self.plan, x)
                # Join the old moving reference and release it before support ends.
                old_weight = self._old_weights(count, self.config['alpha'])
                columns = self.joint_columns
                fused[np.ix_(rows, columns)] = (
                    old[:, columns] * old_weight[:, None]
                    + targets[np.ix_(rows, columns)] * (1-old_weight[:, None]))
                info.update(applied=True, overlap_steps=count)
            else:
                info['history_expired'] = True
        if self.config['enabled'] and self.config['method'] == 'velocity':
            fused, velocity_info = self._velocity_plan(targets, old, x, old_weight, start_s, dt)
            info.update(velocity_info)
        if old is not None and self.config['gripper_alpha'] is not None:
            # Aperture is an absolute position. Do not integrate arm-style
            # velocities or jump to each conflicting forecast's first value.
            columns = [6, 13]
            if targets.shape[1] <= max(columns) or any(c in self.joint_columns for c in columns):
                raise ValueError('gripper fusion requires dual A10 gripper columns 6 and 13')
            weight = self._old_weights(len(old), self.config['gripper_alpha'])[:, None]
            fused[np.ix_(rows, columns)] = (old[:, columns]*weight
                                           + targets[np.ix_(rows, columns)]*(1-weight))
            info['gripper_fusion_applied'] = True
            info['max_gripper_correction'] = float(abs(fused[:, columns]-targets[:, columns]).max())
        error = fused[:, self.joint_columns] - targets[:, self.joint_columns]
        info.update(max_correction_rad=float(np.abs(error).max()),
                    rms_correction_rad=float(np.sqrt(np.mean(error*error))))
        self.plan, self.start_s, self.dt = fused.copy(), float(start_s), float(dt)
        return fused, info
