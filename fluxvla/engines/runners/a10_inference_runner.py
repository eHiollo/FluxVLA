"""Synchronous KAANH A10 inference runner; hardware timing belongs to operator."""

import logging
import json
import math
import time
import traceback
from collections import deque
from types import SimpleNamespace

import numpy as np
import torch

from ..utils.root import RUNNERS
from ..utils.a10_debug_log import create_a10_debug_log
from .base_inference_runner import BaseInferenceRunner

logger = logging.getLogger(__name__)


@RUNNERS.register_module()
class A10InferenceRunner(BaseInferenceRunner):
    """Observe, predict, execute K targets, then observe again (without ROS).

    get_frame supplies actual qpos[7] (rad/mm), HWC uint8 RGB base_0_rgb and
    right_wrist_0_rgb, and has_error. Extra metadata is preserved. Blocking
    trajectory return means sending finished, not physical arrival. Remote
    predictions must already be absolute rad/mm targets. Local predictions
    use the configured denormalizer with the original observation state.
    """

    def __init__(self, *, execute_horizon=None, action_dt=0.1, joint_target_repeat=1, dry_run=True,
                 instruction='', max_joint_step_rad=None,
                 gripper_mm_range=(0., 85.), model_from_config=False, debug_log=None,
                 rtc_config=None, **kwargs):
        if model_from_config and (kwargs.get('ckpt_path') is not None
                                  or kwargs.get('remote_inference') is not None):
            raise ValueError('model_from_config cannot combine with ckpt_path or remote inference')
        kwargs.setdefault('action_chunk', 30)
        kwargs.setdefault('state_dim', 7)
        kwargs.setdefault('task_suite_name', 'a10')
        kwargs.setdefault('camera_names', ['base_0_rgb', 'right_wrist_0_rgb'])
        horizon = kwargs['action_chunk'] if execute_horizon is None else execute_horizon
        for name, value in [('action_chunk', kwargs['action_chunk']),
                            ('execute_horizon', horizon)]:
            if type(value) is not int or value <= 0:
                raise ValueError(f'{name} must be a positive integer')
        maximum = kwargs.get('max_publish_step', 10000)
        if maximum is not None and (type(maximum) is not int or maximum <= 0):
            raise ValueError('max_publish_step must be a positive integer or None')
        if horizon > kwargs['action_chunk']:
            raise ValueError('execute_horizon cannot exceed action_chunk')
        from ..utils.a10_rtc import validate_rtc
        self.rtc_config = validate_rtc(rtc_config, kwargs['action_chunk'], horizon)
        self._rtc_remaining = None
        if not math.isfinite(action_dt) or action_dt <= 0:
            raise ValueError('action_dt must be positive and finite')
        if type(joint_target_repeat) is not int or joint_target_repeat <= 0:
            raise ValueError('joint_target_repeat must be a positive integer')
        if type(dry_run) is not bool:
            raise ValueError('dry_run must be bool')
        if kwargs['state_dim'] != 7 or kwargs['camera_names'] != [
                'base_0_rgb', 'right_wrist_0_rgb']:
            raise ValueError('A10 requires 7D state and base/right-wrist cameras')
        if not kwargs.get('operator'):
            raise ValueError('Configure the A10 joint operator explicitly')
        if max_joint_step_rad is not None and (
                not math.isfinite(max_joint_step_rad) or max_joint_step_rad <= 0):
            raise ValueError('max_joint_step_rad must be positive and finite')
        if not dry_run and max_joint_step_rad is None:
            raise ValueError('Set max_joint_step_rad before enabling execution')
        limits = np.asarray(gripper_mm_range, dtype=float)
        if limits.shape != (2,) or not np.isfinite(limits).all() or not (
                0 <= limits[0] < limits[1] <= 100):
            raise ValueError('gripper_mm_range must be an increasing range in 0..100')
        self.execute_horizon = horizon
        self.action_dt = float(action_dt)
        self.joint_target_repeat = joint_target_repeat
        self.dry_run = dry_run
        self.instruction = instruction
        self.max_joint_step_rad = max_joint_step_rad
        self.gripper_mm_range = limits
        self._ready = self._follower_requested = self._cleaned = False
        self.last_actions = self.last_cycle = None
        self._recorder = None
        self._debug_cycle = 0
        self._debug_phase = 'setup'
        self._last_trajectory_cycle = None
        super().__init__(**kwargs)
        config = kwargs.get('cfg')
        metadata = dict(resolved_config=(config.to_dict() if hasattr(config, 'to_dict') else kwargs),
                        instruction=instruction, dry_run=dry_run, execute_horizon=horizon,
                        action_dt=action_dt, joint_target_repeat=joint_target_repeat,
                        joint_publish_hz=joint_target_repeat / action_dt, max_publish_step=maximum,
                        max_joint_step_rad=max_joint_step_rad,
                        mode='remote' if self._use_remote else 'local')
        self._recorder = create_a10_debug_log(debug_log, role='client', metadata=metadata)
        if self._recorder is not None:
            self.ros_operator.record_trajectory_trace = True
        if model_from_config:
            from fluxvla.engines import (build_dataset_from_cfg,
                                         build_transform_from_cfg, build_vla_from_cfg)
            try:
                self.dataset = build_dataset_from_cfg(kwargs['dataset'])
                self.denormalize_action = build_transform_from_cfg(kwargs['denormalize_action'])
                self.vla = build_vla_from_cfg(kwargs['cfg'].inference_model)
            except BaseException:
                self.cleanup()
                raise

    def _predict_action(self, inputs):
        rtc_request = None
        rtc_info = None
        if self.rtc_config.get('enabled', False):
            rtc_request = dict(config=self.rtc_config, absolute_actions=self._rtc_remaining)
            if self._use_remote:
                inputs['a10_rtc_config'] = json.dumps(self.rtc_config)
                if self._rtc_remaining is not None:
                    inputs['a10_rtc_actions'] = self._rtc_remaining
        if self._use_remote and self._recorder is not None:
            inputs['debug_request_id'] = f'{self._recorder.run_id}:{self._debug_cycle}'
        if not self._use_remote:
            device = next(self.vla.parameters()).device
            inputs = {key: value.to(device) if isinstance(value, torch.Tensor)
                      else value for key, value in inputs.items()}
            if rtc_request is not None:
                from ..utils.a10_rtc import inject_rtc
                rtc_info = inject_rtc(inputs, rtc_request, state=self._action_ctx.state,
                                      denormalizer=self.denormalize_action, model=self.vla)
        output = super()._predict_action(inputs)
        if rtc_request is not None:
            if self._use_remote:
                rtc_info = (self.last_profile.get('server_debug') or {}).get('rtc')
                if not rtc_info or rtc_info.get('enabled') is not True:
                    raise RuntimeError('Model server did not acknowledge A10 RTC; restart the model server with updated code')
                if self._rtc_remaining is not None and not rtc_info.get('applied'):
                    raise RuntimeError('Model server did not apply the requested RTC guidance')
            logger.info('A10 RTC guidance: applied=%s, prefix=%d model steps; synchronous execution',
                        rtc_info['applied'], rtc_info['prefix_len'])
            if self._recorder is not None:
                self._recorder.record('rtc_guidance', cycle=self._debug_cycle, **rtc_info)
        if self._recorder is not None:
            extra = {} if self._use_remote else {
                key: value for key, value in inputs.items()
                if key in ('states', 'lang_tokens', 'lang_masks', 'img_masks')}
            self._recorder.record('model_output', cycle=self._debug_cycle,
                                  values=output, value_space=('absolute_rad_mm' if self._use_remote
                                                             else 'normalized_model_output'),
                                  model_inputs=extra,
                                  remote_profile=getattr(self, 'last_profile', None))
        return output

    def run_setup(self):
        if self._cleaned:
            raise RuntimeError('Create a new runner after cleanup')
        if self._ready:
            return
        try:
            if self.ros_operator.command_mode != 'joint':
                raise ValueError('A10 inference requires command_mode="joint"')
            super().run_setup()
            logger.info('A10 joint stream: %.1f Hz, %d sends per model target; model %.1f Hz, USB gripper once per target',
                        self.joint_target_repeat / self.action_dt, self.joint_target_repeat, 1. / self.action_dt)
            self.ros_operator.connect()
            self.update_observation_window()
            self._ready = True
        except BaseException as exc:
            if self._recorder is not None:
                self._recorder.record('setup_error', error=repr(exc), traceback=traceback.format_exc())
            self.cleanup()
            raise

    def update_observation_window(self):
        frame = self.ros_operator.get_frame()
        if not isinstance(frame, dict) or frame.get('has_error') is not False:
            raise RuntimeError('A10 frame is missing or reports unknown/fault status')
        state = np.asarray(frame.get('qpos'), dtype=np.float32)
        if state.shape != (7,) or not np.isfinite(state).all():
            raise ValueError('A10 qpos must contain seven finite actual values')
        observation = dict(frame, qpos=state.copy())
        for name in self.camera_names:
            image = np.asarray(frame.get(name))
            if (image.ndim != 3 or image.shape[-1] != 3
                    or min(image.shape[:2]) == 0 or image.dtype != np.uint8):
                raise ValueError(f'{name} must be nonempty HWC uint8 RGB')
            observation[name] = image.copy()
        if self.observation_window is None:
            self.observation_window = deque(maxlen=1)
        self.observation_window.append(observation)
        if self._recorder is not None:
            frequency = self._recorder.image_every_n_cycles
            images = ({key: observation[key] for key in self.camera_names}
                      if frequency and self._debug_cycle % frequency == 0 else None)
            self._recorder.record('observation', cycle=self._debug_cycle, images=images,
                                  feedback={k: v for k, v in observation.items()
                                            if k not in self.camera_names},
                                  feedback_age_s=(time.monotonic() - observation['received_monotonic_s']
                                                  if 'received_monotonic_s' in observation else None))
        return observation

    def _postprocess_actions(self, raw_action):
        # Remove singleton batch before the base class slices the time axis.
        raw = torch.as_tensor(raw_action)
        if raw.ndim == 3 and raw.shape[0] == 1:
            raw = raw[0]
        if raw.ndim != 2 or not len(raw) or raw.shape[1] < 7:
            raise ValueError('Model actions must be [T,D>=7] or [1,T,D>=7]')
        actions = np.asarray(super()._postprocess_actions(raw), dtype=np.float32)
        if actions.ndim != 2 or actions.shape[1] != 7 or not np.isfinite(actions).all():
            raise ValueError('Processed actions must be finite [T,7] absolute targets')
        if self._recorder is not None:
            self._recorder.record('denormalized_actions_before_clamp', cycle=self._debug_cycle,
                                  actions=actions)
        low, high = self.gripper_mm_range
        outside = (actions[:, 6] < low) | (actions[:, 6] > high)
        if np.any(outside):
            logger.warning(
                'A10 model gripper targets %.3f..%.3f mm: clamping %d rows to %.3f..%.3f mm',
                float(actions[:, 6].min()), float(actions[:, 6].max()),
                int(outside.sum()), low, high)
            actions = actions.copy()
            actions[:, 6] = np.clip(actions[:, 6], low, high)
        return np.ascontiguousarray(actions)

    def _execute_actions(self, actions, rate=None):
        del rate  # Operator owns the clock; no ROS Rate or second sleep.
        low, high = self.gripper_mm_range
        if np.any((actions[:, 6] < low) | (actions[:, 6] > high)):
            raise ValueError('Gripper target exceeds configured range')
        previous = np.concatenate([
            self._action_ctx.state[None, :6], actions[:-1, :6]], axis=0)
        changes = np.abs(actions[:, :6] - previous)
        step, joint = np.unravel_index(np.argmax(changes), changes.shape)
        largest = float(changes[step, joint])
        if self.dry_run:
            logger.warning(
                'A10 dry-run planned largest joint step: %.6f rad (%.2f deg), '
                'action %d/%d, joint %d; actual/previous %.6f -> target %.6f rad',
                largest, math.degrees(largest), step + 1, len(actions), joint + 1,
                float(previous[step, joint]), float(actions[step, joint]))
        if self.max_joint_step_rad is not None and largest > self.max_joint_step_rad:
            raise ValueError(
                f'Joint target step exceeds configured limit: action {step + 1}/{len(actions)}, '
                f'joint {joint + 1}, {largest:.6f} rad ({math.degrees(largest):.2f} deg) '
                f'> {self.max_joint_step_rad:.6f} rad; no motion target sent')
        self.last_actions = actions.copy()
        if self.dry_run:
            return
        if not self._follower_requested:
            # Attempt stop even if start sends a command but confirmation fails.
            self._follower_requested = True
            self.ros_operator.start_follower()
        self._last_trajectory_cycle = self._debug_cycle
        self.ros_operator.execute_trajectory(
            arm_trajectories={'right': actions[:, :6]},
            gripper_trajectories={'right': actions[:, 6]},
            dt=self.action_dt, joint_target_repeat=self.joint_target_repeat, async_exec=False)

    def run(self, initial_instruction=None):
        instruction = self.instruction if initial_instruction is None else initial_instruction
        steps = 0
        previous_last_action = None
        try:
            if not isinstance(instruction, str) or not instruction.strip():
                raise ValueError('Provide a nonempty A10 task instruction')
            self.run_setup()
            with torch.inference_mode():
                while self.max_publish_step is None or steps < self.max_publish_step:
                    self._debug_cycle += 1
                    started = time.monotonic()
                    self._action_ctx = SimpleNamespace(instruction=instruction)
                    self._debug_phase = 'observation_preprocess'
                    inputs = self._preprocess(instruction)
                    self._debug_phase = 'model_inference'
                    infer_started = time.monotonic()
                    with torch.autocast('cuda', dtype=self.mixed_precision_dtype,
                                        enabled=self.enable_mixed_precision and not self._use_remote):
                        raw = self._predict_action(inputs)
                    infer_s = time.monotonic() - infer_started
                    self._debug_phase = 'action_postprocess'
                    actions = self._postprocess_actions(raw)
                    count = min(len(actions), self.execute_horizon)
                    if self.max_publish_step is not None:
                        count = min(count, self.max_publish_step - steps)
                    execution_started = time.monotonic()
                    if self._recorder is not None:
                        chosen = actions[:count]
                        previous = np.concatenate([self._action_ctx.state[None, :6], chosen[:-1, :6]])
                        velocity = (chosen[:, :6] - previous) / self.action_dt
                        self._recorder.record('planned_actions', cycle=self._debug_cycle,
                                              actions=actions, selected_actions=chosen,
                                              largest_joint_step_rad=float(np.abs(chosen[:, :6] - previous).max()),
                                              nominal_target_velocity_rad_s=velocity,
                                              nominal_target_acceleration_rad_s2=np.diff(velocity, axis=0) / self.action_dt,
                                              cross_chunk_target_jump_rad=(None if previous_last_action is None
                                                  else chosen[0, :6] - np.asarray(previous_last_action[:6])))
                    self._debug_phase = 'preflight_or_execution'
                    self._execute_actions(actions[:count])
                    if self.rtc_config.get('enabled', False):
                        self._rtc_remaining = actions[count:].copy()
                    steps += count
                    self._prev_ctx = self._action_ctx
                    self.last_cycle = dict(
                        steps=steps, predicted=len(actions), selected=count,
                        executed=0 if self.dry_run else count, infer_s=infer_s,
                        joint_target_repeat=self.joint_target_repeat,
                        joint_publish_hz=self.joint_target_repeat / self.action_dt,
                        observed_qpos=self._action_ctx.state.tolist(),
                        first_action=actions[0].tolist(),
                        last_action=actions[count - 1].tolist(),
                        previous_last_action=previous_last_action,
                        tracking_error_at_observation_rad=(None if previous_last_action is None
                            else (self._action_ctx.state[:6]
                                  - np.asarray(previous_last_action[:6])).tolist()),
                        execution_s=time.monotonic() - execution_started,
                        elapsed_s=time.monotonic() - started, dry_run=self.dry_run)
                    logger.info('A10 cycle: %s', self.last_cycle)
                    if self._recorder is not None:
                        self._recorder.record('cycle', cycle=self._debug_cycle,
                                              result=self.last_cycle,
                                              trajectory=getattr(self.ros_operator, 'last_trajectory', None))
                    if not self.dry_run:
                        previous_last_action = actions[count - 1].tolist()
                    if self._poll_keyboard_pause():
                        logger.info('A10 paused after the current chunk; ending run')
                        if self._recorder is not None:
                            self._recorder.record('run_end', reason='keyboard_pause', selected_steps=steps)
                        break
            if self._recorder is not None and self.max_publish_step is not None and steps >= self.max_publish_step:
                self._recorder.record('run_end', reason='max_publish_step', selected_steps=steps)
            logger.info('A10 run finished after %d selected steps (dry_run=%s)',
                        steps, self.dry_run)
        except BaseException as exc:
            trajectory = getattr(self.ros_operator, 'last_trajectory', None)
            if self._recorder is not None:
                self._recorder.record('run_error', cycle=self._debug_cycle, phase=self._debug_phase,
                                      trajectory_cycle=self._last_trajectory_cycle, error=repr(exc),
                                      traceback=traceback.format_exc(), trajectory=trajectory)
            if trajectory is not None:
                logger.warning('A10 last trajectory send result: %s', trajectory)
            logger.warning('A10 run ended: %s: %s', type(exc).__name__, exc)
            raise
        finally:
            self.cleanup()

    def cleanup(self):
        if self._cleaned:
            return
        self._cleaned = True
        self._ready = False
        try:
            if self._follower_requested:
                try:
                    state = self.ros_operator.stop_follower()
                    if self._recorder is not None:
                        self._recorder.record('follower_stop',
                                              confirmed=(getattr(state, 'follower_active', None) is False
                                                         or getattr(self.ros_operator, 'last_stop_confirmed', False)),
                                              scope='this_session',
                                              state=getattr(state, 'raw', None))
                except Exception:
                    logger.exception('A10 follower stop failed; controller state is unconfirmed')
                    if self._recorder is not None:
                        self._recorder.record('follower_stop', confirmed=False, traceback=traceback.format_exc())
        finally:
            self._follower_requested = False
            try:
                self.ros_operator.close()
            finally:
                try:
                    super().cleanup()
                finally:
                    if self._recorder is not None:
                        self._recorder.record('cleanup_complete')
                        self._recorder.close()
