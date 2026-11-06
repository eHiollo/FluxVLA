"""Versioned recording contracts. No hardware access or training dependencies."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RecordingSchema:
    name: str
    state_names: tuple[str, ...]
    action_names: tuple[str, ...]
    cameras: tuple[str, ...]
    export_cameras: tuple[str, ...]

    @property
    def dual(self):
        return self.name == 'a10_dual_v1'

    @property
    def image_aliases(self):
        return {f'observation.images.{old}': f'observation.images.{new}'
                for old, new in zip(self.cameras, self.export_cameras)}

    def contract(self):
        if not self.dual:
            # Preserve the exact pre-existing contract for single-arm resume.
            return dict(version=1, state_units=['rad'] * 6 + ['mm'],
                        action_units=['bool_float'] + ['m'] * 3 + ['rad'] * 3 + ['normalized'],
                        action_source='legacy_vr_mapper_delta', training_source='observation.state')
        units = (['rad'] * 6 + ['normalized_closure']) * 2
        return dict(version=1, schema=self.name, state_names=list(self.state_names),
                    action_names=list(self.action_names), state_units=units,
                    action_units=list(units), action_source='same_frame_measured_state_copy',
                    training_source='observation.state', action_time_offset_frames=0,
                    physical_arm_order=['left', 'right'], physical_arm_ee_ids={'left': 1, 'right': 0},
                    gripper=dict(source='actual_position_raw', min_raw=0, max_raw=9000,
                                 formula='raw / 9000', open=0.0, closed=1.0),
                    camera_aliases=self.image_aliases)

    def validate_values(self, state, action):
        state = np.asarray(state, dtype=np.float32)
        action = np.asarray(action, dtype=np.float32)
        if (state.shape != (len(self.state_names),) or action.shape != (len(self.action_names),)
                or not np.isfinite(state).all() or not np.isfinite(action).all()):
            raise ValueError(f'{self.name}: expected finite {len(self.state_names)}D state '
                             f'and {len(self.action_names)}D action')
        if self.dual:
            if np.any(state[[6, 13]] < 0) or np.any(state[[6, 13]] > 1):
                raise ValueError('Dual gripper closure must be within 0..1 (raw / 9000)')
            if not np.array_equal(state, action):
                raise ValueError('Dual action must be the same-frame measured state copy, not a command')
        elif state[-1] < 0 or action[0] not in (0., 1.) or abs(action[-1]) > 1:
            raise ValueError('Invalid gripper width / enabled flag / normalized gripper action')


_SINGLE_STATE = tuple([f'joint_{i}.pos' for i in range(1, 7)] + ['gripper.pos'])
_DUAL_STATE = tuple(name for side in ('left', 'right')
                    for name in ([f'{side}.joint_{i}.pos' for i in range(1, 7)]
                                 + [f'{side}.gripper.closure']))
SCHEMAS = {
    'a10_single_v1': RecordingSchema('a10_single_v1', _SINGLE_STATE,
        ('ee.enabled', 'ee.delta_x', 'ee.delta_y', 'ee.delta_z', 'ee.delta_rx',
         'ee.delta_ry', 'ee.delta_rz', 'gripper.pos'),
        ('base_0_rgb', 'right_wrist_0_rgb'), ('top', 'right')),
    'a10_dual_v1': RecordingSchema('a10_dual_v1', _DUAL_STATE, _DUAL_STATE,
        ('base_0_rgb', 'left_wrist_0_rgb', 'right_wrist_0_rgb'), ('top', 'left', 'right')),
}


def get_schema(name='a10_single_v1'):
    if not isinstance(name, str) or name not in SCHEMAS:
        raise ValueError(f'Unknown A10 recording schema: {name!r}')
    return SCHEMAS[name]


def schema_from_info(info):
    # Legacy datasets have no explicit schema marker. Never infer dual from length alone.
    return get_schema(info.get('a10_recording_schema', 'a10_single_v1'))


def pack_dual_state(joints_by_ee_id, gripper_raw_by_side):
    """Pack measured feedback in PHYSICAL left/right order; no operator-key guessing.

    joints_by_ee_id: {1: six actual radians, 0: six actual radians}.
    gripper_raw_by_side: {'left': actual raw, 'right': actual raw}.
    Missing/invalid feedback is rejected rather than filled from targets or zeros.
    """
    if set(joints_by_ee_id) != {0, 1} or set(gripper_raw_by_side) != {'left', 'right'}:
        raise ValueError('Both ee_id 0/1 and physical left/right gripper feedback are required')
    values = []
    for side, ee_id in (('left', 1), ('right', 0)):
        joints = np.asarray(joints_by_ee_id[ee_id], dtype=np.float32)
        raw = np.asarray(gripper_raw_by_side[side], dtype=np.float64)
        if joints.shape != (6,) or not np.isfinite(joints).all():
            raise ValueError(f'Invalid actual joints for ee_id={ee_id}')
        if raw.shape != () or not np.isfinite(raw) or not 0 <= raw <= 9000:
            raise ValueError(f'Invalid actual gripper raw for {side}: expected 0..9000')
        values.extend(joints.tolist()); values.append(float(raw) / 9000)
    return np.asarray(values, dtype=np.float32)
