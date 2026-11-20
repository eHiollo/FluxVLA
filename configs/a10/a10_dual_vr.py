"""A10 dual-arm VR: 90 Hz world translation and local rotation.

Robot connection/topology comes from a10_dual_arm_operator.py; device identities
come from a10_dual_devices.py. This file owns VR input, motion tuning and session policy.
Preview: python scripts/teleoperate.py --config configs/a10/a10_dual_vr.py --preview
Live defaults include both arms and ZX grippers; no automatic enable or homing.
"""

_base_ = ['./a10_dual_arm_operator.py', './a10_dual_devices.py']

from pathlib import Path

_root = Path('{{fileDirname}}').resolve().parents[1]

teleop = dict(
    vr_only=False, observe_only=False, fps=90, max_steps=None,
    control_mode='reference', cartesian_adapter='sdk_tool_cumulative',
    pose_mapping='world_translation_local_rotation',
    pose_calibration_path=None,
    pose_calibration=dict(
        vr_to_robot_quat_xyzw=None,
        hand_to_tool_quat_xyzw=[0.7071067811865475, 0.7071067811865475, 0.0, 0.0],
    ),
    # Controller runtime-log capacity is full; local JSONL diagnostics stay on.
    # Enable with --record-controller after managing controller log storage.
    record_controller=False,
    diagnostics_path='work_dirs/a10/vr/dual_world_translation_diagnostics.jsonl',
    input_recovery_stable_s=.5,
    resilient_takeover=True,
    # Normal release brakes/holds the last reference and keeps this session.
    # Re-grip does not stop/start/GET; input faults and exit still request stop.
    stop_on_both_release=False,
    confirm_stop=False,  # Send stop only; do not claim a confirmed physical stop.
    keep_session_on_query_error=True,
    # Control uses sent references; background measured GET is for recording.
    continuous_feedback=False, feedback_poll_hz=100,
    controller_speed_percent=70,
    # Bounds shape frontend targets, not motor limits or collision distances.
    reference_options=dict(
        max_linear_speed_m_s=.17, max_angular_speed_rad_s=1.4,
        max_linear_accel_m_s2=1., max_angular_accel_rad_s2=6.,
        position_gain=10., rotation_gain=12.,
        rotation_follow_mode='incremental_velocity',
        rotation_velocity_deadzone_rad_s=.03, max_angular_brake_rad_s2=10.,
        enforce_tracking_error_limits=False,
        max_reference_lead_m=None, max_reference_lead_rad=None,
    ),
    input=dict(
        type='VROperator', arm='both',
        # Physical left is ee 1 (operator alias right); physical right is ee 0.
        hand_to_arm=dict(left='right', right='left'),
        enable_left_events=False,
        network=dict(
            host_ip='0.0.0.0', https_port=8443, websocket_port=8442,
            tls_directory=str(_root / 'work_dirs/a10/vr/tls_vr'),
        ),
        mapping=dict(
            pos_scale=1.8, angle_scale=1.3, vr_to_robot_scale=1.0,
            fine_trigger_threshold=.5, fine_scale_factor=.5,
            position_control_mode='safe_frame_delta', stale_timeout_s=.25,
            max_vr_speed_m_s=2., spike_recovery_frames=2,
            compute_anchor_shadow=True, motion_shaping_mode='robot_controller',
            position_cutoff_hz=5., max_linear_speed_m_s=.25,
            max_linear_accel_m_s2=1., engage_ramp_s=.35,
            pos_deadzone_m=.0005, angle_deadzone_deg=.05,
            max_delta_pos_m=None, max_delta_angle_deg=None,
            require_squeeze_to_move=True, squeeze_translation_only=True,
            gripper_thumbstick_axis='x', gripper_thumbstick_deadzone=.05,
            axis_remap=[[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]],
        ),
        # Physical hands; world translation alone uses these base-axis matrices.
        mapping_by_hand=dict(
            left=dict(world_position_axis_remap=[
                [0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]]),
            right=dict(world_position_axis_remap=[
                [0., 0., 1.], [1., 0., 0.], [0., 1., 0.]]),
        ),
    ),
    # Session overrides do not mutate the shared console/inference defaults.
    # A recording GET failure must not revoke the VR target stream.
    operator=dict(
        _base_.operator, allow_motion=True, gripper_transport='controller',
        shared_control=dict(_base_.operator.shared_control, require_feedback=False),
    ),
    zx_grippers=dict(
        enabled=True, drop_alarm_policy='warn',
        # VR motor speed is 50%; the device-check default remains 5%.
        devices=dict(
            left=dict(_base_.grippers.left, speed=50),
            right=dict(_base_.grippers.right, speed=50),
        ),
        target_rate_raw_s=4500., deadzone=.1, direction=dict(left=1, right=1),
        serial_timeout_s=.05, max_lead_raw=900., io_hz=20.,
        input_timeout_s=.25, feedback_timeout_s=.5,
    ),
    # Retained runtime fields; fixed step bounds are dormant in reference mode.
    max_position_step_m=.005, max_rotation_step_rad=.015,
    max_feedback_age_s=.25, gripper_speed_mm_s=30., gripper_direction=1,
    max_position_tracking_error_m=.05, max_rotation_tracking_error_rad=.35,
)
