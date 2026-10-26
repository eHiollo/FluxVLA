"""Native VR-only defaults. Confirm feedback units before live operation.

python scripts/teleoperate.py --config configs/a10/a10_vr_teleop.py
Override parameters with --cfg-options teleop.input.network.host_ip=YOUR_LAN_IP.
No site YAML or projects/a10 import is used. No automatic motor enable/homing.
"""
from pathlib import Path

_root = Path('{{fileDirname}}').resolve().parents[1]
teleop = dict(
    vr_only=True, fps=30, max_steps=None,
    # delta: existing baseline. reference: migrated frontend target generation.
    # --observe reads controller feedback but never starts/sends/stops motion.
    control_mode='delta', observe_only=False,
    # Optional one-run controller percentage (1..100); None leaves it unchanged.
    # Applied/read back before follower start; restored only after confirmed stop.
    controller_speed_percent=None,
    # Opt-in single-arm reference adapter; does not enable feedback compensation.
    cartesian_adapter='legacy_anchor',  # or 'sdk_tool_cumulative'
    # Used only in reference mode. Offline-selected initial values, not hardware acceptance.
    # Outer half of each budget gradually slows outward reference progress.
    reference_options=dict(max_reference_lead_m=.02, max_reference_lead_rad=.12),
    # Old mapping stays the default. New mode requires control_mode='reference'.
    pose_mapping='anchor_relative',  # or 'calibrated_absolute'
    # Tracking space -> robot base still requires an intentional reference pose.
    # Tool axes -> hand reuses inverse(AXIS_REMAP_VR_TO_ROBOT) from old teleop.
    # This preserves the local-axis convention, not a measured world calibration.
    pose_calibration=dict(
        vr_to_robot_quat_xyzw=None,
        hand_to_tool_quat_xyzw=[0.7071067811865475, 0.7071067811865475, 0.0, 0.0],
    ),
    # Site result, written by --calibrate-pose; auto-loaded only in absolute mode.
    pose_calibration_path=str(_root / 'configs/a10/vr_calibration.local.json'),
    diagnostics_path=str(_root / 'work_dirs/a10/vr/teleop_diagnostics.jsonl'),
    input=dict(type='VROperator', arm='right',
               network=dict(host_ip='127.0.0.1', https_port=8443,
                            websocket_port=8442,
                            # Keep migrated VR certificates separate from the
                            # main checkout's pre-existing local TLS identity.
                            tls_directory=str(_root / 'work_dirs/a10/vr/tls_vr')),
               mapping={'pos_scale': 0.9,
 'angle_scale': 1.3,
 'vr_to_robot_scale': 1.0,
 'fine_trigger_threshold': 0.5,
 'fine_scale_factor': 0.5,
 'position_control_mode': 'safe_frame_delta',
 'stale_timeout_s': 0.25,
 'max_vr_speed_m_s': 2.0,
 'spike_recovery_frames': 2,
 'compute_anchor_shadow': True,
 'motion_shaping_mode': 'robot_controller',
 'position_cutoff_hz': 5.0,
 'max_linear_speed_m_s': 0.25,
 'max_linear_accel_m_s2': 1.0,
 'engage_ramp_s': 0.35,
 'pos_deadzone_m': 0.0005,
 'angle_deadzone_deg': 0.05,
 'max_delta_pos_m': None,
 'max_delta_angle_deg': None,
 'require_squeeze_to_move': True,
 'gripper_thumbstick_axis': 'x',
 'gripper_thumbstick_deadzone': 0.05,
 'axis_remap': [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]]}),
    operator=dict(type='A10Operator', host='127.0.0.1',
                  ws_port=5999, udp_port=9998, joint_udp_port=9999,
                  command_mode='cartesian', allow_motion=False,
                  # Preserve small negative angles; SDK pq normalizes them to ~2*pi.
                  cartesian_wire_format='pe321',
                  feedback_position_unit=None,
                  # Optional USB gripper: use gripper_transport='usb' and
                  # gripper=dict(port='YOUR_DEVICE', max_width_mm=85., timeout=.015).
                  gripper_transport='controller'),
    # ReferenceConfig already defaults to the original vr_vel 0.12 m/s and
    # 0.32 rad/s. Remove the old CLI overrides 0.01/0.05 for the speed trial.
    # At 30 Hz this is 4 mm / 0.01067 rad per frame; retain finite step guards.
    max_position_step_m=.005, max_rotation_step_rad=.015,
    max_feedback_age_s=.25, gripper_speed_mm_s=30.,
    max_position_tracking_error_m=.05,
    max_rotation_tracking_error_rad=.35,
)
