"""Dual A10 VR: relative anchors + reference + SDK tool cumulative targets.

Default VR-only; no recording, calibration, gripper or automatic motor enable.
Confirm the actual dual XML / GET row layout before opting into live motion.
"""

_base_ = ['./a10_vr_teleop.py', './a10_dual_arm_operator.py']

teleop = dict(
    vr_only=True, observe_only=False,
    control_mode='reference', pose_mapping='anchor_relative',
    cartesian_adapter='sdk_tool_cumulative', fps=30,
    # Initial low-speed trial values, independent of single-arm tuning.
    # These bound frontend targets, not physical joint speed or collision distance.
    reference_options=dict(
        max_linear_speed_m_s=.04, max_angular_speed_rad_s=.20,
        max_linear_accel_m_s2=.4, max_angular_accel_rad_s2=.96,
        max_reference_lead_m=.02, max_reference_lead_rad=.12,
    ),
    pose_calibration_path=None,
    input=dict(arm='both', hand_to_arm=dict(left='left', right='right'),
               enable_left_events=False, network=dict(host_ip='0.0.0.0')),
    operator={{_base_.operator}},
    diagnostics_path='work_dirs/a10/vr/dual_teleop_diagnostics.jsonl',
)
