"""Dual A10 VR: relative anchors + reference + SDK tool cumulative targets.

Default VR-only; no recording, pose calibration or automatic motor enable. ZX grippers are opt-in.
Confirm the actual dual XML / GET row layout before opting into live motion.
"""

_base_ = ['./a10_vr_teleop.py', './a10_dual_arm_operator.py', './a10_devices.py']

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
    zx_grippers=dict(
        enabled=False, devices={{_base_.grippers}},
        target_rate_raw_s=1800., deadzone=.1, direction=dict(left=1, right=1),
        max_lead_raw=300., io_hz=20., input_timeout_s=.25, feedback_timeout_s=.5),
    pose_calibration_path=None,
    # Onsite .150: physical left arm is ee 1 (operator key 'right'), right is ee 0.
    # Swap input bindings only; keep GET mappings and UDP row order unchanged.
    input=dict(arm='both', hand_to_arm=dict(left='right', right='left'),
               # Both hands use the shared mapping; no left-only translation swap.
               enable_left_events=False, network=dict(host_ip='0.0.0.0')),
    operator={{_base_.operator}},
    diagnostics_path='work_dirs/a10/vr/dual_teleop_diagnostics.jsonl',
)
