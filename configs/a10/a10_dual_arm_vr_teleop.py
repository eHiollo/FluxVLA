"""Dual-hand relative anchors + reference, one controller (topology TEMPLATE).

Default VR-only; no recording, calibration, gripper or automatic motor enable.
Confirm the actual dual XML / GET row layout before opting into live motion.
"""

_base_ = ['./a10_vr_teleop.py', './a10_dual_arm_operator.py']

teleop = dict(
    vr_only=True, observe_only=False,
    control_mode='reference', pose_mapping='anchor_relative',
    # Preserve the existing dual-arm defaults; this rollout targets single-arm VR.
    reference_options=dict(max_reference_lead_m=None, max_reference_lead_rad=None),
    pose_calibration_path=None,
    input=dict(arm='both', hand_to_arm=dict(left='left', right='right'),
               enable_left_events=False, network=dict(host_ip='0.0.0.0')),
    operator={{_base_.operator}},
    diagnostics_path='work_dirs/a10/vr/dual_teleop_diagnostics.jsonl',
)
