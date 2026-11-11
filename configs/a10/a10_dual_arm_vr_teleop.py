"""Dual A10 VR: relative anchors + reference + SDK tool cumulative targets.

Default VR-only; no recording, pose calibration or automatic motor enable. ZX grippers are opt-in.
Confirm the actual dual XML / GET row layout before opting into live motion.
"""

_base_ = ['./a10_vr_teleop.py', './a10_dual_arm_operator.py', './a10_devices.py']

teleop = dict(
    vr_only=True, observe_only=False,
    control_mode='reference', pose_mapping='anchor_relative',
    input_recovery_stable_s=.5,  # 松手回中、追踪稳定后，重新按侧键才恢复。
    cartesian_adapter='sdk_tool_cumulative', fps=30,
    # 2026-11-11：固化用户本轮现场调参；启动实机仍需显式开放运动权限。
    # 以下约束前端末端目标；不是电机限速或碰撞距离。
    controller_speed_percent=70,
    reference_options=dict(
        max_linear_speed_m_s=.17, max_angular_speed_rad_s=1.1,
        max_linear_accel_m_s2=1., max_angular_accel_rad_s2=3.,
        position_gain=10., rotation_gain=12.,
        # 旋转软领先预算为0.1rad（约5.7度）；本组配置尚待现场复测。
        max_reference_lead_m=.03, max_reference_lead_rad=.1,
    ),
    zx_grippers=dict(
        # 2026-11-06: user-confirmed live settings; device checks retain 5%.
        enabled=False, devices=dict(
            left=dict(_base_.grippers.left, speed=15),
            right=dict(_base_.grippers.right, speed=15)),
        target_rate_raw_s=3000., deadzone=.1, direction=dict(left=1, right=1),
        max_lead_raw=600., io_hz=20., input_timeout_s=.25, feedback_timeout_s=.5),
    pose_calibration_path=None,
    # Onsite .150: physical left arm is ee 1 (operator key 'right'), right is ee 0.
    # Swap input bindings only; keep GET mappings and UDP row order unchanged.
    input=dict(arm='both', hand_to_arm=dict(left='right', right='left'),
               # Both hands use the shared mapping; no left-only translation swap.
               enable_left_events=False, network=dict(host_ip='0.0.0.0')),
    operator={{_base_.operator}},
    diagnostics_path='work_dirs/a10/vr/dual_teleop_diagnostics.jsonl',
)
