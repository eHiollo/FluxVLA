"""Dual A10 VR: relative anchors + reference + SDK tool cumulative targets.

Onsite default: live dual-arm VR, ZX grippers and controller state recording.
No automatic motor enable. Use --preview for configuration checks without devices.
"""

_base_ = ['./a10_vr_teleop.py', './a10_dual_arm_operator.py', './a10_devices.py']

teleop = dict(
    vr_only=False, observe_only=False, record_controller=True,
    control_mode='reference', pose_mapping='anchor_relative',
    input_recovery_stable_s=.5,  # 松手回中、追踪稳定后，重新按侧键才恢复。
    # 现场出现无输入晚运动：双手松开时结束原生跟随并确认停止，不持续重发旧保持包。
    # 正常停止确认后，新的侧键按下可直接重新接管，不要求再松一次。
    stop_on_both_release=True,
    # 查询失败保留VR界面，但冻结新目标；不使用过期反馈继续运动或自动重连。
    keep_session_on_query_error=True,
    cartesian_adapter='sdk_tool_cumulative', fps=30,
    # 2026-11-12：保留已验收的运动参数，按用户要求默认开启实机与夹爪。
    # 以下约束前端末端目标；不是电机限速或碰撞距离。
    controller_speed_percent=70,
    reference_options=dict(
        max_linear_speed_m_s=.17, max_angular_speed_rad_s=1.4,
        max_linear_accel_m_s2=1., max_angular_accel_rad_s2=6.,
        position_gain=10., rotation_gain=12.,
        # 增量速度模式不补追旧旋转；单臂/绝对姿态模式保持原算法。
        rotation_follow_mode='incremental_velocity',
        rotation_velocity_deadzone_rad_s=.03, max_angular_brake_rad_s2=10.,
        # 现场开发：误差只记录，关闭软领先减速；运动仍受速度/加速度/制动约束。
        # 需要恢复超差停止时设True；设置具体lead数值可重新启用软领先限制。
        enforce_tracking_error_limits=False,
        max_reference_lead_m=None, max_reference_lead_rad=None,
    ),
    zx_grippers=dict(
        # 2026-11-06: user-confirmed live settings; device checks retain 5%.
        enabled=True, devices=dict(
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
    operator=dict(_base_.operator, allow_motion=True),
    diagnostics_path='work_dirs/a10/vr/dual_teleop_diagnostics.jsonl',
)
