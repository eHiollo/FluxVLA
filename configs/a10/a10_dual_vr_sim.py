"""Dual A10 VR on the virtual controller. The viewer owns port 5999."""

_base_ = './a10_dual_vr.py'

teleop = dict(
    fps=30,
    controller_speed_percent=20,
    confirm_stop=True,
    diagnostics_path='work_dirs/a10/vr/dual_sim_diagnostics.jsonl',
    operator=dict(
        host='192.168.0.198',
        ws_port=5888,
        status_ws_port=5888,
        shared_control=None,
    ),
    zx_grippers=dict(enabled=False),
    reference_options=dict(
        max_linear_speed_m_s=.06,
        max_angular_speed_rad_s=.45,
        max_linear_accel_m_s2=.4,
        max_angular_accel_rad_s2=1.5,
    ),
)
