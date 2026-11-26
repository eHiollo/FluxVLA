"""Full synchronous Wall-X action chunks; requires a verified mapping contract."""
_base_ = ["./a10_wallx_inference.py"]

wallx = dict(
    mode="motion", cycles=100, execute_horizon=6,
    # Original targets and gripper values are preserved, including row zero.
    interpolation_steps=3, adaptive_interpolation=True, max_interpolation_steps=600,
    max_control_hz=90., max_tcp_speed_m_s=.15, max_tcp_rotation_rad_s=.5,
    max_ee_step_m=.05, max_ee_step_rad=.15,
    max_chunk_displacement_m=.15, max_chunk_rotation_rad=.5,
    stop_between_chunks=True, print_progress=True,
)
