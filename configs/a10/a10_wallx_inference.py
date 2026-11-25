"""Wall-X synchronous EE loop, using the existing A10 devices and SDK Tool path."""
_base_ = ["./a10_dual_arm_operator.py", "./a10_dual_devices.py"]

# Preserve site motor limits. The EE path uses UDP 9998, not timed_aj UDP 10000.
operator = dict(motor_acceleration_deg_s2=None, joint_follower_velocity_percent=None)

wallx = dict(
    mode="preview", cycles=1, prompt="fold the black cloth using both arms",
    geometry="configs/a10/a10_wallx_geometry.json",
    timeout_s=120., execute_horizon=6, action_dt=.0337, interpolation_steps=3,
    max_robot_age_s=.15, max_camera_age_s=.15, max_gripper_age_s=.2, max_skew_s=.15,
    max_ee_step_m=.02, max_ee_step_rad=.10,
    max_control_step_m=.005, max_control_step_rad=.04,
    max_chunk_displacement_m=.10, max_chunk_rotation_rad=.5,
    max_observation_drift_m=.02, max_observation_drift_rad=.15,
    max_tracking_error_m=.05, max_tracking_error_rad=.35,
    max_send_lag_s=.10, feedback_every_n_steps=3,
    arrival_timeout_s=10., arrival_position_tolerance_m=.005,
    arrival_rotation_tolerance_rad=.05, arrival_stable_s=.04,
    min_grasp_height_above_table_m=.005, min_grasp_separation_m=.10,
    manual_speed_percent=None,
    log_root="work_dirs/a10/wallx_inference",
)
