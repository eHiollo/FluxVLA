"""Dual A10 follower using the native OpenPI server; preview by default."""

_base_ = ["./a10_dual_arm_operator.py", "./a10_devices.py"]
inference = dict(
    type="A10DualInferenceRunner",
    mode="preview",
    cycles=2,
    remote_inference=dict(
        protocol="openpi_ws",
        server_host="127.0.0.1",
        server_port=8001,
        timeout_s=120.0,
        client_root="../openpi/packages/openpi-client/src",
    ),
    prompt=None,
    execute_horizon=10,
    action_dt=1 / 30.0,
    max_joint_step_rad=0.2,
)
