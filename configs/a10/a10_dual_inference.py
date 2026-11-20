"""Common dual A10 inference control; select only the remote model backend.

Default: FluxVLA cloth-folding 50k. Switch with --model openpi or
A10_MODEL_BACKEND=openpi, without changing any robot control parameters.
"""
import os
from pathlib import Path
import importlib
_backend_settings = importlib.import_module("fluxvla.engines.utils.a10_inference_backends").backend_settings

model_backend = os.environ.get("A10_MODEL_BACKEND", "fluxvla")
model_checkpoint_dir = os.environ.get(
    "A10_DUAL_CHECKPOINT",
    str(Path("{{fileDirname}}").resolve().parents[1] / "checkpoints/FluxVLA-Cloth-Folding-04-50k"),
)
_model = _backend_settings(model_backend, model_checkpoint_dir)
inference_model = dict(
    type="NativeA10Policy", checkpoint_dir=model_checkpoint_dir,
    weight_file="checkpoints/model.safetensors", model_config_file="model_config.json",
    model_overrides=dict(num_steps=10),
)

_base_ = ["./a10_dual_arm_operator.py", "./a10_dual_devices.py"]
# Robot-side follower upper limits come from the shared operator config.
# Speed uses operator.joint_follower_velocity_percent; acceleration uses motor XML.

# With motor acceleration 375 deg/s^2 and manual speed 100%:
# follower 12% => 129.9 deg/s^2 and 18 / 21.6 deg/s.
operator = dict(joint_follower_filter=False, joint_follower_velocity_percent=12,
                motor_acceleration_deg_s2=375)

# Inference-only alarm handling; teleoperation and console defaults are unchanged.
grippers = dict(left=dict(drop_alarm_policy="warn"), right=dict(drop_alarm_policy="warn"))

inference = dict(
    type="A10DualInferenceRunner",
    mode="motion",
    manual_speed_percent=100,
    cycles=1_000_000,
    remote_inference=dict(**_model["remote_inference"], timeout_s=3600.0),
    prompt=_model["prompt"],
    execute_horizon=20,
    action_dt=1 / 30.0,
    # Preserve twenty 30Hz policy points; repeat each twice on the 60Hz wire.
    # Verify position, stop, then observe; no synthetic compensation targets.
    follower_tail=dict(
        enabled=True,
        arrival_timeout_s=30., poll_hz=30.,
        position_tolerance_deg=.5, speed_tolerance_deg_s=None,
        stable_duration_s=0., max_feedback_gap_s=.1,
    ),
    # Optional read-only 30Hz feedback while the synchronous model call waits.
    trace_inference_feedback=False,
    max_joint_step_rad=0.2,
)

# These transforms run on the native model server, never on the robot client.
for _key in ("dataset", "denormalize_action", "task_suite_name"):
    if _key in _model:
        inference[_key] = _model[_key]
inference["debug_log"] = dict(enabled=True, root_dir="work_dirs/a10/inference_logs", image_every_n_cycles=0)
