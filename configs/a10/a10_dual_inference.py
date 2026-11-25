"""Common dual A10 inference control; select only the remote model backend.

Default: FluxVLA cloth-folding 50k. Select openpi or longwam with --model or
A10_MODEL_BACKEND. LongWAM starts read-only; robot motion limits stay shared.
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
# timed_aj owns tracking parameters; preserve site motor configuration and speed.
operator = dict(joint_follower_filter=False, joint_follower_velocity_percent=None,
                motor_acceleration_deg_s2=None)

# Only position/speed/current are polled during streaming. Each serial link
# shares reads/writes: 100 Hz is the polling ceiling, missed slots are skipped.
# Initial alarm/enable checks remain; never wait for gripper arrival.
_gripper_inference = dict(
    speed=30, io_hz=100., feedback_mode="motion", serial_timeout_s=.02,
    drop_alarm_policy="warn", compact_trigger=True,
)
grippers = dict(left=dict(_gripper_inference), right=dict(_gripper_inference))

inference = dict(
    type="A10DualInferenceRunner",
    mode="read_only" if model_backend == "longwam" else "motion",
    manual_speed_percent=None,
    timed_aj=dict(
        enabled=True, plan_ws_port=5888, udp_host='192.168.0.150', udp_port=10000,
        # Controller network entry configured onsite (2026-11-25).
        # Readiness records site configuration; UDP has no per-target ACK.
        network_ready=True,
    ),
    cycles=1_000_000,
    remote_inference=dict(**_model["remote_inference"], timeout_s=3600.0),
    prompt=_model["prompt"],
    execute_horizon=20,
    action_dt=1 / 30.0,
    # Send each of the twenty original policy points once at 30Hz.
    # Ordinary blocks share one stream and do not wait for arrival or stop.
    # Updated downstream holds the last received point during inference gaps.
    # Each block keeps 30 Hz pacing without catching up across that hold.
    follower_tail=dict(
        enabled=False,
        arrival_timeout_s=30., poll_hz=50.,
        position_tolerance_deg=.5, speed_tolerance_deg_s=.5,
        stable_duration_s=.04, max_feedback_gap_s=.1,
    ),
    # Optional read-only 100Hz feedback while the synchronous model call waits.
    trace_inference_feedback=False,
    max_joint_step_rad=0.2,
)

# These transforms run on the native model server, never on the robot client.
for _key in ("dataset", "denormalize_action", "task_suite_name"):
    if _key in _model:
        inference[_key] = _model[_key]
inference["debug_log"] = dict(enabled=True, root_dir="work_dirs/a10/inference_logs", image_every_n_cycles=0)
