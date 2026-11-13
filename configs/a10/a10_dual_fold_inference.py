"""Native FluxVLA dual-fold model server and dual follower client configuration."""

_base_ = ["./a10_dual_arm_operator.py", "./a10_devices.py"]
import os
from pathlib import Path

_root = Path("{{fileDirname}}").resolve().parents[1]
_checkpoint = Path(os.environ.get("A10_DUAL_CHECKPOINT", str(_root / "checkpoints/FluxVLA-A10-Dual-Fold-10steps")))
# Read the architecture only when the server builds the model. Remote clients
# can load this same config without downloading any checkpoint assets.
inference_model = dict(
    type="NativeA10Policy",
    checkpoint_dir=str(_checkpoint),
    weight_file="checkpoints/model.safetensors",
    model_config_file="model_config.json",
    model_overrides=dict(num_steps=10),
)
_stats = str(_checkpoint / "dataset_statistics.json")
inference = dict(
    type="A10DualInferenceRunner",
    mode="preview",
    cycles=2,
    remote_inference=dict(
        protocol="fluxvla_zmq",
        server_host="127.0.0.1",
        server_port=5556,
        timeout_s=120.0,
    ),
    prompt="Fold the garment neatly using both arms.",
    execute_horizon=10,
    action_dt=1 / 30.0,
    max_joint_step_rad=0.2,
    task_suite_name="a10_dual",
    dataset=dict(
        type="A10NativeInferenceDataset",
        norm_stats=_stats,
        tokenizer_path=str(_checkpoint / "tokenizer"),
        state_dim=14,
        statistic_name="a10_dual",
        norm_type="quantile",
        camera_names=["base_0_rgb", "left_wrist_0_rgb", "right_wrist_0_rgb"],
    ),
    denormalize_action=dict(
        type="A10NativeDenormalizeAction",
        norm_stats=_stats,
        state_dim=14,
        statistic_name="a10_dual",
        norm_type="quantile",
        relative_mask=([True] * 6 + [False]) * 2,
    ),
    debug_log=dict(enabled=True, root_dir="work_dirs/a10/inference_logs", image_every_n_cycles=0),
)
