"""Compatibility entrypoint: common A10 control with the native 50k backend."""
_base_ = ["./a10_dual_inference.py"]
import importlib

model_backend = "fluxvla"
_native = importlib.import_module("fluxvla.engines.utils.a10_inference_backends").backend_settings(
    model_backend, _base_.model_checkpoint_dir)
inference = dict(_native, mode="motion")
inference["remote_inference"] = dict(_delete_=True, **_native["remote_inference"], timeout_s=3600.)
