"""Compatibility entrypoint: common A10 control with the native 50k backend."""
_base_ = ["./a10_dual_inference.py"]
model_backend = "fluxvla"
