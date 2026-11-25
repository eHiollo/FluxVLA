"""LongWAM HTTP inference; start by recording predictions without motion."""
_base_ = ["./a10_dual_inference.py"]
model_backend = "longwam"
