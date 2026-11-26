"""Relative-only Wall-X retargeting: first future prediction -> current A10 EE.

This removes each chunk's absolute first-pose bias, and also discards the real
motion in that first future action. It does not fix model statistics or scale.
Model observations remain actual A10 measurements through the existing contract.
"""
_base_ = ["./a10_wallx_timed_ee.py"]

wallx = dict(
    ee_action_mode="delta",
    delta_translation_scale=1.,
    delta_rotation_scale=1.,
)
