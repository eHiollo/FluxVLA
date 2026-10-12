"""Compatibility entry point for A10 camera capture."""

from fluxvla.engines.operators.camera_operator import CameraOperator, RGBCamera


def build_cameras(cfg):
    return CameraOperator(cfg['cameras']).cameras
