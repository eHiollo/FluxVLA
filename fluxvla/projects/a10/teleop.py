"""Run the migrated XLeVR input through VROperator and A10Operator."""

import argparse
import os

from fluxvla.engines.operators.a10_operator import A10Operator
from fluxvla.engines.operators.gripper_operator import GripperOperator
from fluxvla.engines.operators.vr_operator import VROperator

from .config import DEFAULT_CONFIG, DEFAULT_SITE, load_project
from .vr.runtime import build_vr


def build_teleop(cfg, *, vr_only=False):
    vr, mapper = build_vr(cfg)
    robot = gripper = None
    settings = cfg['vr_operator']
    if not vr_only:
        robot = A10Operator(
            host=cfg['robot']['host'], ws_port=settings['ws_port'],
            udp_port=settings['udp_port'], joint_udp_port=settings['joint_udp_port'],
            command_mode='cartesian', allow_motion=cfg['robot']['allow_motion'],
            feedback_position_unit=settings['feedback_position_unit'])
        if settings['gripper_port']:
            gripper = GripperOperator(port=settings['gripper_port'])
    return VROperator(
        vr, mapper, robot, execute=not vr_only, gripper=gripper,
        max_position_step_m=settings['max_position_step_m'],
        max_rotation_step_rad=settings['max_rotation_step_rad'],
        max_feedback_age_s=settings['max_feedback_age_s'],
        gripper_speed_mm_s=settings['gripper_speed_mm_s'],
        max_position_tracking_error_m=settings['max_position_tracking_error_m'],
        max_rotation_tracking_error_rad=settings['max_rotation_tracking_error_rad'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vr-only', action='store_true',
                        help='Open XLeVR and process input without connecting to A10')
    args = parser.parse_args()
    cfg = load_project(os.environ.get('FLUXVLA_A10_CONFIG', DEFAULT_CONFIG),
                       os.environ.get('FLUXVLA_A10_SITE', DEFAULT_SITE))
    build_teleop(cfg, vr_only=args.vr_only).run(fps=cfg['vr']['control_fps'])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
