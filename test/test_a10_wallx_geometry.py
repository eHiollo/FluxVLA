"""A10 frame, TCP, and hand binding regressions without opening hardware."""
import copy
import importlib.util
import json
from pathlib import Path
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("a10_wallx_test_geometry", ROOT / "fluxvla/projects/a10/wallx.py")
g = importlib.util.module_from_spec(spec)
spec.loader.exec_module(g)
CONFIG = json.loads((ROOT / "configs/a10/a10_wallx_geometry.json").read_text())


def feedback():
    return dict(ret_code=0, ret_context=dict(
        robot_msg=dict(tool=[0, 0], wobj=[0, 0], tool_mode=["Tool", "Tool"], cs_error_code=0),
        motion_msg=dict(pq=[[100., 200., 300., 0., 0., 0., 1.],
                            [400., 500., 600., 0., 0., 0., 1.]]),
        log_msg=dict(has_error=False), driver_msg=dict(driver_err_code=[0, 0])))


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.adapter = g.A10WallXGeometry(copy.deepcopy(CONFIG))

    def assertPoseEqual(self, actual, expected):
        np.testing.assert_allclose(g.pose_matrix(actual), g.pose_matrix(expected), atol=1e-12)

    def test_physical_hands_select_correct_feedback_rows(self):
        result = self.adapter.observation_geometry(feedback())["arms"]
        self.assertEqual(result["left_ee"]["ee_id"], 1)
        self.assertEqual(result["right_ee"]["ee_id"], 0)
        np.testing.assert_allclose(result["left_ee"]["position"], [.4, 1.075, .895], atol=1e-12)
        np.testing.assert_allclose(result["right_ee"]["position"], [-.1, -.775, .595], atol=1e-12)

    def test_rotated_tool_offset_applied_in_tool_frame(self):
        quat = Rotation.from_euler("y", 90, degrees=True).as_quat()
        pose = self.adapter.grasp_in_rig("left", [100., 0., 300., *quat], 0)
        # A +Z tool extension points along base +X after a 90-degree pitch.
        np.testing.assert_allclose(pose[:3], [.395, .575, .3], atol=1e-12)

    def test_active_tool1_is_not_extended_twice(self):
        pose = self.adapter.grasp_in_rig("left", [100., 0., 300., 0., 0., 0., 1.], 1)
        np.testing.assert_allclose(pose[:3], [.1, .575, .3], atol=1e-12)

    def test_nontrivial_pose_roundtrips_for_both_tools_and_arms(self):
        quat = Rotation.from_euler("xyz", [.7, -.4, 1.2]).as_quat()
        controller_mm = [175., -210., 650., *quat]
        for side in ("left", "right"):
            for tool in (0, 1):
                with self.subTest(side=side, tool=tool):
                    grasp = self.adapter.grasp_in_rig(side, controller_mm, tool)
                    controller = self.adapter.controller_pose_from_grasp(side, grasp, tool)
                    self.assertPoseEqual(controller, [.175, -.210, .650, *quat])

    def test_rpy_matches_documented_wallx_rotation_order(self):
        r, p, y = [.3, -.7, 1.1]
        actual = g.pose_matrix(g.rpy_pose([1., 2., 3.], [r, p, y]))[:3, :3]
        expected = (Rotation.from_euler("z", y).as_matrix()
                    @ Rotation.from_euler("y", p).as_matrix()
                    @ Rotation.from_euler("x", r).as_matrix())
        np.testing.assert_allclose(actual, expected, atol=1e-12)

    def test_rejects_unverified_feedback_tool_or_wobj(self):
        mutations = (
            lambda c: c["robot_msg"].update(tool=[2, 0]),
            lambda c: c["robot_msg"].update(wobj=[1, 0]),
            lambda c: c["robot_msg"].update(tool_mode=["Wobj", "Tool"]),
            lambda c: c["robot_msg"].update(cs_error_code=1),
            lambda c: c["log_msg"].update(has_error=True),
            lambda c: c["log_msg"].pop("has_error"),
            lambda c: c["motion_msg"]["pq"][0].__setitem__(3, float("nan")),
            lambda c: c["motion_msg"]["pq"][0].__setitem__(6, 0.5),
        )
        for mutate in mutations:
            reply = feedback()
            mutate(reply["ret_context"])
            with self.assertRaises(ValueError):
                self.adapter.observation_geometry(reply)

    def test_gripper_closure_uses_existing_calibrated_direction(self):
        self.assertEqual(self.adapter.closure_fraction(0), 0.)
        self.assertEqual(self.adapter.closure_fraction(4500), .5)
        self.assertEqual(self.adapter.closure_fraction(9000), 1.)
        for bad in (-1, 9001, float("nan"), True):
            with self.assertRaises(ValueError):
                self.adapter.closure_fraction(bad)

    def test_invalid_mount_spacing_or_duplicate_id_rejected(self):
        for mutate in (lambda c: c.update(base_spacing_m=2.),
                       lambda c: c["arms"]["left"].update(ee_id=0)):
            config = copy.deepcopy(CONFIG)
            mutate(config)
            with self.assertRaises(ValueError):
                g.A10WallXGeometry(config)


if __name__ == "__main__":
    unittest.main()
