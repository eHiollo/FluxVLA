"""A10 grasp geometry for Wall-X experiments; no hardware or model imports.

The rig frame here is explicitly defined in a10_wallx_geometry.json. It is
not assumed to equal any public Wall-X embodiment's training frame.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def vector(value, length, label):
    result = np.asarray(value)
    if result.shape != (length,) or result.dtype.kind not in "fiu" or not np.isfinite(result).all():
        raise ValueError(f"{label}: expected {length} finite numbers")
    return np.array(result, dtype=float, copy=True)


def pose_matrix(pose):
    """xyz in metres followed by quaternion xyzw, mapping child to parent."""
    pose = vector(pose, 7, "pose_m_xyzw")
    if not np.isclose(np.linalg.norm(pose[3:]), 1., atol=1e-3, rtol=0):
        raise ValueError("Quaternion xyzw must have unit length")
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(pose[3:]).as_matrix()
    result[:3, 3] = pose[:3]
    return result


def matrix_pose(matrix):
    return np.r_[matrix[:3, 3], Rotation.from_matrix(matrix[:3, :3]).as_quat()]


def rpy_pose(position, rpy):
    """Wall-X convention Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    xyz = vector(position, 3, "position_m")
    rotation = Rotation.from_euler("xyz", vector(rpy, 3, "rotation_rpy_rad"))
    return np.r_[xyz, rotation.as_quat()]


def public_pose(pose):
    matrix = pose_matrix(pose)
    return dict(position=matrix[:3, 3].tolist(),
                rotation_rpy=Rotation.from_matrix(matrix[:3, :3]).as_euler("xyz").tolist())


def feedback_context(reply):
    if "reply" in reply:
        reply = reply["reply"]
    if reply.get("ret_code") != 0:
        raise ValueError("A10 GET did not succeed")
    context = reply.get("ret_context")
    if isinstance(context, str):
        context = json.loads(context)
    if not isinstance(context, dict):
        raise ValueError("Missing A10 GET context")
    return context


class A10WallXGeometry:
    def __init__(self, config):
        if isinstance(config, (str, Path)):
            config = json.loads(Path(config).read_text())
        self.config = config
        if config.get("schema") != "a10_wallx_geometry_v1":
            raise ValueError("Unknown A10 Wall-X geometry schema")
        if set(config["arms"]) != {"left", "right"}:
            raise ValueError("Both physical arms are required")
        ids = [config["arms"][arm]["ee_id"] for arm in ("left", "right")]
        if sorted(ids) != [0, 1] or any(type(value) is not int for value in ids):
            raise ValueError("Physical arms must map to distinct ee_id 0 and 1")
        self.mounts = {}
        for side, arm in config["arms"].items():
            yaw = vector([arm["base_yaw_in_rig_rad"]], 1, "base_yaw")[0]
            self.mounts[side] = pose_matrix(rpy_pose(arm["base_in_rig_m"], [0., 0., yaw]))
        spacing = np.linalg.norm(self.mounts["left"][:3, 3] - self.mounts["right"][:3, 3])
        if not np.isclose(spacing, config["base_spacing_m"], atol=1e-6, rtol=0):
            raise ValueError("Configured base spacing disagrees with mount transforms")
        self.tool0_grasp = pose_matrix(np.r_[
            vector(config["grasp_point_in_tool0_m"], 3, "grasp offset"),
            vector(config["grasp_rotation_in_tool0_xyzw"], 4, "grasp quaternion")])
        self.gripper_open = float(config["gripper"]["open_raw"])
        self.gripper_closed = float(config["gripper"]["closed_raw"])
        if not np.isfinite([self.gripper_open, self.gripper_closed]).all() or self.gripper_closed <= self.gripper_open:
            raise ValueError("Expected increasing finite raw closure endpoints")

    def active_tool_to_grasp(self, tool):
        if type(tool) is not int:
            raise ValueError("Tool index must be an integer")
        if tool == 0:
            return self.tool0_grasp
        if tool == 1:
            # User confirmed tool1 equals the configured grasp point. Reading
            # tool1 feedback must not apply the 295 mm offset a second time.
            return np.eye(4)
        raise ValueError("Only calibrated tool0/tool1 are supported")

    def grasp_in_rig(self, side, feedback_pq_mm, active_tool):
        pose = vector(feedback_pq_mm, 7, "A10 TCP pq mm/xyzw")
        pose[:3] *= .001
        grasp = self.mounts[side] @ pose_matrix(pose) @ self.active_tool_to_grasp(active_tool)
        return matrix_pose(grasp)

    def controller_pose_from_grasp(self, side, grasp_pose_m_xyzw, active_tool):
        """Return base-frame metres/xyzw for the existing controller adapter.

        This is geometry only; it does not choose a follower mode or encode a
        command. Callers must first map the model frame to this rig frame.
        """
        target = (np.linalg.inv(self.mounts[side]) @ pose_matrix(grasp_pose_m_xyzw)
                  @ np.linalg.inv(self.active_tool_to_grasp(active_tool)))
        return matrix_pose(target)

    def closure_fraction(self, raw):
        value = vector([raw], 1, "measured gripper raw")[0]
        if not self.gripper_open <= value <= self.gripper_closed:
            raise ValueError("Measured gripper is outside calibrated endpoints")
        return float((value - self.gripper_open) / (self.gripper_closed - self.gripper_open))

    def observation_geometry(self, reply):
        context = feedback_context(reply)
        robot, motion = context["robot_msg"], context["motion_msg"]
        if context.get("log_msg", {}).get("has_error") is not False:
            raise ValueError("Missing or faulted A10 feedback")
        if robot.get("cs_error_code") != 0 or any(context.get("driver_msg", {}).get("driver_err_code", [])):
            raise ValueError("A10 feedback reports a controller/driver fault")
        tools, wobjs, poses = robot["tool"], robot["wobj"], motion["pq"]
        if len(tools) != 2 or len(wobjs) != 2 or len(poses) != 2:
            raise ValueError("Expected two complete TCP/tool/wobj groups")
        result = {}
        for side, arm in self.config["arms"].items():
            ee = arm["ee_id"]
            if type(wobjs[ee]) is not int or wobjs[ee] != 0:
                raise ValueError("A10 geometry requires verified base wobj0 feedback")
            tool_mode = robot.get("tool_mode")
            if tool_mode is None or len(tool_mode) != 2 or tool_mode[ee] != "Tool":
                raise ValueError("Handheld workpiece tool mode is not supported")
            grasp = self.grasp_in_rig(side, poses[ee], tools[ee])
            recovered = self.controller_pose_from_grasp(side, grasp, tools[ee])
            original = vector(poses[ee], 7, "TCP feedback")
            original[:3] *= .001
            rotation_error = (Rotation.from_quat(original[3:]).inv()
                              * Rotation.from_quat(recovered[3:])).magnitude()
            result[side + "_ee"] = dict(
                ee_id=ee, active_tool=tools[ee], active_wobj=wobjs[ee],
                controller_tcp_base_m_xyzw=original.tolist(),
                grasp_rig_m_xyzw=grasp.tolist(), **public_pose(grasp),
                height_above_table_m=float(grasp[2] - self.config["table"]["height_above_base_mount_m"]),
                roundtrip_position_error_m=float(np.linalg.norm(recovered[:3] - original[:3])),
                roundtrip_rotation_error_rad=float(rotation_error))
        return dict(frame="a10_rig", units=dict(position="m", rotation_rpy="rad"),
                    arms=result, table=self.config["table"],
                    model_contract_verified=self.config["model_contract"]["verified"],
                    robot_commands_sent=0)
