"""SDK Tool geometry, loopback protocol and VR lifecycle; no hardware access."""
import json
import unittest
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation as R

from fluxvla.engines.operators.a10_operator import SDKToolCumulativeAdapter
from fluxvla.engines.operators.vr.control import ReferenceConfig, ReferenceGovernor
from test_a10_operator import FakeController
from test_vr_reference_runtime import Clock, StoppableA10, runtime, sample

IDENTITY = np.array([0., 0., 0., 0., 0., 0., 1.])
LIMITS = dict(max_position_step_m=.005, max_rotation_step_rad=.015)


def encode(adapter, target):
    pose, pe = adapter.propose(target, **LIMITS)
    adapter.commit(pose, pe)
    return pe


def sdk_integrate(start, packets, subdivisions=20):
    """Independent Tool branch: component difference -> 321 -> right multiply.

    Equal substeps approximate geometry only, not SDK filters/moveAbsolute2.
    """
    position, rotation = np.array(start[:3]), R.from_quat(start[3:])
    previous = np.zeros(6)
    for packet in packets:
        delta = (packet - previous) / subdivisions
        for _ in range(subdivisions):
            position += rotation.apply(delta[:3])
            rotation = rotation * R.from_euler('ZYX', delta[3:])
        previous = packet
    return np.r_[position, rotation.as_quat()]


def error(a, b):
    return (np.linalg.norm(a[:3] - b[:3]),
            (R.from_quat(a[3:]).inv() * R.from_quat(b[3:])).magnitude())


class GeometryTest(unittest.TestCase):
    def test_straight_path_is_not_double_accumulated_and_repeat_holds(self):
        adapter = SDKToolCumulativeAdapter(IDENTITY)
        for x in np.linspace(0, -.30, 301):
            target = IDENTITY.copy(); target[0] = x
            packet = encode(adapter, target)
            np.testing.assert_allclose(packet, [x, 0, 0, 0, 0, 0], atol=1e-12)
        for _ in range(20):
            np.testing.assert_allclose(encode(adapter, target), packet, atol=1e-12)

    def test_turn_then_translate_in_fixed_base_direction(self):
        adapter = SDKToolCumulativeAdapter(IDENTITY); packets = []
        for yaw in np.linspace(0, np.pi/2, 201):
            target = np.r_[np.zeros(3), R.from_euler('Z', yaw).as_quat()]
            packets.append(encode(adapter, target))
        for x in np.linspace(0, .10, 101):
            target = np.r_[x, 0, 0, R.from_euler('Z', np.pi/2).as_quat()]
            packets.append(encode(adapter, target))
        np.testing.assert_allclose(packets[-1], [0, -.1, 0, np.pi/2, 0, 0], atol=1e-12)
        np.testing.assert_allclose(sdk_integrate(IDENTITY, packets), target, atol=1e-12)

    def test_combined_rotation_translation_beats_legacy_geometric_path(self):
        adapter = SDKToolCumulativeAdapter(IDENTITY); packets = []; legacy = []
        for t in np.linspace(0, 1, 401):
            rotation = R.from_euler('ZYX', [t*.8, t*.6, -t*.4])
            target = np.r_[.12*t, .08*t*t, .02*np.sin(t*3), rotation.as_quat()]
            packets.append(encode(adapter, target))
            legacy.append(np.r_[target[:3], rotation.as_euler('ZYX')])
        old = error(sdk_integrate(IDENTITY, legacy), target)
        new = error(sdk_integrate(IDENTITY, packets), target)
        self.assertGreater(old[0], .01); self.assertGreater(old[1], .1)
        self.assertLess(new[0], .001); self.assertLess(new[1], .002)

    def test_delayed_component_planner_with_fixed_target_settling(self):
        # Assumed 80 ms first-order filtering + component slew at 500 Hz.
        # This is a stress model, not a reconstruction of native moveAbsolute2.
        def delayed(packets):
            filtered = np.zeros(6); planned = np.zeros(6)
            position = np.zeros(3); rotation = R.identity()
            trajectory = list(packets) + [packets[-1]] * 90
            for packet in trajectory:
                for _ in range(17):
                    filtered += (1-np.exp(-.002/.08)) * (packet-filtered)
                    step = np.clip(filtered-planned, -np.array([.1]*3+[.262]*3)*.002,
                                   np.array([.1]*3+[.262]*3)*.002)
                    planned += step
                    position += rotation.apply(step[:3])
                    rotation = rotation * R.from_euler('ZYX', step[3:])
            return np.r_[position, rotation.as_quat()]
        adapter = SDKToolCumulativeAdapter(IDENTITY); packets=[]; legacy=[]
        for t in np.linspace(0, 1, 201):
            rotation = R.from_euler('ZYX', [.8*t, .6*t, -.4*t])
            target = np.r_[.12*t, .08*t*t, 0, rotation.as_quat()]
            packets.append(encode(adapter,target))
            legacy.append(np.r_[target[:3],rotation.as_euler('ZYX')])
        old=error(delayed(legacy),target); new=error(delayed(packets),target)
        self.assertLess(new[0], old[0]/5)
        self.assertLess(new[1], old[1]/5)
        self.assertGreater(new[1], 1e-6)  # Does not promise exact execution.

    def test_invalid_or_large_step_does_not_advance_state(self):
        adapter = SDKToolCumulativeAdapter(IDENTITY)
        for target in ([1., 0, 0, 0, 0, 0, 1], [float('nan'), 0, 0, 0, 0, 0, 1]):
            with self.assertRaises(ValueError): adapter.propose(target, **LIMITS)
        np.testing.assert_array_equal(adapter.previous_pose, IDENTITY)
        np.testing.assert_array_equal(adapter.cumulative_pe, np.zeros(6))

    def test_delayed_downstream_does_not_cancel_lead_or_fault_guards(self):
        cfg = ReferenceConfig(max_reference_lead_m=.02, max_reference_lead_rad=.12)
        governor = ReferenceGovernor(cfg); governor.engage(IDENTITY)
        adapter = SDKToolCumulativeAdapter(IDENTITY)
        goal = IDENTITY.copy(); goal[0] = .2
        # Downstream stalls: no feedback progress, not an invitation to add error.
        for _ in range(200):
            governor.step(goal, IDENTITY, 1/30)
            encode(adapter, governor.reference)
            self.assertLessEqual(governor.reference[0], .02000001)
        before = adapter.cumulative_pe.copy()
        for _ in range(20): encode(adapter, governor.reference)
        np.testing.assert_allclose(adapter.cumulative_pe, before, atol=1e-12)
        bad_feedback = IDENTITY.copy(); bad_feedback[0] = -.1
        for _ in range(4): state = governor.step(goal, bad_feedback, 1/30)
        self.assertEqual(state, 'fault')


class ProtocolTest(unittest.TestCase):
    def test_loopback_wire_lifecycle_and_mixing_protection(self):
        with FakeController() as controller:
            robot = controller.operator(cartesian_wire_format='pe321', feedback_position_unit='mm')
            with self.assertRaises(RuntimeError): robot.begin_sdk_tool_trajectory(IDENTITY)
            robot.start_follower(); robot.begin_sdk_tool_trajectory(IDENTITY)
            target = IDENTITY.copy(); target[0] = .001
            robot.send_sdk_tool_pose(target, **LIMITS)
            packet = controller.udp.recv(4096)
            self.assertEqual(int.from_bytes(packet[:4], 'little'), len(packet)-40)
            self.assertEqual(packet[4:40], bytes(36))
            self.assertEqual(json.loads(packet[40:]), {'type':'321', 'pe':[[.001,0,0,0,0,0]]})
            with self.assertRaises(RuntimeError): robot.send_eepose({'right':IDENTITY})
            with self.assertRaises(RuntimeError): robot.begin_sdk_tool_trajectory(IDENTITY)
            robot.stop_follower()
            with self.assertRaises(RuntimeError): robot.send_sdk_tool_pose(target, **LIMITS)
            robot.start_follower(); robot.begin_sdk_tool_trajectory(target)
            robot.send_sdk_tool_pose(target, **LIMITS)
            np.testing.assert_allclose(json.loads(controller.udp.recv(4096)[40:])['pe'], [np.zeros(6)])
            robot.stop_follower(); robot.start_follower()
            robot.send_eepose({'right':IDENTITY}); controller.udp.recv(4096)
            with self.assertRaises(RuntimeError): robot.begin_sdk_tool_trajectory(IDENTITY)
            robot.stop_follower()

    def test_send_failure_does_not_commit_and_forbids_retry(self):
        with FakeController() as controller:
            robot = controller.operator(cartesian_wire_format='pe321')
            robot.start_follower(); robot.begin_sdk_tool_trajectory(IDENTITY)
            adapter = robot._sdk_tool_adapter
            with patch.object(robot, '_send_follower_target', side_effect=OSError('lost')):
                with self.assertRaises(OSError):
                    robot.send_sdk_tool_pose(np.r_[.001,0,0,0,0,0,1], **LIMITS)
            np.testing.assert_array_equal(adapter.cumulative_pe, np.zeros(6))
            with self.assertRaises(RuntimeError): robot.send_sdk_tool_pose(IDENTITY, **LIMITS)
            self.assertFalse(robot._confirmed_active)
            robot.stop_follower()


class AdapterRobot(StoppableA10):
    cartesian_wire_format = 'pe321'
    def begin_sdk_tool_trajectory(self, pose):
        self.events.append('begin_adapter'); self.adapter = SDKToolCumulativeAdapter(pose)
    def send_sdk_tool_pose(self, target, **limits):
        pose, pe = self.adapter.propose(target, **limits)
        self.adapter.commit(pose, pe); self.targets.append(pe.copy())
        return dict(cumulative_pe321=pe.tolist(), execution_confirmed=False)


class RuntimeTest(unittest.TestCase):
    def test_new_path_both_mappings_release_and_reengage(self):
        for mapping in ('anchor_relative', 'calibrated_absolute'):
            rows = [sample(1),sample(2),sample(3,offset=.01),sample(4,active=False),sample(5),sample(6)]
            for row in rows:
                row['mapped']['_xlevr.anchor_command'].update(hand_pose_vr=IDENTITY.tolist(), position_scale=1.)
            kwargs = dict(cartesian_adapter='sdk_tool_cumulative',pose_mapping=mapping)
            if mapping == 'calibrated_absolute':
                kwargs['pose_calibration'] = dict(vr_to_robot_quat_xyzw=[0,0,0,1],hand_to_tool_quat_xyzw=[0,0,0,1])
            rt, robot = runtime(rows, robot=AdapterRobot(), **kwargs); clock=Clock()
            with patch('time.monotonic', clock.now):
                rt.connect()
                for row in rows: clock.advance(); rt.step()
                self.assertEqual(robot.events.count('begin_adapter'), 2)
                self.assertIsNone(rt.last_relative_target)
                self.assertIsNone(rt.last_reference_result['relative_target'])
                rt.close()

    def test_config_rejects_unsupported_modes_before_connect(self):
        from mmengine import Config
        from fluxvla.engines.operators.vr.runtime import build_runtime
        cfg=Config.fromfile(Path(__file__).resolve().parents[1]/'configs/a10/a10_vr_teleop.py')
        cfg.teleop.cartesian_adapter='sdk_tool_cumulative'
        with self.assertRaisesRegex(ValueError,'reference'): build_runtime(cfg.teleop)
        cfg.teleop.control_mode='reference'
        self.assertEqual(build_runtime(cfg.teleop).cartesian_adapter,'sdk_tool_cumulative')
        cfg.teleop.cartesian_adapter='unknown'
        with self.assertRaisesRegex(ValueError,'cartesian_adapter'): build_runtime(cfg.teleop)

    def test_observe_never_initializes_or_sends_adapter(self):
        rt, robot = runtime([sample(1),sample(2)], observe=True, robot=AdapterRobot(), cartesian_adapter='sdk_tool_cumulative')
        clock=Clock()
        with patch('time.monotonic', clock.now):
            rt.connect()
            for _ in range(2): clock.advance(); rt.step()
            rt.close()
        self.assertNotIn('begin_adapter', robot.events); self.assertFalse(robot.targets)


def replay_log(path):
    """Replay latest executed absolute-mapping session, keeping old commands fixed."""
    rows = []; current = []
    with open(path) as stream:
        for line in stream:
            row = json.loads(line)
            if 'control_frame_index' not in row:
                continue
            if row['control_frame_index'] == 0:
                if current and current[-1].get('pose_mapping') == 'calibrated_absolute' and current[-1].get('execute') and current[-1].get('cartesian_adapter', 'legacy_anchor') == 'legacy_anchor':
                    rows = current
                current = []
            current.append(row)
    if current and current[-1].get('pose_mapping') == 'calibrated_absolute' and current[-1].get('execute') and current[-1].get('cartesian_adapter', 'legacy_anchor') == 'legacy_anchor':
        rows = current
    report = []; start = None
    for row in rows:
        result = row.get('reference_control') or {}
        if result.get('state') == 'await_fresh_sample':
            start = np.asarray(row['feedback']['tcp_pq_m'])
            adapter = SDKToolCumulativeAdapter(start); old_packets=[]; new_packets=[]
            previous_euler=np.zeros(3); first=row['control_frame_index']; target=None
        elif start is not None and result.get('state') == 'tracking':
            target=np.asarray(result['command_pose'])
            new_packets.append(encode(adapter, target))
            relative=np.asarray(row['sent_relative_target'])
            euler=R.from_quat(relative[3:]).as_euler('ZYX')
            previous_euler += (euler-previous_euler+np.pi)%(2*np.pi)-np.pi
            old_packets.append(np.r_[relative[:3],previous_euler])
        elif start is not None and result.get('state') in ('inactive','fault'):
            if target is not None:
                old=error(sdk_integrate(start,old_packets),target)
                new=error(sdk_integrate(start,new_packets),target)
                report.append(dict(first_frame=first,last_frame=row['control_frame_index'],
                                   old_position_mm=old[0]*1000,old_angle_deg=np.degrees(old[1]),
                                   new_position_mm=new[0]*1000,new_angle_deg=np.degrees(new[1])))
            start=None
    if not report:
        raise ValueError('No completed executed calibrated_absolute segments found')
    return dict(method='fixed historical commands; geometric substeps only, no SDK filter/dynamics or closed-loop rerun',
                session_id=rows[-1].get('anchor_shadow_session_id'),segments=report)


if __name__ == '__main__':
    import argparse
    from pathlib import Path
    parser=argparse.ArgumentParser()
    parser.add_argument('--replay-log',type=Path)
    parser.add_argument('--report',type=Path)
    args=parser.parse_args()
    if args.replay_log:
        result=replay_log(args.replay_log)
        output=json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n'
        if args.report:
            args.report.parent.mkdir(parents=True,exist_ok=True);args.report.write_text(output)
        print(output)
    else:
        unittest.main(argv=['test_a10_sdk_adapter'])
