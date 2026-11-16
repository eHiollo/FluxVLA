"""Final command-only VR profile: synthetic robot and real loopback wire tests."""
from dataclasses import replace
import json
from pathlib import Path
import select
import threading
import unittest
from unittest.mock import patch
import numpy as np
from mmengine import Config
import test_vr_dual_runtime as dual
from test_vr_dual_runtime import HandInput, hand
from test_vr_reference_runtime import Clock
from test_a10_operator import FakeController, dual_motion
from fluxvla.engines.communication.a10_shared_server import SharedControl, SharedControlServer
from fluxvla.engines.operators.vr.runtime import build_runtime
from fluxvla.engines.operators.a10_operator import SDKToolCumulativeAdapter

class CommandOnlyTest(unittest.TestCase):
    robot_type = dual.DualSDKRobot
    cartesian_adapter = 'sdk_tool_cumulative'
    tick = dual.DualRuntimeTest.tick
    move = dual.DualRuntimeTest.move

    def setUp(self):
        dual.DualRuntimeTest.setUp(self)
        self.runtime.continuous_feedback = False
        self.runtime.confirm_stop = False
        self.robot.follow = False  # Actual feedback stays distinct from virtual references.
        cfg = replace(self.runtime.reference_control.config, enforce_tracking_error_limits=False,
                      max_reference_lead_m=None, max_reference_lead_rad=None)
        self.runtime.reference_control.config = cfg
        for context in self.runtime._arms.values():
            context['control'].config = cfg
            context['control'].governor.config = cfg
        def stop():
            self.robot.stop_follower()
            return dict(requested=True, confirmed=False)
        self.robot.request_stop_follower = stop
        self.runtime.input_recovery_stable_s = .1

    def test_idle_never_queries_measured_feedback(self):
        with patch.object(self.robot, 'get_frame', side_effect=AssertionError('idle GET')):
            for _ in range(10): self.tick(False, False)
        self.assertEqual(self.robot.targets, [])
        self.assertEqual(self.runtime.last_reference_result['state'], 'idle')

    def test_running_beyond_feedback_timeout_never_queries_and_keeps_actual_frame_time(self):
        self.move(4)
        actual_stamp = self.runtime.last_frame['received_monotonic_s']
        count = len(self.robot.sdk_packets)
        with patch.object(self.robot, 'get_frame', side_effect=AssertionError('control GET')):
            for i in range(90): self.tick(True, True, .001 + i*.0001, .001)
        self.assertGreater(len(self.robot.sdk_packets), count + 80)
        self.assertEqual(self.runtime.last_frame['received_monotonic_s'], actual_stamp)
        self.assertEqual(self.runtime.last_reference_result['reference_source'], 'last_sent_target')
        self.assertEqual(self.runtime.last_reference_result['reference_origin'], 'local_zero')
        self.assertTrue(self.vr.is_connected)
        self.assertIsNone(self.runtime.communication_pause)

    def test_initial_packet_is_zero_and_origin_does_not_use_actual_position(self):
        actual_positions = {name: value[:3].copy() for name, value in self.robot.poses.items()}
        self.tick(); self.tick()
        self.assertEqual(len(self.robot.sdk_packets), 1)
        for name in self.robot.arm_names:
            np.testing.assert_array_equal(self.robot.sdk_packets[0][name], np.zeros(6))
            np.testing.assert_array_equal(self.runtime._arms[name]['control'].startup_anchor[:3], np.zeros(3))
            np.testing.assert_array_equal(self.robot.poses[name][:3], actual_positions[name])

    def test_one_arm_release_and_reclutch_do_not_query_or_restart_whole_follower(self):
        self.move(5)
        with patch.object(self.robot, 'get_frame', side_effect=AssertionError('reclutch GET')):
            for _ in range(12): self.tick(False, True)
            for _ in range(10): self.tick(True, True)
        self.assertEqual(self.robot.events.count('start'), 1)
        self.assertTrue(all(set(packet) == set(self.robot.arm_names) for packet in self.robot.sdk_packets))

    def test_both_release_stops_without_query_and_next_takeover_reads_fresh_state(self):
        self.move(4)
        sent = len(self.robot.sdk_packets)
        with patch.object(self.robot, 'get_frame', side_effect=AssertionError('stop/idle GET')):
            self.tick(False, False)
            for _ in range(3): self.tick(False, False)
        self.assertEqual(self.robot.events.count('stop'), 1)
        self.assertNotIn('wait_stopped', self.robot.events)
        self.assertEqual(len(self.robot.sdk_packets), sent)
        with patch.object(self.robot, 'get_frame', wraps=self.robot.get_frame) as query:
            for _ in range(4): self.tick(True, False)
            self.assertGreaterEqual(query.call_count, 1)
        self.assertEqual(self.robot.events.count('start'), 2)

    def test_lost_vr_stops_and_stability_gate_does_not_query_feedback(self):
        self.move(4)
        sent = len(self.robot.sdk_packets)
        for sample in self.vr.input.rows.values(): sample['xlevr.sample_age_s'] = 2.
        with patch.object(self.robot, 'get_frame', side_effect=AssertionError('pause GET')):
            self.clock.advance()
            self.runtime.step()
            self.assertIsNotNone(self.runtime.input_pause_status)
            for _ in range(12): self.tick(False, False)
            self.assertEqual(self.runtime.input_pause_status['state'], 'ready')
            self.tick(True, False)
        self.assertEqual(len(self.robot.sdk_packets), sent)
        self.assertEqual(self.robot.events.count('stop'), 1)
        self.assertEqual(self.robot.events.count('start'), 1)

    def test_failed_send_cannot_commit_a_new_local_reference(self):
        self.move(4)
        self.robot.fail_send = True
        with self.assertRaisesRegex(OSError, 'UDP'):
            self.tick(True, False, .002)
        self.assertIsNone(self.runtime._sent_reference_poses)
        self.assertIsNone(self.runtime._last_target_send_s)

    def test_session_holding_stays_quiet_and_fault_still_requests_stop(self):
        self.runtime.stop_on_both_release = False
        self.move(12)
        for _ in range(25): self.tick(False, False)
        count = len(self.robot.sdk_packets)
        self.assertNotIn('stop', self.robot.events)
        with patch.object(self.robot, 'get_frame', side_effect=AssertionError('re-grip GET')):
            for _ in range(90): self.tick(False, False)
            self.assertEqual(len(self.robot.sdk_packets), count)
            for _ in range(5): self.tick(True, False)
        self.assertEqual(self.robot.events.count('start'), 1)
        self.assertIsNone(self.runtime.communication_pause)
        for row in self.vr.input.rows.values(): row['xlevr.sample_age_s'] = 2.
        self.clock.advance()
        self.runtime.step()
        self.assertEqual(self.robot.events.count('stop'), 1)
        self.assertIsNotNone(self.runtime.input_pause_status)

    def test_sdk_increment_is_invariant_to_absolute_position_origin(self):
        q = self.robot.poses['left'][3:]
        delta = np.array([.002, -.001, .003])
        rows = []
        for origin in ([0., 0., 0.], [10., -20., 30.]):
            initial = np.r_[origin, q]
            adapter = SDKToolCumulativeAdapter(initial)
            _, command = adapter.propose(np.r_[np.asarray(origin)+delta, q],
                                         max_position_step_m=None, max_rotation_step_rad=None)
            rows.append(command)
        np.testing.assert_allclose(rows[0], rows[1], atol=1e-14)


class RecordingSubscriptionTest(unittest.TestCase):
    def test_only_recording_subscribes_to_background_get(self):
        from fluxvla.engines.operators.vr_operator import VROperator
        from fluxvla.engines.operators.vr.runtime import DualVRTeleopRuntime
        for recording in (False, True):
            with self.subTest(recording=recording):
                cfg = dual.configuration(identity_bindings=True)
                vr = VROperator(**{k: v for k, v in cfg.input.items() if k != 'type'})
                vr.input = dual.HandInput()
                robot = dual.DualSDKRobot()
                robot.start_feedback_polling = lambda hz: robot.events.append(('poll', hz))
                runtime = DualVRTeleopRuntime(vr, robot, execute=True, control_mode='reference',
                    cartesian_adapter='sdk_tool_cumulative', continuous_feedback=False,
                    feedback_poll_hz=30., reference_options=dict(enforce_tracking_error_limits=False))
                if recording: runtime.recording = object()
                try:
                    runtime.connect()
                    self.assertEqual(('poll', 30.) in robot.events, recording)
                    self.assertFalse(runtime._feedback_polling)
                finally:
                    runtime.close()



ROOT = Path(__file__).resolve().parents[1]


class LiveProfileLoopbackTest(unittest.TestCase):
    def test_live_profile_connect_does_not_rewrite_existing_70_percent(self):
        with FakeController() as native:
            native.motion_override = dual_motion()
            native.jog_vel = 70
            native.active, native.mode = True, 'Cart'
            cfg = Config.fromfile(str(ROOT/'configs/a10/a10_dual_arm_vr_teleop_world_translation.py')).teleop
            cfg.zx_grippers.enabled = False
            cfg.diagnostics_path = None
            cfg.operator.update(host='127.0.0.1', ws_port=native.server.socket.getsockname()[1],
                status_ws_port=native.server.socket.getsockname()[1], udp_port=native.udp.getsockname()[1],
                joint_udp_port=native.joint_udp.getsockname()[1], shared_control=None, timeout_s=1.)
            runtime = build_runtime(cfg)
            runtime.teleoperator.input = runtime.vr = HandInput()
            try:
                runtime.connect()
                self.assertTrue(runtime._connected)
                self.assertTrue(runtime.speed_status['already_set'])
                self.assertTrue(all(c == 'get' for c in native.commands))
            finally:
                runtime.close()
            self.assertTrue(all(c == 'get' for c in native.commands))

    def test_live_profile_buttons_row_order_no_continuous_get_and_reclutch(self):
        with FakeController() as native:
            native.motion_override = dual_motion()
            ws_port = native.server.socket.getsockname()[1]
            relay = SharedControl('127.0.0.1', ws_port, ws_port,
                native.udp.getsockname()[1], native.joint_udp.getsockname()[1], timeout_s=1.)
            relay.connect()
            server = SharedControlServer(('127.0.0.1', 0), relay)
            worker = threading.Thread(target=server.serve_forever, kwargs=dict(poll_interval=.01), daemon=True)
            worker.start()
            runtime = None
            try:
                cfg = Config.fromfile(str(ROOT/'configs/a10/a10_dual_arm_vr_teleop_world_translation.py')).teleop
                cfg.zx_grippers.enabled = False  # No serial hardware in a loopback test.
                cfg.diagnostics_path = None
                cfg.operator.update(host='127.0.0.1', ws_port=ws_port, status_ws_port=ws_port,
                    udp_port=native.udp.getsockname()[1], joint_udp_port=native.joint_udp.getsockname()[1], timeout_s=1.)
                cfg.operator.shared_control.update(mode='shared', host='127.0.0.1', port=server.server_address[1])
                runtime = build_runtime(cfg)
                source = HandInput()
                runtime.teleoperator.input = runtime.vr = source
                self.assertTrue(runtime.execute)
                self.assertFalse(runtime.continuous_feedback)
                self.assertFalse(runtime.confirm_stop)
                self.assertEqual(runtime.teleoperator.hand_to_arm, {'left': 'right', 'right': 'left'})
                self.assertEqual(tuple(runtime.robot.cartesian_target_order), ('left', 'right'))
                clock = Clock(); clock.value = 100.
                sequence = 0
                packets = []
                def tick(*, grip=False, trigger=0., x=0., angle=0.):
                    nonlocal sequence
                    clock.value += 1./90.
                    sequence += 1
                    left = hand(sequence, grip, x, angle=angle, trigger=trigger)
                    left['xlevr.enabled'] = grip or trigger >= .5
                    source.rows = {'left': left, 'right': hand(sequence, False)}
                    runtime.step()
                    if runtime.last_reference_result.get('transmitted'):
                        packet = native.udp.recv(65535)
                        payload = json.loads(packet[40:])
                        self.assertEqual(payload['type'], '321')
                        self.assertEqual(np.shape(payload['pe']), (2, 6))
                        packets.append(np.asarray(payload['pe']))
                        return packets[-1]
                    self.assertFalse(select.select([native.udp], [], [], 0)[0], 'Unexpected target while idle/stopped')
                    return None
                with patch('time.monotonic', clock.now):
                    runtime.connect()
                    self.assertEqual(native.jog_vel, 70.)
                    self.assertFalse(relay._poll_rates)
                    gets = native.commands.count('get')
                    for _ in range(3): tick()
                    self.assertEqual(native.commands.count('get'), gets)
                    for i in range(1, 25): tick(grip=True, x=.0003*i, angle=.02*i)
                    np.testing.assert_allclose(packets[0], np.zeros((2, 6)), atol=1e-12)
                    np.testing.assert_allclose(np.asarray(packets)[:, 0, :], 0., atol=1e-12)
                    np.testing.assert_allclose(np.asarray(packets)[:, 1, 3:], 0., atol=1e-12)
                    self.assertGreater(np.linalg.norm(packets[-1][1, :3]), .0001)
                    gets = native.commands.count('get')
                    switch = tick(trigger=1., x=.0072, angle=.7)
                    np.testing.assert_allclose(switch[1, 3:], 0., atol=1e-12)
                    for i in range(1, 25): tick(trigger=1., x=.0072+.0001*i, angle=.7+.015*i)
                    self.assertEqual(native.commands.count('get'), gets, 'Continuous GET in the control path')
                    self.assertGreater(np.linalg.norm(packets[-1][1, 3:]), .001)
                    np.testing.assert_allclose(np.asarray(packets)[:, 0, :], 0., atol=1e-12)
                    self.assertFalse(runtime._arms['right']['control'].fault_reason)
                    for _ in range(60): tick()
                    self.assertEqual(native.commands.count('get'), gets, 'Normal release queried the robot')
                    self.assertTrue(runtime.robot.follower_stop_pending)
                    self.assertIsNotNone(relay.owner)
                    self.assertNotIn('stop_follower', native.commands)
                    held = packets[-1].copy()
                    count = len(packets)
                    for _ in range(90): tick()
                    self.assertEqual(len(packets), count, 'Repeated old HOLD targets')
                    self.assertIsNone(runtime.communication_pause)
                    rearmed = None
                    for _ in range(4):
                        row = tick(trigger=1., x=.0096, angle=1.1)
                        if row is not None and rearmed is None: rearmed = row
                    self.assertIsNotNone(rearmed)
                    np.testing.assert_allclose(rearmed, held, atol=1e-12)
                    self.assertEqual(native.commands.count('follower_cart'), 1)
                    self.assertEqual(native.commands.count('get'), gets)
                    self.assertNotIn('set_jog_coordinate --tool', native.commands)
                    runtime.close()
                    runtime = None
            finally:
                if runtime is not None: runtime.close()
                server.shutdown(); worker.join(2.); server.server_close()
                relay.close()
