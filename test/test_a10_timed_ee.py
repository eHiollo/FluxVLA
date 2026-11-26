"""Offline TimedEe protocol and lifecycle checks; no robot connections."""
import json
import math
import struct
import unittest
from unittest.mock import Mock, patch

import numpy as np

from fluxvla.engines.communication.a10_timed_ee import (
    TimedEeTransport, encode_target, endpoint_reached, require_stream,
    timed_config, validate_status)


POSES = [[.3, -.1, .4, 0., 0., 0., 1.], [.3, .1, .4, 0., 0., 0., 1.]]


def tee(**changes):
    result = dict(ret_code=0, implementation='joint_tracking.TimedEe', input_source='udp',
                  control_space='ee', tracking_profile='capture_fast',
                  pose_convention='xyz_m_quaternion_xyzw', frame='wobj0', tool='tool1',
                  path_policy='per_target_ik_then_capture_fast_joint_tracking',
                  cartesian_line_guaranteed=False, ee_pose_source='command_fk',
                  ee_endpoint_policy='exact_ik_goal', gap_policy='hold_and_resume',
                  parameters=dict(endpoint_tolerance_deg=0),
                  endpoint_position_tolerance_m=.001,
                  endpoint_orientation_tolerance_rad=math.radians(.1),
                  stream_id=8, active=True, ready=True, accepting=True,
                  stop_requested=False, waiting_for_input=False, hold_transition=False,
                  end_received=False, final_reference_reached=False,
                  received=0, consumed=0, queue_depth=0, state=2, reason=0,
                  command_pose_m_quat=[p.copy() for p in POSES],
                  command_joint_rad=[0.] * 12, command_joint_velocity_rad_s=[0.] * 12,
                  joint_goal_rad=[0.] * 12, endpoint_position_error_m=[0., 0.],
                  endpoint_orientation_error_rad=[0., 0.])
    result.update(changes)
    return result


def complete(**changes):
    return tee(**dict(dict(received=2, consumed=2, end_received=True,
                           final_reference_reached=True, accepting=False, state=3), **changes))


def fast_complete(**changes):
    """Current upstream fast-v2, with a valid nonzero IK goal residual."""
    return complete(**dict(dict(ee_endpoint_policy='bounded_cartesian_capture',
        parameters=dict(endpoint_tolerance_deg=.3), endpoint_capture_complete=True,
        endpoint_position_tolerance_m=.002, endpoint_orientation_tolerance_rad=math.radians(.2),
        command_joint_rad=[math.radians(.2)] * 12,
        endpoint_position_error_m=[.0015, .0018],
        endpoint_orientation_error_rad=[math.radians(.15), math.radians(.18)]), **changes))


class Clock:
    def __init__(self):
        self.now = 0.

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


class ProtocolTests(unittest.TestCase):
    def transport(self, timeout=.2):
        t = TimedEeTransport(Mock(), Mock(), '127.0.0.1', 5999, 5999, timeout)
        t.stream_id, t.udp, t.last_status = 8, Mock(), tee()
        t.udp.send.side_effect = len
        return t

    def test_wire_absolute_m_xyzw_physical_order_no_aris_or_gripper(self):
        body = encode_target(8, 3, POSES, final=True)
        self.assertNotIn(b'\0', body)
        self.assertEqual(json.loads(body), dict(stream_id=8, index=3, pose_m_quat=POSES,
                                               frame='wobj0', tool='tool1', final=True))
        # Quaternion signs are preserved; q and -q do not encode a turn.
        negative = np.asarray(POSES)
        negative[0, 3:] *= -1
        self.assertEqual(json.loads(encode_target(8, 0, negative))['pose_m_quat'][0][6], -1.)

    def test_wire_rejects_single_arm_euler_gripper_invalid_quaternion(self):
        for poses in (POSES[0], [[0.] * 6] * 2, [[0.] * 8] * 2,
                      [[0.] * 7] * 2, [['0'] * 7] * 2, [[False] * 7] * 2,
                      [[float('nan')] * 7] * 2,
                      [POSES[0][:-1] + [1.01], POSES[1]]):
            with self.subTest(poses=poses), self.assertRaises(ValueError):
                encode_target(8, 0, poses)
        for sid, index, final in ((0, 0, False), (True, 0, False), (2**64, 0, False),
                                  (8, -1, False), (8, True, False), (8, 0, 1)):
            with self.assertRaises(ValueError):
                encode_target(sid, index, POSES, final)

    def test_status_cannot_be_joint_selected_tool_or_old_early_capture(self):
        validate_status(tee())
        for changes in (dict(implementation='joint_tracking.TimedAj'), dict(tool='tool0'),
                        dict(frame='rig'), dict(ee_pose_source='actual_feedback'),
                        dict(parameters=dict(endpoint_tolerance_deg=.5)),
                        dict(cartesian_line_guaranteed=True), dict(reason=12),
                        dict(active=1), dict(stream_id=True), dict(tracking_profile='legacy')):
            with self.subTest(changes=changes), self.assertRaises((RuntimeError, ValueError)):
                validate_status(tee(**changes))
        with self.assertRaises((RuntimeError, ValueError)):
            validate_status(dict(ret_code=0, ret_context=tee()))
        for reason in (10, 11):
            validate_status(tee(reason=reason))
            with self.assertRaises(RuntimeError):
                require_stream(tee(reason=reason), 8)

    def test_configuration_has_no_tuning_override(self):
        self.assertEqual(timed_config(None)['udp_port'], 10000)
        for options in (dict(omega=25), dict(udp_port=True), dict(network_ready=1)):
            with self.assertRaises(ValueError):
                timed_config(options)
        t = self.transport()
        with self.assertRaisesRegex(RuntimeError, 'network'):
            t.start()
        with self.assertRaisesRegex(ValueError, 'dedicated'):
            t.start(dict(network_ready=True, plan_ws_port=5999))

    def test_inactive_sample_history_can_be_read_but_not_used_for_udp_sending(self):
        history = tee(active=False, ready=False, accepting=False, state=5,
                      input_source='sample30_ee_fk_ik')
        validate_status(history)
        with self.assertRaisesRegex(RuntimeError, 'input_source'):
            validate_status(dict(history, active=True))
        with self.assertRaises(RuntimeError):
            require_stream(history, 8, accepting=True)
        t = self.transport(); t.last_status = history
        with self.assertRaises(RuntimeError):
            t.send(POSES)
        t.udp.send.assert_not_called()

    def test_start_reuses_commands_allocated_id_and_dedicated_long_plan_socket(self):
        t = TimedEeTransport(Mock(), Mock(), '127.0.0.1', 5999, 5999, .2)
        t.command.side_effect = [tee(stream_id=4, active=False),
                                tee(stream_id=4, active=False), tee(stream_id=9)]
        t.robot_state.return_value = dict(ret_context=dict(robot_msg=dict(
            op_mode='Manual', activate='Enabled', status='Normal', motion='Stopped')))
        plan, udp = Mock(), Mock()
        with patch('websockets.sync.client.connect', return_value=plan) as connect, \
                patch('fluxvla.engines.communication.a10_timed_ee.socket.socket', return_value=udp), \
                patch('fluxvla.engines.communication.a10_timed_ee.threading.Thread'):
            result = t.start(dict(network_ready=True))
        self.assertEqual(result['stream_id'], 9)  # Not guessed previous_id+1.
        self.assertEqual(result['command_pose_m_quat'], POSES)
        self.assertEqual(t.stream_id, 9)
        self.assertTrue(t.start_attempted)
        self.assertEqual(connect.call_args.args[0], 'ws://127.0.0.1:5888')
        self.assertIn(b'timed_ee', plan.send.call_args.args[0])
        payload = b'timed_ee\0'
        self.assertEqual(plan.send.call_args.args[0], struct.pack('<IIQqqq', len(payload), 1, 0, 0, 0, 0) + payload)
        udp.connect.assert_called_once_with(('127.0.0.1', 10000))
        self.assertTrue(all(c.args[0] == 'tee_status' for c in t.command.call_args_list))

    def test_start_does_not_enable_robot_or_start_over_unmanaged_stream(self):
        t = TimedEeTransport(Mock(return_value=tee()), Mock(), '127.0.0.1', 5999, 5999, .2)
        with self.assertRaises(PermissionError):
            t.start(dict(network_ready=True))
        t.robot_state.assert_not_called()
        t.command.return_value = tee(active=False)
        t.robot_state.return_value = dict(ret_context=dict(robot_msg=dict(
            op_mode='Manual', activate='Disabled', status='Normal', motion='Stopped')))
        with self.assertRaisesRegex(RuntimeError, 'Enabled'):
            t.start(dict(network_ready=True))
        self.assertFalse(t.start_attempted)
        self.assertTrue(all(c.args[0] == 'tee_status' for c in t.command.call_args_list))

    def test_send_has_no_get_status_pacing_or_duplicate_retry(self):
        t = self.transport()
        t.send(POSES)
        t.send(POSES, final=True)
        packets = [json.loads(c.args[0]) for c in t.udp.send.call_args_list]
        self.assertEqual([p['index'] for p in packets], [0, 1])
        self.assertEqual([p['final'] for p in packets], [False, True])
        t.command.assert_not_called()
        t.robot_state.assert_not_called()
        with self.assertRaises(RuntimeError):
            t.send(POSES)
        self.assertEqual(t.udp.send.call_count, 2)
        t = self.transport()
        t.udp.send.return_value = 1
        t.udp.send.side_effect = None
        with self.assertRaises(ConnectionError):
            t.send(POSES)
        self.assertEqual(t.udp.send.call_count, 1)
        self.assertEqual(t.index, 0)

    def test_unlimited_wait_for_first_point_and_gap_resume(self):
        t = self.transport()
        t.started_s = -3600.
        t.last_status = tee(waiting_for_input=True, state=0)
        t.send(POSES)
        t.last_status = tee(waiting_for_input=True, state=3)
        t.send(POSES)
        self.assertEqual(t.index, 2)
        t.command.assert_not_called()

    def test_final_consumed_alone_not_command_or_actual_arrival(self):
        self.assertTrue(endpoint_reached(complete(), 2))
        for changes in (dict(consumed=1), dict(received=1), dict(end_received=False),
                        dict(final_reference_reached=False), dict(command_joint_rad=[.0001] * 12),
                        dict(command_joint_velocity_rad_s=[math.radians(.51)] * 12),
                        dict(endpoint_position_error_m=[0., .00101]),
                        dict(endpoint_orientation_error_rad=[math.radians(.101), 0.])):
            with self.subTest(changes=changes):
                self.assertFalse(endpoint_reached(complete(**changes), 2))
        self.assertTrue(endpoint_reached(complete(command_joint_rad=[[0.] * 6] * 2,
                                                  endpoint_position_error_m=0.), 2))
        with self.assertRaises(ValueError):
            endpoint_reached(complete(endpoint_position_error_m=[float('nan')] * 2), 2)

    def test_finish_returns_on_complete_sample_without_extra_wait_get_or_stop(self):
        t = self.transport()
        t.ended, t.index = True, 2
        clock = Clock()
        t.status = Mock(return_value=complete())
        with patch('fluxvla.engines.communication.a10_timed_ee.time.monotonic', clock.monotonic), \
                patch('fluxvla.engines.communication.a10_timed_ee.time.sleep', clock.sleep):
            result = t.finish()
        self.assertEqual(clock.now, 0.)
        t.status.assert_called_once()
        self.assertEqual(result['ee_pose_source'], 'command_fk')
        self.assertEqual(t.stream_id, 8)
        t.command.assert_not_called()
        t.robot_state.assert_not_called()
        t.udp.close.assert_not_called()

    def test_finish_rejects_inactive_stream_as_upstream_does(self):
        t = self.transport()
        t.ended, t.index = True, 2
        t.status = Mock(return_value=complete(active=False, state=6))
        clock = Clock()
        with patch('fluxvla.engines.communication.a10_timed_ee.time.monotonic', clock.monotonic), \
                patch('fluxvla.engines.communication.a10_timed_ee.time.sleep', clock.sleep):
            with self.assertRaisesRegex(RuntimeError, 'interrupted'):
                t.finish()

    def test_fast_v2_accepts_geometric_capture_without_exact_joint_goal(self):
        validate_status(fast_complete())
        self.assertTrue(endpoint_reached(fast_complete(), 2))
        for changes in (dict(endpoint_capture_complete=False),
                        dict(command_joint_rad=[math.radians(.301)] * 12),
                        dict(endpoint_position_error_m=[.00201, 0.]),
                        dict(endpoint_orientation_error_rad=[0., math.radians(.201)]),
                        dict(consumed=1), dict(final_reference_reached=False),
                        dict(command_joint_velocity_rad_s=[math.radians(.501)] * 12),
                        dict(active=False), dict(reason=10)):
            with self.subTest(changes=changes):
                self.assertFalse(endpoint_reached(fast_complete(**changes), 2))
        # Tighter candidates still have to honor their own joint tolerance.
        self.assertFalse(endpoint_reached(fast_complete(parameters=dict(endpoint_tolerance_deg=.15)), 2))

    def test_fast_v2_requires_capture_flag_and_declared_2mm_point2deg_band(self):
        absent = fast_complete(); del absent['endpoint_capture_complete']
        with self.assertRaises(ValueError):
            validate_status(absent)
        for change in (dict(endpoint_capture_complete=1), dict(parameters=dict(endpoint_tolerance_deg=.5)),
                       dict(parameters=dict(endpoint_tolerance_deg=-.1)),
                       dict(endpoint_position_tolerance_m=.006),
                       dict(endpoint_orientation_tolerance_rad=math.radians(1.)),
                       dict(ee_endpoint_policy='unrecognized')):
            with self.subTest(change=change), self.assertRaises((ValueError, RuntimeError)):
                validate_status(fast_complete(**change))

    def test_fast_v2_waits_for_capture_complete_then_returns_without_hold(self):
        t = self.transport(); t.ended, t.index = True, 2
        clock = Clock()
        history = [fast_complete(endpoint_capture_complete=False)] * 3
        t.status = Mock(side_effect=lambda: history.pop(0) if history else fast_complete())
        with patch('fluxvla.engines.communication.a10_timed_ee.time.monotonic', clock.monotonic), \
                patch('fluxvla.engines.communication.a10_timed_ee.time.sleep', clock.sleep):
            self.assertTrue(t.finish()['endpoint_capture_complete'])
        self.assertAlmostEqual(clock.now, .03)
        t.robot_state.assert_not_called()

    def test_near_unit_quaternion_normalization_matches_upstream(self):
        poses = np.asarray(POSES); poses[0, 6] = -1.0005
        before = poses.copy()
        encoded = np.asarray(json.loads(encode_target(8, 0, poses))['pose_m_quat'])
        np.testing.assert_allclose(encoded[0, 3:], [0., 0., 0., -1.], atol=1e-15)
        np.testing.assert_array_equal(poses, before)

    def test_ee_plan_reply_accepts_upstream_nul_termination(self):
        t = self.transport()
        payload = b'{"ret_code":0}\0'
        ws = Mock(); ws.recv.return_value = struct.pack('<IIQqqq', len(payload), 1, 0, 0, 0, 0) + payload
        t._read_plan(ws)
        self.assertIsNone(t.plan_error)

    def test_finish_keeps_original_stream_identity_during_concurrent_stop_start(self):
        t = self.transport()
        t.ended, t.index = True, 2
        def changed():
            t.stream_id = 9
            return complete(stream_id=9)
        t.status = Mock(side_effect=changed)
        with self.assertRaisesRegex(RuntimeError, 'interrupted'):
            t.finish()

    def test_finish_without_final_or_endpoint_settle_fails(self):
        t = self.transport(timeout=.05)
        with self.assertRaisesRegex(RuntimeError, 'final'):
            t.finish()
        t.ended, t.index = True, 2
        t.status = Mock(return_value=complete(endpoint_position_error_m=.006))
        clock = Clock()
        with patch('fluxvla.engines.communication.a10_timed_ee.time.monotonic', clock.monotonic), \
                patch('fluxvla.engines.communication.a10_timed_ee.time.sleep', clock.sleep):
            with self.assertRaises(TimeoutError):
                t.finish()

    def test_stop_never_targets_foreign_stream_and_ack_not_completion(self):
        t = self.transport(timeout=.03)
        t.command.return_value = tee(stream_id=9)
        with self.assertRaisesRegex(RuntimeError, 'foreign'):
            t.stop()
        self.assertEqual([c.args[0] for c in t.command.call_args_list], ['tee_status'])
        t.command.reset_mock()
        t.command.side_effect = lambda name, **_: dict(ret_code=0) if name == 'tee_stop' else tee()
        t.robot_state.return_value = dict(ret_context=dict(robot_msg=dict(motion='Running')))
        clock = Clock()
        with patch('fluxvla.engines.communication.a10_timed_ee.time.monotonic', clock.monotonic), \
                patch('fluxvla.engines.communication.a10_timed_ee.time.sleep', clock.sleep):
            with self.assertRaisesRegex(TimeoutError, 'ACK'):
                t.stop()
        self.assertEqual([c.args[0] for c in t.command.call_args_list].count('tee_stop'), 1)
        self.assertEqual(t.stream_id, 8)

    def test_inactive_owned_stream_cleanup_does_not_issue_stop(self):
        t = self.transport()
        udp = t.udp
        t.command.return_value = complete(active=False, state=6)
        t.robot_state.return_value = dict(ret_context=dict(robot_msg=dict(motion='Stopped')))
        self.assertFalse(t.stop()['active'])
        self.assertIsNone(t.stream_id)
        udp.close.assert_called_once()
        t.command.assert_called_once_with('tee_status', raw=True)

    def test_idle_joint_history_only_allows_start_after_strict_new_ee_confirmation(self):
        idle = dict(ret_code=0, active=False, stream_id=4, implementation='joint_tracking.TimedAj')
        t = TimedEeTransport(Mock(side_effect=[idle, tee(stream_id=9)]), Mock(), '127.0.0.1', 5999, 5999, .2)
        t.robot_state.return_value = dict(ret_context=dict(robot_msg=dict(op_mode='Manual', activate='Enabled', status='Normal', motion='Stopped')))
        with patch('websockets.sync.client.connect'), \
                patch('fluxvla.engines.communication.a10_timed_ee.socket.socket'), \
                patch('fluxvla.engines.communication.a10_timed_ee.threading.Thread'):
            self.assertEqual(t.start(dict(network_ready=True))['stream_id'], 9)
        self.assertEqual(t.last_status['tool'], 'tool1')
        self.assertEqual(t.stream_id, 9)

    def test_idle_history_never_grants_target_permission(self):
        t = TimedEeTransport(Mock(return_value=dict(ret_code=0, active=False, stream_id=0)), Mock(), '127.0.0.1', 5999, 5999, .2)
        self.assertFalse(t.status()['active'])
        self.assertIsNone(t.last_status)
        with self.assertRaisesRegex(RuntimeError, 'confirm'):
            t.send(POSES)
        t.command.return_value = dict(ret_code=0, active=True, stream_id=9, implementation='joint_tracking.TimedAj')
        with self.assertRaisesRegex(RuntimeError, 'received.*TimedAj'):
            t.status()

    def test_invalid_profile_after_start_retains_identity_for_own_stop(self):
        t = TimedEeTransport(Mock(), Mock(), '127.0.0.1', 5999, 5999, .2)
        t.start_attempted, t._starting_after_id = True, 4
        t.command.return_value = tee(stream_id=9, tool='tool0')
        with self.assertRaisesRegex(RuntimeError, 'tool1'):
            t.status()
        self.assertEqual(t.stream_id, 9)
        t.command.side_effect = [tee(stream_id=9, tool='tool0'), dict(ret_code=0), tee(stream_id=9, active=False, tool='tool0')]
        t.robot_state.return_value = dict(ret_context=dict(robot_msg=dict(motion='Stopped')))
        self.assertFalse(t.stop()['active'])


if __name__ == '__main__':
    unittest.main()
