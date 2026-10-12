"""A10 reference protocol tests. All endpoints are loopback or in-memory."""

import json
import socket
import threading
import time
import struct
import unittest
from unittest.mock import patch

import numpy as np

from fluxvla.engines import build_operator_from_cfg
from fluxvla.engines.operators.a10_operator import (
    A10ControllerError, A10Operator, A10ProtocolError, _decode_reply, _pack,
    _parse_state, _pose, _unpack)
from fluxvla.engines.operators.base_operator import BaseOperator


def status_reply(*, active=False, mode='None'):
    return {'ret_code': 0, 'ret_msg': '', 'ret_context': {
        'motion_msg': {'actual_pos': [[0, 90, -90, 180, 0, 45]],
                       'jog_coordinate': 'Tool',
                       'motor_pos': [[1, 2, 3, 4, 5, 6]],
                       'pq': [[100, 200, 300, 0, 0, 0, 1]]},
        'follower_msg': {'running_state': active, 'follower_mode': mode},
        'driver_msg': {'driver_err_code': [0] * 6},
        'robot_msg': {'cs_error_code': 0, 'motion': 'Stopped',
                      'op_mode': 'Manual', 'activate': 'Enabled'},
        'log_msg': {'has_error': False}}}


def wire_reply(reply):
    """Independent reference encoder; production codec is not used here."""
    payload = json.dumps(reply).encode('utf-8')
    return (len(payload).to_bytes(4, 'little') + bytes(36) + payload)


def parse(reply, **kwargs):
    options = dict(joint_unit='deg', position_unit=None, gripper_feedback_path=None)
    options.update(kwargs)
    return _parse_state(reply, **options)


class CodecTest(unittest.TestCase):
    def test_reference_command_and_udp_headers(self):
        for command, fields in ((True, (1, 0x1000, 0xA1B2C3D4, 0, 0)),
                                (False, (0, 0, 0, 0, 0))):
            packet = _pack(b'get', command=command)
            self.assertEqual(len(packet), 43)
            self.assertEqual(struct.unpack('<IIQqqq', packet[:40]), (3, *fields))
            self.assertEqual(packet[40:], b'get')

    def test_nested_context_and_bad_frames(self):
        reply = status_reply()
        reply['ret_context'] = json.dumps(reply['ret_context'])
        self.assertIn('motion_msg', _decode_reply(wire_reply(reply))['ret_context'])
        valid = wire_reply(status_reply())
        for packet in (b'', bytes(39), 'text', valid[:-1], valid + b'x',
                       (2**21).to_bytes(4, 'little') + valid[4:]):
            with self.subTest(packet_type=type(packet), length=len(packet)):
                with self.assertRaises(A10ProtocolError):
                    _unpack(packet)
        for reply in ({'ret_code': True}, {'ret_code': '0'}, {}, []):
            with self.assertRaises(A10ProtocolError):
                _decode_reply(wire_reply(reply))
        with self.assertRaises(A10ControllerError):
            _decode_reply(wire_reply({'ret_code': 7, 'ret_msg': 'not enabled'}))
        payload = b'{"ret_code":0,"ret_context":{"x":NaN}}'
        with self.assertRaises(A10ProtocolError):
            _decode_reply(len(payload).to_bytes(4, 'little') + bytes(36) + payload)

    def test_feedback_units_and_missing_gripper(self):
        reply = status_reply()
        state = parse(reply)
        np.testing.assert_allclose(state.actual_joints_rad,
                                   [0, np.pi/2, -np.pi/2, np.pi, 0, np.pi/4])
        np.testing.assert_allclose(state.target_joints_rad, np.deg2rad([1, 2, 3, 4, 5, 6]))
        self.assertIsNone(state.qpos)
        self.assertIsNone(state.gripper_mm)
        self.assertIsNone(state.tcp_pq_m)
        self.assertEqual(state.tcp_pq_raw[0], 100)
        # Synthetic field only; this is NOT a claimed controller field.
        reply['ret_context']['test_gripper'] = {'actual_mm': 42.5}
        state = parse(reply, position_unit='mm', gripper_feedback_path=['test_gripper', 'actual_mm'])
        np.testing.assert_allclose(state.tcp_pq_m[:3], [.1, .2, .3])
        self.assertEqual(state.qpos.shape, (7,))
        self.assertEqual(state.qpos[-1], 42.5)
        flat = status_reply()
        flat['ret_context']['motion_msg']['actual_pos'] = [1.] * 6
        np.testing.assert_array_equal(parse(flat, joint_unit='rad').actual_joints_rad, [1.] * 6)

    def test_single_arm_tcp_and_auxiliary_axis_pose(self):
        reply = status_reply()
        reply['ret_context']['motion_msg']['pq'].append([0, 0, 0, 0, 0, 0, 1])
        state = parse(reply, position_unit='mm')
        np.testing.assert_allclose(state.tcp_pq_m[:3], [.1, .2, .3])
        reply['ret_context']['motion_msg']['pq'].append([0] * 7)
        with self.assertRaises(A10ProtocolError):
            parse(reply)

    def test_target_does_not_substitute_actual_feedback(self):
        reply = status_reply()
        del reply['ret_context']['motion_msg']['actual_pos']
        state = parse(reply)
        self.assertIsNone(state.actual_joints_rad)
        self.assertIsNotNone(state.target_joints_rad)
        self.assertIsNone(state.qpos)

    def test_other_robot_shapes_and_invalid_feedback_rejected(self):
        for values in ([0.] * 7, [0.] * 20, [[0.] * 6, [0.] * 6],
                       [False] * 6, ['0'] * 6, [float('nan')] * 6):
            reply = status_reply()
            reply['ret_context']['motion_msg']['actual_pos'] = values
            with self.assertRaises(A10ProtocolError):
                parse(reply)
        reply = status_reply()
        reply['ret_context']['follower_msg']['running_state'] = 'false'
        with self.assertRaises(A10ProtocolError):
            parse(reply)
        reply = status_reply()
        reply['ret_context']['driver_msg']['driver_err_code'][3] = 9
        self.assertTrue(parse(reply).has_error)

    def test_pose_validation_preserves_units_and_small_values(self):
        pose = [.000001, .2, -.3, 0, 0, 0, 1]
        np.testing.assert_array_equal(_pose(pose), pose)
        for pose in ([0] * 6, [0] * 8, [0] * 7,
                     [0, 0, 0, 0, 0, 0, 2], [float('inf'), 0, 0, 0, 0, 0, 1]):
            with self.assertRaises(A10ProtocolError):
                _pose(pose)

    def test_registry_construction_and_motion_gate_have_no_io(self):
        with patch('socket.socket', side_effect=AssertionError('unexpected socket')):
            operator = build_operator_from_cfg(dict(type='A10Operator', host='127.0.0.1'))
            self.assertIsInstance(operator, BaseOperator)
            self.assertFalse(operator.is_connected)
            with self.assertRaises(PermissionError):
                operator.start_follower()
            with self.assertRaises(PermissionError):
                operator.send_gripper({'right': 50})
            with self.assertRaises(PermissionError):
                operator.send_joints({'right': [0] * 6})
            operator.close()
            operator.close()


class FakeController:
    """Independent loopback server implementing only the reference wire shape."""

    def __init__(self, *, change_mode=True, initial_active=False, fault=None, no_pong=False):
        self.change_mode = change_mode
        self.active = initial_active
        self.fault = fault
        self.no_pong = no_pong
        self.commands, self.errors, self.operators = [], [], []
        self.actual = True
        self.tcp = True
        self.start_mode_override = None
        self.mode = 'Cart' if initial_active else 'None'
        self.driver_fault = False
        self.coordinate = 'Tool'
        self.motion = 'Stopped'
        self.op_mode = 'Manual'
        self.activation = 'Enabled'
        self.activation_change = True
        self.reject_enable = False
        self.coordinate_change = True
        self.start_rejected = False
        self.active_after_rejection = False
        self.recovery_fault = None
        self.lock = threading.Lock()

    def __enter__(self):
        from websockets.sync.server import serve, ServerConnection
        from websockets.frames import OP_PING
        class NoPongConnection(ServerConnection):
            def __init__(self, sock, protocol, **kwargs):
                receive = protocol.recv_frame
                def without_pong(frame):
                    if frame.opcode != OP_PING:
                        receive(frame)
                protocol.recv_frame = without_pong
                super().__init__(sock, protocol, **kwargs)
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.bind(('127.0.0.1', 0))
        self.udp.settimeout(.2)
        self.joint_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.joint_udp.bind(('127.0.0.1', 0))
        self.joint_udp.settimeout(.05)
        self.server = serve(self.handle, '127.0.0.1', 0,
                            create_connection=NoPongConnection if self.no_pong else None)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        return self

    def operator(self, **kwargs):
        options = dict(host='127.0.0.1', ws_port=self.server.socket.getsockname()[1],
                       udp_port=self.udp.getsockname()[1], timeout_s=.3,
                       joint_udp_port=self.joint_udp.getsockname()[1],
                       allow_motion=True)
        options.update(kwargs)
        operator = A10Operator(**options)
        self.operators.append(operator)
        operator.connect()
        return operator

    def handle(self, ws):
        from websockets.exceptions import ConnectionClosed
        try:
            for packet in ws:
                # Check reference fields independently, without production helpers.
                assert isinstance(packet, bytes)
                assert len(packet) >= 40
                size, msg_id, kind, reserved1, reserved2, reserved3 = struct.unpack(
                    '<IIQqqq', packet[:40])
                assert (msg_id, kind, reserved1, reserved2, reserved3) == (
                    1, 0x1000, 0xA1B2C3D4, 0, 0)
                assert len(packet[40:]) == size
                command = packet[40:].decode('utf-8')
                with self.lock:
                    self.commands.append(command)
                if command == 'get':
                    reply = status_reply(active=self.active, mode=self.mode)
                    reply['ret_context']['motion_msg']['jog_coordinate'] = self.coordinate
                    reply['ret_context']['robot_msg']['motion'] = self.motion
                    reply['ret_context']['robot_msg']['op_mode'] = self.op_mode
                    reply['ret_context']['robot_msg']['activate'] = self.activation
                    if not self.actual:
                        del reply['ret_context']['motion_msg']['actual_pos']
                    if not self.tcp:
                        del reply['ret_context']['motion_msg']['pq']
                    if self.driver_fault:
                        reply['ret_context']['driver_msg']['driver_err_code'][0] = 8
                    reply['ret_context']['test_gripper'] = {'actual_mm': 42.5}
                    reply['ret_context'] = json.dumps(reply['ret_context'])
                    if self.fault == 'timeout':
                        continue
                    if self.fault == 'disconnect':
                        ws.close()
                        return
                    if self.fault == 'ret_code':
                        reply = {'ret_code': 9, 'ret_msg': 'denied', 'ret_context': ''}
                    if self.fault == 'bad_state':
                        reply['ret_context'] = {'motion_msg': {'actual_pos': [0] * 20}}
                    frame = wire_reply(reply)
                    if self.fault == 'length':
                        frame = frame[:-1]
                    elif self.fault == 'text':
                        frame = json.dumps(reply)
                    elif self.fault == 'ack_flood':
                        for _ in range(33):
                            ws.send(wire_reply({'ret_code': 0, 'ret_msg': '', 'ret_context': ''}))
                        continue
                    ws.send(frame)
                elif command in ('follower_cart', 'follower_aj', 'stop_follower'):
                    if command != 'stop_follower' and self.start_rejected:
                        self.active = self.active_after_rejection
                        self.fault = self.recovery_fault
                        ws.send(wire_reply({'ret_code': -987313,
                            'ret_msg': '请切换到笛卡尔坐标系', 'ret_context': ''}))
                        continue
                    if self.change_mode:
                        self.active = command != 'stop_follower'
                        self.mode = (self.start_mode_override or
                                     ('Axis' if command == 'follower_aj' else 'Cart')) if self.active else 'None'
                    # Delayed empty ACK is intentionally ahead of the get reply.
                    ws.send(wire_reply({'ret_code': 0, 'ret_msg': '', 'ret_context': ''}))
                elif command in ('set_jog_coordinate --tool', 'set_jog_coordinate --axis'):
                    if self.coordinate_change:
                        self.coordinate = 'Axis' if command.endswith('--axis') else 'Tool'
                    ws.send(wire_reply({'ret_code': 0, 'ret_msg': '', 'ret_context': ''}))
                elif command in ('manual_en', 'manual_ds'):
                    if self.reject_enable:
                        ws.send(wire_reply({'ret_code': 9, 'ret_msg': 'denied', 'ret_context': ''}))
                        continue
                    if self.activation_change:
                        self.activation = 'Enabled' if command == 'manual_en' else 'Disabled'
                    ws.send(wire_reply({'ret_code': 0, 'ret_msg': '', 'ret_context': ''}))
                elif command.startswith('g_move --pos='):
                    float(command.split('=')[1])
                    ws.send(wire_reply({'ret_code': 0, 'ret_msg': '', 'ret_context': ''}))
                else:
                    raise AssertionError(f'Unexpected command: {command}')
        except ConnectionClosed:
            pass
        except Exception as exc:
            self.errors.append(exc)

    def __exit__(self, exc_type, exc, tb):
        for operator in self.operators:
            operator.close()
        self.server.shutdown()
        self.thread.join(timeout=2)
        self.udp.close()
        self.joint_udp.close()
        if exc is None:
            if self.thread.is_alive():
                raise AssertionError('Fake server did not terminate')
            if self.errors:
                raise self.errors[0]


class NetworkTest(unittest.TestCase):
    def test_joint_roundtrip_wire_degrees_and_separate_ports(self):
        with FakeController() as controller:
            controller.coordinate = 'Axis'
            controller.tcp = False  # Joint start doesn't require a TCP pose.
            operator = controller.operator()
            self.assertEqual(operator.start_follower(mode='joint').follower_mode, 'Axis')
            self.assertEqual(operator.command_mode, 'joint')
            # A complete 6-axis absolute target; no seventh gripper value.
            operator.send_joints({'right': [0, np.pi/2, -np.pi/2, np.pi, 0, np.pi/4]})
            packet, _ = controller.joint_udp.recvfrom(65535)
            self.assertEqual(struct.unpack('<IIQqqq', packet[:40]),
                             (len(packet)-40, 0, 0, 0, 0, 0))
            self.assertEqual(packet[40:], b'jp --motor_pos=[0,90,-90,180,0,45]')
            with self.assertRaises(socket.timeout):
                controller.udp.recvfrom(65535)
            self.assertFalse(operator.stop_follower().follower_active)
            self.assertEqual(controller.commands, ['get','follower_aj','get','stop_follower','get'])
            operator.close()
            self.assertIsNone(operator._connection.joint_udp)

    def test_joint_and_cartesian_targets_and_mode_switch_are_exclusive(self):
        with FakeController() as controller:
            operator = controller.operator()
            operator.start_follower(mode='joint')
            with self.assertRaisesRegex(RuntimeError, 'Cart'):
                operator.send_eepose({'right': [0,0,0,0,0,0,1]})
            with self.assertRaisesRegex(RuntimeError, 'stop_follower'):
                operator.start_follower(mode='cartesian')
            operator.stop_follower()
            operator.start_follower(mode='cartesian')
            with self.assertRaisesRegex(RuntimeError, 'Axis'):
                operator.send_joints({'right': [0]*6})
            with self.assertRaises(socket.timeout):
                controller.joint_udp.recvfrom(65535)
            operator.stop_follower()
            self.assertEqual(controller.commands.count('follower_aj'), 1)
            self.assertEqual(controller.commands.count('follower_cart'), 1)

    def test_joint_invalid_targets_never_reach_either_socket(self):
        with FakeController() as controller:
            operator = controller.operator(command_mode='joint')
            operator.start_follower()
            for value in ([0]*5, [0]*7, [[0]*6], [False]*6, ['0']*6,
                          [float('nan')]*6, [float('inf')]*6, [1e308]*6):
                with self.subTest(value=value), self.assertRaises(A10ProtocolError):
                    operator.send_joints({'right': value})
            for target in ({'left':[0]*6}, {'right':[0]*6,'left':[0]*6}):
                with self.assertRaises(ValueError):
                    operator.send_joints(target)
            with self.assertRaises(socket.timeout):
                controller.joint_udp.recvfrom(65535)
            operator.stop_follower()

    def test_joint_wrong_feedback_or_no_transition_cannot_authorize_udp(self):
        for override, transition in (('Cart',True), (None,False)):
            with self.subTest(override=override), FakeController(change_mode=transition) as controller:
                controller.start_mode_override = override
                operator = controller.operator(timeout_s=.05)
                with self.assertRaises(TimeoutError):
                    operator.start_follower(mode='joint')
                with self.assertRaises(RuntimeError):
                    operator.send_joints({'right':[0]*6})
                self.assertTrue(operator.follower_stop_pending)
                controller.change_mode = True
                operator.stop_follower()

    def test_joint_rejection_recovers_only_read_only(self):
        with FakeController() as controller:
            controller.start_rejected = True
            operator = controller.operator()
            with self.assertRaises(A10ControllerError):
                operator.start_follower(mode='joint')
            self.assertFalse(operator.follower_stop_pending)
            self.assertEqual(controller.commands.count('follower_aj'), 1)
            self.assertNotIn('stop_follower', controller.commands)
            with self.assertRaises(RuntimeError):
                operator.send_joints({'right':[0]*6})

    def test_later_feedback_cannot_authorize_an_unconfirmed_joint_start(self):
        for rejected in (False,True):
            with self.subTest(rejected=rejected), FakeController(change_mode=False) as controller:
                controller.start_rejected = rejected
                controller.active_after_rejection = rejected
                operator = controller.operator(timeout_s=.05)
                with self.assertRaises(A10ControllerError if rejected else TimeoutError):
                    operator.start_follower(mode='joint')
                controller.active, controller.mode = True, 'Axis'
                self.assertTrue(operator.get_state().follower_active)
                with self.assertRaises(RuntimeError):
                    operator.send_joints({'right':[0]*6})
                controller.change_mode = True
                operator.stop_follower()

    def test_joint_status_changes_revoke_send_and_reconnect_loses_ownership(self):
        for field,value in (('mode','Cart'),('active',False),('driver_fault',True),('activation','Disabled')):
            with self.subTest(field=field), FakeController() as controller:
                operator = controller.operator()
                operator.start_follower(mode='joint')
                setattr(controller,field,value)
                operator.get_state()
                with self.assertRaises(RuntimeError):
                    operator.send_joints({'right':[0]*6})
        with FakeController() as controller:
            operator = controller.operator()
            operator.start_follower(mode='joint')
            operator.close()
            operator.connect()
            self.assertEqual(operator.get_state().follower_mode, 'Axis')
            with self.assertRaises(RuntimeError):
                operator.send_joints({'right':[0]*6})
            with self.assertRaises(RuntimeError):
                operator.stop_follower()

    def test_joint_udp_send_failure_revokes_further_targets(self):
        for failed in (True,False):
            with self.subTest(failed=failed), FakeController() as controller:
                operator=controller.operator()
                operator.start_follower(mode='joint')
                with patch.object(operator._connection, 'joint_udp') as udp:
                    if failed:
                        udp.send.side_effect=OSError('UDP unavailable')
                    else:
                        udp.send.return_value=1  # Cannot claim a complete send.
                    with self.assertRaises(OSError):
                        operator.send_joints({'right':[0]*6})
                    self.assertFalse(operator._confirmed_active)
                    with self.assertRaises(RuntimeError):
                        operator.send_joints({'right':[0]*6})
                    self.assertEqual(udp.send.call_count,1)
                operator.stop_follower()

    def test_joint_start_conditions_and_readonly_gate(self):
        for field,value in (('actual',False),('active',True),('activation','Disabled'),
                            ('motion','Running'),('driver_fault',True)):
            with self.subTest(field=field), FakeController() as controller:
                setattr(controller,field,value)
                operator = controller.operator()
                with self.assertRaises((RuntimeError,A10ProtocolError)):
                    operator.start_follower(mode='joint')
                self.assertNotIn('follower_aj', controller.commands)
        with FakeController() as controller:
            operator = controller.operator(allow_motion=False)
            for call in (lambda:operator.start_follower(mode='joint'),
                         lambda:operator.send_joints({'right':[0]*6}),
                         lambda:operator.set_jog_coordinate('Axis')):
                with self.assertRaises(PermissionError):
                    call()
            self.assertEqual(controller.commands, [])

    def test_axis_coordinate_selection_is_explicit_and_confirmed(self):
        with FakeController() as controller:
            operator = controller.operator()
            self.assertEqual(operator.set_jog_coordinate('Axis').raw['ret_context']['motion_msg']['jog_coordinate'], 'Axis')
            self.assertEqual(controller.commands, ['get','set_jog_coordinate --axis','get'])
            operator.set_tool_coordinate()
            self.assertEqual(controller.coordinate, 'Tool')
            with self.assertRaises(ValueError):
                operator.set_jog_coordinate('Unknown')

    def test_second_udp_socket_setup_failure_releases_all_resources(self):
        with FakeController() as controller:
            operator = A10Operator(host='127.0.0.1', ws_port=controller.server.socket.getsockname()[1])
            real_socket = socket.socket
            made = []
            def socket_factory(*args,**kwargs):
                if args[:2] == (socket.AF_INET,socket.SOCK_DGRAM):
                    if made:
                        raise OSError('second UDP socket failed')
                    sock = real_socket(*args,**kwargs)
                    made.append(sock)
                    return sock
                return real_socket(*args,**kwargs)
            with patch('socket.socket',socket_factory), self.assertRaises(OSError):
                operator.connect()
            self.assertFalse(operator.is_connected)
            self.assertEqual(made[0].fileno(), -1)
            self.assertIsNone(operator._connection.ws)
            self.assertIsNone(operator._connection.joint_udp)

    def test_idle_controller_without_pong_still_answers_get(self):
        from websockets.sync.client import connect
        def short_defaults(*args, **kwargs):
            # Accelerate the library's default heartbeat for regression. An
            # explicit None from the adapter must retain its usual semantics.
            kwargs.setdefault('ping_interval', .02)
            kwargs['ping_timeout'] = .02
            return connect(*args, **kwargs)
        with FakeController(no_pong=True) as controller:
            with patch('websockets.sync.client.connect', short_defaults):
                operator = controller.operator()
            self.assertFalse(operator.get_state().follower_active)
            self.assertFalse(operator._connection.ws.ping().wait(.03))
            time.sleep(.08)
            self.assertFalse(operator.get_state().follower_active)
            controller.fault = 'timeout'
            with self.assertRaises(TimeoutError):
                operator.get_state(timeout_s=.05)
            self.assertFalse(operator.is_connected)

    def test_manual_enable_disable_confirm_feedback(self):
        with FakeController() as controller:
            controller.activation = 'Disabled'
            operator = controller.operator()
            self.assertEqual(operator.enable().raw['ret_context']['robot_msg']['activate'], 'Enabled')
            self.assertEqual(operator.disable().raw['ret_context']['robot_msg']['activate'], 'Disabled')
            self.assertEqual(controller.commands, ['get', 'manual_en', 'get', 'get', 'manual_ds', 'get'])

    def test_enable_preconditions_never_send_control(self):
        for field, value in (('op_mode', 'Auto'), ('op_mode', None),
                             ('motion', 'Running'), ('motion', None),
                             ('driver_fault', True), ('active', True)):
            with self.subTest(field=field), FakeController() as controller:
                setattr(controller, field, value)
                operator = controller.operator()
                with self.assertRaises(RuntimeError):
                    operator.enable()
                self.assertEqual(controller.commands, ['get'])

    def test_enable_ack_timeout_and_rejection(self):
        for rejected in (False, True):
            with self.subTest(rejected=rejected), FakeController() as controller:
                controller.activation = 'Disabled'
                controller.activation_change = False
                controller.reject_enable = rejected
                operator = controller.operator(timeout_s=.1)
                with self.assertRaises(A10ControllerError if rejected else TimeoutError):
                    operator.enable()
                self.assertEqual(controller.commands.count('manual_en'), 1)

    def test_disable_during_fault_and_stop_confirmation(self):
        with FakeController() as controller:
            operator = controller.operator()
            operator.start_follower()
            self.assertTrue(operator.follower_stop_pending)
            controller.driver_fault = True
            self.assertTrue(operator.disable().has_error)
            with self.assertRaises(RuntimeError):
                operator.send_eepose({'right': [0, 0, 0, 0, 0, 0, 1]})
            self.assertFalse(operator.stop_follower().follower_active)
            self.assertFalse(operator.follower_stop_pending)

    def test_disabled_follower_start_refused(self):
        with FakeController() as controller:
            controller.activation = 'Disabled'
            operator = controller.operator()
            with self.assertRaisesRegex(RuntimeError, 'Enabled'):
                operator.start_follower()
            self.assertEqual(controller.commands, ['get'])

    def test_coordinate_precheck_and_explicit_selection(self):
        with FakeController() as controller:
            controller.coordinate = 'Axis'
            operator = controller.operator()
            with self.assertRaisesRegex(RuntimeError, 'set_tool_coordinate'):
                operator.start_follower()
            self.assertFalse(operator._started_here)
            self.assertEqual(controller.commands, ['get'])
            state = operator.set_tool_coordinate()
            self.assertEqual(state.raw['ret_context']['motion_msg']['jog_coordinate'], 'Tool')
            self.assertEqual(controller.commands, ['get', 'get', 'set_jog_coordinate --tool', 'get'])
            self.assertTrue(operator.start_follower().follower_active)
            with self.assertRaisesRegex(RuntimeError, 'Stop the follower'):
                operator.set_tool_coordinate()
            operator.stop_follower()
            with self.assertRaises(socket.timeout):
                controller.udp.recvfrom(65535)

    def test_coordinate_ack_requires_status_confirmation(self):
        with FakeController() as controller:
            controller.coordinate = 'Axis'
            controller.coordinate_change = False
            operator = controller.operator(timeout_s=.1)
            with self.assertRaisesRegex(TimeoutError, 'Tool jog coordinate'):
                operator.set_tool_coordinate()
            self.assertNotIn('follower_cart', controller.commands)

    def test_coordinate_change_refuses_fault_or_unconfirmed_stop(self):
        for motion, fault in (('Running', False), (None, False), ('Stopped', True)):
            with self.subTest(motion=motion, fault=fault), FakeController() as controller:
                controller.motion, controller.driver_fault = motion, fault
                operator = controller.operator()
                with self.assertRaises(RuntimeError):
                    operator.set_tool_coordinate()
                self.assertEqual(controller.commands, ['get'])

    def test_rejected_start_uses_fresh_read_only_recovery(self):
        with FakeController() as controller:
            controller.start_rejected = True
            operator = controller.operator()
            old_socket = operator._connection.ws
            with self.assertRaisesRegex(A10ControllerError, '-987313'):
                operator.start_follower()
            self.assertTrue(operator.is_connected)
            self.assertIsNot(operator._connection.ws, old_socket)
            self.assertFalse(operator._started_here)
            self.assertFalse(operator._confirmed_active)
            self.assertFalse(operator.get_state().follower_active)
            self.assertEqual(controller.commands.count('follower_cart'), 1)
            self.assertNotIn('stop_follower', controller.commands)
            self.assertTrue(all(c in ('get', 'follower_cart') for c in controller.commands))

    def test_rejection_with_uncertain_or_active_state_keeps_cleanup_pending(self):
        for active, fault in ((True, None), (False, 'disconnect')):
            with self.subTest(active=active), FakeController() as controller:
                controller.start_rejected = True
                controller.active_after_rejection = active
                controller.recovery_fault = fault
                operator = controller.operator()
                with self.assertRaisesRegex(A10ControllerError, '-987313'):
                    operator.start_follower()
                self.assertTrue(operator._started_here)
                self.assertFalse(operator._confirmed_active)
                with self.assertRaises((RuntimeError, ConnectionError)):
                    operator.send_eepose({'right': [0, 0, 0, 0, 0, 0, 1]})
                self.assertNotIn('stop_follower', controller.commands)
                self.assertEqual(controller.commands.count('follower_cart'), 1)

    def test_ws_status_start_udp_pose_gripper_and_stop(self):
        with FakeController() as controller:
            operator = controller.operator(feedback_position_unit='mm',
                gripper_feedback_path=['test_gripper', 'actual_mm'])
            frame = operator.get_frame()
            np.testing.assert_allclose(frame['qpos'], [0, np.pi/2, -np.pi/2, np.pi, 0, np.pi/4, 42.5])
            np.testing.assert_allclose(frame['tcp_pq_m'][:3], [.1, .2, .3])
            self.assertTrue(operator.start_follower().follower_active)
            target = [.001, -.002, .003, 0, 0, np.sin(.1), np.cos(.1)]
            operator.send_eepose({'right': target})
            packet, address = controller.udp.recvfrom(65535)
            self.assertEqual(address[0], '127.0.0.1')
            self.assertEqual(packet[4:40], bytes(36))
            self.assertEqual(int.from_bytes(packet[:4], 'little'), len(packet) - 40)
            payload = json.loads(packet[40:])
            self.assertEqual(set(payload), {'type', 'pq'})
            self.assertEqual(payload['type'], 'pq')
            self.assertEqual(payload['pq'], [target])
            self.assertEqual(operator.send_gripper({'right': 35.5})['ret_code'], 0)
            self.assertFalse(operator.stop_follower().follower_active)
            operator.close()
            self.assertIsNone(operator._connection.ws)
            self.assertIsNone(operator._connection.udp)
            self.assertEqual(controller.commands,
                ['get', 'get', 'follower_cart', 'get', 'get', 'g_move --pos=35.5', 'stop_follower', 'get'])

    def test_ack_without_mode_transition_cannot_enable_udp(self):
        with FakeController(change_mode=False) as controller:
            operator = controller.operator(timeout_s=.15)
            with self.assertRaises(TimeoutError):
                operator.start_follower()
            with self.assertRaisesRegex(RuntimeError, 'not been confirmed'):
                operator.send_eepose({'right': [0, 0, 0, 0, 0, 0, 1]})
            with self.assertRaises(socket.timeout):
                controller.udp.recvfrom(65535)
            # A timed-out start can still be explicitly stopped on this connection.
            self.assertFalse(operator.stop_follower().follower_active)

    def test_refuse_takeover_or_missing_actual_joints(self):
        with FakeController(initial_active=True) as controller:
            operator = controller.operator()
            with self.assertRaisesRegex(RuntimeError, 'refusing takeover'):
                operator.start_follower()
            with self.assertRaisesRegex(RuntimeError, 'did not start'):
                operator.stop_follower()
            with self.assertRaises(RuntimeError):
                operator.send_eepose({'right': [0, 0, 0, 0, 0, 0, 1]})
            self.assertEqual(controller.commands, ['get'])
        with FakeController() as controller:
            controller.actual = False
            operator = controller.operator()
            with self.assertRaisesRegex(A10ProtocolError, 'requires joint'):
                operator.start_follower()
            self.assertEqual(controller.commands, ['get'])

    def test_read_only_operator_never_sends_motion(self):
        with FakeController() as controller:
            operator = controller.operator(allow_motion=False)
            self.assertIsNone(operator.get_frame()['qpos'])
            for action in (operator.start_follower, operator.stop_follower, operator.set_tool_coordinate,
                           operator.enable, operator.disable,
                           lambda: operator.send_eepose({'right': [0, 0, 0, 0, 0, 0, 1]}),
                           lambda: operator.send_gripper({'right': 40})):
                with self.assertRaises(PermissionError):
                    action()
            operator.close()
            self.assertEqual(controller.commands, ['get'])

    def test_faulty_transactions_close_connections(self):
        from websockets.exceptions import ConnectionClosed
        for fault, error in [('timeout', TimeoutError), ('disconnect', ConnectionClosed),
                             ('ret_code', A10ControllerError), ('length', A10ProtocolError),
                             ('text', A10ProtocolError), ('ack_flood', A10ProtocolError),
                             ('bad_state', A10ProtocolError)]:
            with self.subTest(fault=fault), FakeController(fault=fault) as controller:
                operator = controller.operator(timeout_s=.1)
                with self.assertRaises(error):
                    operator.get_state()
                self.assertFalse(operator.is_connected)
                self.assertIsNone(operator._connection.udp)
                self.assertFalse(operator._confirmed_active)

    def test_invalid_targets_never_reach_wire(self):
        with FakeController() as controller:
            operator = controller.operator()
            operator.start_follower()
            commands = list(controller.commands)
            for target in ({'left': [0] * 7}, {'right': [0] * 7},
                           {'right': [0, 0, 0, 0, 0, 0, 1], 'left': [0] * 7}):
                with self.assertRaises(ValueError):
                    operator.send_eepose(target)
            for target in ({'right': -1}, {'right': 101}, {'right': float('nan')},
                           {'right': True}, {'left': 50}):
                with self.assertRaises(ValueError):
                    operator.send_gripper(target)
            with self.assertRaises(NotImplementedError):
                operator.send_gripper({'right': 50}, wait=True)
            self.assertEqual(controller.commands, commands)
            with self.assertRaises(socket.timeout):
                controller.udp.recvfrom(65535)
            operator.stop_follower()

    def test_reported_fault_revokes_pose_send(self):
        with FakeController() as controller:
            operator = controller.operator()
            operator.start_follower()
            controller.driver_fault = True
            self.assertTrue(operator.get_state().has_error)
            with self.assertRaises(RuntimeError):
                operator.send_eepose({'right': [0, 0, 0, 0, 0, 0, 1]})
            with self.assertRaises(A10ControllerError):
                operator.send_gripper({'right': 50})
            self.assertFalse(any(x.startswith('g_move') for x in controller.commands))
            controller.driver_fault = False
            operator.stop_follower()

    def test_parallel_reads_are_serialized(self):
        with FakeController() as controller:
            operator = controller.operator()
            results, errors = [], []
            def query():
                try:
                    results.append(operator.get_state())
                except Exception as exc:
                    errors.append(exc)
            threads = [threading.Thread(target=query) for _ in range(6)]
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join(timeout=2)
            self.assertEqual(errors, [])
            self.assertEqual(len(results), 6)
            self.assertEqual(controller.commands, ['get'] * 6)

    def test_reconnect_does_not_reuse_old_follower_ownership(self):
        with FakeController() as controller:
            operator = controller.operator()
            operator.start_follower()
            operator.close()
            operator.connect()
            self.assertTrue(operator.get_state().follower_active)
            with self.assertRaises(RuntimeError):
                operator.send_eepose({'right': [0, 0, 0, 0, 0, 0, 1]})
            self.assertNotIn('stop_follower', controller.commands)


if __name__ == '__main__':
    unittest.main()
