"""A10 reference protocol tests. All endpoints are loopback or in-memory."""

import json
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
                       'motor_pos': [[1, 2, 3, 4, 5, 6]],
                       'pq': [[100, 200, 300, 0, 0, 0, 1]]},
        'follower_msg': {'running_state': active, 'follower_mode': mode},
        'driver_msg': {'driver_err_code': [0] * 6},
        'robot_msg': {'cs_error_code': 0}, 'log_msg': {'has_error': False}}}


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
            with self.assertRaises(NotImplementedError):
                operator.send_joints({'right': [0] * 6})
            operator.close()
            operator.close()


if __name__ == '__main__':
    unittest.main()
