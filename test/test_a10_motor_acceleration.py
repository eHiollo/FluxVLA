"""Startup motor policy tests; in-memory only, no physical controller."""
import copy
import json
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import Mock, patch

from test_a10_set_acc import Robot
from fluxvla.engines.operators.kaanh_commands.motor_acceleration import ensure_motor_acceleration
from fluxvla.engines.communication.a10_shared_server import SharedControl


class MotorPolicyTests(unittest.TestCase):
    def test_matching_enabled_configuration_is_read_only(self):
        robot = Robot()
        result = ensure_motor_acceleration(robot, 400)
        self.assertFalse(result['changed'])
        self.assertEqual(robot.calls, [])

    def test_mismatch_never_prepares_controller_state(self):
        for attribute, value in [('activate', 'Enabled'), ('program', 'Loaded'),
                                 ('motion', 'Running'), ('active', True),
                                 ('user', 'Operator'), ('mode', 'Auto')]:
            robot = self.prepared()
            setattr(robot, attribute, value)
            with self.subTest(attribute=attribute), self.assertRaises(RuntimeError):
                ensure_motor_acceleration(robot, 375)
            self.assertEqual(robot.calls, [])

    @staticmethod
    def prepared():
        robot = Robot()
        robot.activate = 'Disabled'
        robot.program = 'Unloaded'
        robot.motion = 'Stopped'
        return robot

    def test_prepared_write_preserves_other_fields(self):
        robot = self.prepared()
        before = copy.deepcopy(robot.data)
        with TemporaryDirectory() as backup:
            result = ensure_motor_acceleration(robot, 375, output=lambda _: None, backup_root=backup)
        self.assertTrue(result['changed'])
        self.assertEqual((robot.activate, robot.program, robot.motion), ('Disabled', 'Unloaded', 'Stopped'))
        self.assertEqual(len(robot.calls), 12)
        self.assertTrue(all(command.startswith('write_motor --id=') for command in robot.calls))
        for row, old in zip(robot.data, before):
            self.assertEqual(row, dict(old, max_acc=375., min_acc=-375.))

    def test_state_change_after_preflight_cannot_trigger_preparation(self):
        robot = self.prepared()
        original = robot.get_state
        calls = 0
        def changed():
            nonlocal calls
            calls += 1
            if calls > 1:
                robot.activate = 'Enabled'
            return original()
        robot.get_state = changed
        with self.assertRaises(RuntimeError):
            ensure_motor_acceleration(robot, 375)
        self.assertEqual(robot.calls, [])

    def test_shared_rpc_rejects_status_and_other_owner_without_write(self):
        service = SharedControl('127.0.0.1')
        service.control = Mock()
        request = dict(op='motor_acceleration', role='inference', session_id='new', acceleration=375)
        with self.assertRaises(PermissionError):
            service.handle(dict(request, status_channel=True))
        self.assertFalse(service.control.request.called)
        service.control.request.return_value = dict(ret_code=0, ret_context=json.dumps(self.prepared().data))
        service.owner = 'other'
        with self.assertRaises(PermissionError):
            service.handle(request)
        service.control.request.assert_called_once_with('read_motor --all', raw=True)

    def test_active_owner_matching_configuration_is_read_only(self):
        service = SharedControl('127.0.0.1')
        service.owner = 'other'
        service.control = Mock()
        service.control.request.return_value = dict(ret_code=0, ret_context=json.dumps(self.prepared().data))
        result = service.handle(dict(op='motor_acceleration', role='teleop', session_id='reader', acceleration=400))
        self.assertFalse(result['changed'])
        self.assertEqual(service.owner, 'other')
        service.control.request.assert_called_once_with('read_motor --all', raw=True)

    def test_old_relay_value_error_has_restart_hint(self):
        from fluxvla.engines.operators.a10_operator import A10Operator
        from fluxvla.engines.communication.a10_shared_connection import SharedFollowerConnection
        robot = A10Operator('127.0.0.1', allow_motion=True, motor_acceleration_deg_s2=375)
        robot._connection = Mock(spec=SharedFollowerConnection)
        robot._connection.call.side_effect = ValueError('Unknown shared operation: motor_acceleration')
        with patch.object(robot, '_require_motion'), self.assertRaisesRegex(RuntimeError, '重新启动'):
            robot.ensure_motor_acceleration()

    def test_raw_follower_start_checks_before_target_ownership(self):
        from fluxvla.engines.operators.a10_operator import A10Operator
        robot = A10Operator('127.0.0.1', allow_motion=True)
        for command in ('follower_aj', 'follower_aj_tg', 'follower_cart'):
            with patch.object(robot, '_require_motion'), patch.object(robot, 'ensure_motor_acceleration',
                    side_effect=RuntimeError('blocked acceleration')) as check, patch.object(robot, '_require_target_owner') as owner:
                with self.assertRaisesRegex(RuntimeError, 'blocked acceleration'):
                    robot.send_controller_command(command)
                check.assert_called_once()
                owner.assert_not_called()

    def test_shared_teleop_can_only_write_prepared_acceleration(self):
        service = SharedControl('127.0.0.1')
        robot = self.prepared()
        def request(command, **kwargs):
            if command == 'get':
                from test_a10_operator import status_reply
                reply = status_reply(active=False, mode='None')
                reply['ret_context'].update(robot.get_state().raw['ret_context'])
                return reply
            if command == 'read_motor --all':
                return dict(ret_code=0, ret_context=json.dumps(robot.data))
            return robot.send_controller_command(command)
        service.control = Mock()
        service.control.request.side_effect = request
        with TemporaryDirectory() as backup:
            from unittest.mock import patch
            import fluxvla.engines.operators.kaanh_commands.motor_acceleration as policy
            original = policy.set_acceleration
            def write(*args, **kwargs):
                kwargs['backup_root'] = backup
                return original(*args, **kwargs)
            with patch.object(policy, 'set_acceleration', side_effect=write):
                result = service.handle(dict(op='motor_acceleration', role='teleop',
                                             session_id='vr', acceleration=1000))
        self.assertTrue(result['changed'])
        self.assertEqual(len(robot.calls), 12)
        self.assertTrue(all(c.startswith('write_motor --id=') for c in robot.calls))


if __name__ == '__main__':
    unittest.main()
