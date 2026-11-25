"""Local console initialization over the loopback A10 follower only."""

import copy
from contextlib import ExitStack
import json
from pathlib import Path
import runpy
import socket
from tempfile import TemporaryDirectory
import threading
import unittest
from unittest.mock import patch

import numpy as np

from scripts.a10_control import A10Console, CommandError, main, parse_command
from fluxvla.engines.communication.a10_shared_server import SharedControl, SharedControlServer
from fluxvla.engines.operators.a10_operator import A10Operator, A10ControllerError
from test_a10_operator import FakeController, dual_motion, status_reply
from test_a10_set_acc import motors
from fluxvla.engines.operators.kaanh_commands.m_init import move_to_init, InitMotionError


RIGHT_FIRST_ARMS = {
    # Deliberately insert left first: insertion order must not select the wire order.
    'left': dict(ee_id=1, joint_group=1, tcp_index=1),
    'right': dict(ee_id=0, joint_group=0, tcp_index=0),
}


class A10InitMotionTest(unittest.TestCase):
    def test_robot_init_position_right_id_zero_and_instance_independence(self):
        root = Path(__file__).resolve().parents[1]
        dual = runpy.run_path(str(root / 'configs/a10/a10_dual_arm_operator.py'))['operator']
        single = runpy.run_path(str(root / 'configs/a10/a10_single_operator.py'))['operator']
        a = A10Operator('127.0.0.1', arms=RIGHT_FIRST_ARMS,
                        cartesian_target_order=['right', 'left'], init_position=dual['init_position'])
        b = A10Operator('127.0.0.1', init_position=single['init_position'])
        self.assertEqual(a.joint_target_order, ('right', 'left'))
        np.testing.assert_allclose(np.rad2deg(a.init_position['right']), [-18, -36, 128, -18, -106, -5])
        np.testing.assert_allclose(np.rad2deg(a.init_position['left']), [21, 38, -131, 28, 111, -3])
        a.init_position['right'][0] = 0.
        self.assertNotEqual(b.init_position['right'][0], 0.)
        config = runpy.run_path(str(Path(__file__).resolve().parents[1] /
                                'configs/a10/a10_dual_arm_operator.py'))['operator']
        self.assertEqual(config['arms']['right']['ee_id'], 1)
        self.assertEqual(config['arms']['left']['ee_id'], 0)

    def test_missing_init_position_has_no_implicit_target_or_motion(self):
        with FakeController() as c:
            op = c.operator()
            self.assertIsNone(op.init_position)
            with self.assertRaisesRegex(ValueError, 'operator.init_position'):
                A10Console(op, lambda _: None).execute(parse_command(['m_init']))
            self.assertEqual(c.commands, [])

    def test_dual_joint_packet_contains_full_group_in_explicit_order(self):
        for order in (['right', 'left'], ['left', 'right']):
            with FakeController() as c:
                c.motion_override = dual_motion()
                op = c.operator(arms=RIGHT_FIRST_ARMS, cartesian_target_order=['right', 'left'],
                                joint_target_order=order)
                op.start_follower(mode='joint')
                targets = {'left': np.deg2rad([6, 7, 8, 9, 10, 11]),
                           'right': np.deg2rad([0, 1, 2, 3, 4, 5])}
                op.send_joints(targets)
                packet = c.joint_udp.recv(4096)
                self.assertEqual(int.from_bytes(packet[:4], 'little'), len(packet) - 40)
                self.assertEqual(packet[4:40], bytes(36))
                values = json.loads(packet[40:].decode().split('=', 1)[1])
                np.testing.assert_allclose(values, np.rad2deg(np.concatenate([targets[n] for n in order])), atol=1e-12)
                for bad in ({'right': [0] * 6}, {'right': [0] * 7, 'left': [0] * 6},
                            {'right': [np.nan] * 6, 'left': [0] * 6}):
                    with self.assertRaises(ValueError):
                        op.send_joints(bad)
                with self.assertRaises(socket.timeout):
                    c.joint_udp.recv(4096)
                op.stop_follower()

    def test_local_console_returns_after_feedback_arrival_and_stop(self):
        for dual in (False, True):
            with self.subTest(dual=dual), FakeController() as c:
                settings = {}
                if dual:
                    c.motion_override = dual_motion()
                    settings.update(arms=RIGHT_FIRST_ARMS, cartesian_target_order=['right', 'left'])
                op = c.operator(**settings)
                target = {n: arm.actual_joints_rad.copy() for n, arm in op.get_state().arms.items()}
                op.init_position = target
                output = []
                console = A10Console(op, output.append)
                self.assertTrue(console.execute(parse_command(['m_init'])))
                self.assertTrue(op.last_init_motion['completed'])
                self.assertIn('夹爪初始化尚未执行', '\n'.join(output))
                self.assertEqual([x for x in c.commands if x != 'get'], [
                    'set_jog_vel --vel_percent=5', 'start_follower', 'follower_aj',
                    'stop_follower', 'set_jog_vel --vel_percent=10'])
                self.assertTrue(op.last_init_motion['speed_restored'])
                self.assertEqual(op.last_init_motion['original_speed']['percent'], 10)
                self.assertFalse(c.active)
                packet = c.joint_udp.recv(4096)
                values = json.loads(packet[40:].decode().split('=', 1)[1])
                self.assertEqual(len(values), 12 if dual else 6)

    def test_nonzero_dual_init_position_over_udp_with_independent_feedback(self):
        with FakeController() as c:
            c.motion_override = dual_motion()
            op = c.operator(arms=RIGHT_FIRST_ARMS, cartesian_target_order=['right', 'left'],
                            joint_follower_filter=runpy.run_path(str(
                                Path(__file__).resolve().parents[1] /
                                'configs/a10/a10_dual_arm_operator.py'))['operator']['joint_follower_filter'])
            initial = {n: arm.actual_joints_rad.copy() for n, arm in op.get_state().arms.items()}
            target = {n: q + (.03 if n == 'right' else -.02) for n, q in initial.items()}
            op.init_position = target
            packets, errors, done = [], [], threading.Event()
            def follower():
                while not done.is_set():
                    try:
                        packet = c.joint_udp.recv(4096)
                    except socket.timeout:
                        continue
                    try:
                        self.assertEqual(int.from_bytes(packet[:4], 'little'), len(packet) - 40)
                        data = json.loads(packet[40:])
                        self.assertIs(data['filter'], False)
                        values = data['motor_pos']
                        self.assertEqual(len(values), 12)
                        packets.append(values)
                        c.motion_override['actual_pos'] = [values[:6], values[6:]]
                    except BaseException as exc:
                        errors.append(exc)
                        return
            worker = threading.Thread(target=follower, daemon=True)
            worker.start()
            try:
                output = []
                A10Console(op, output.append).execute(parse_command(['m_init']))
            finally:
                done.set()
                worker.join(1)
            self.assertEqual(errors, [])
            self.assertGreater(len(packets), 3)
            np.testing.assert_allclose(packets[0], np.rad2deg(np.r_[target['right'], target['left']]))
            np.testing.assert_allclose(packets[-1], np.rad2deg(np.r_[target['right'], target['left']]))
            self.assertTrue(op.last_init_motion['completed'])
            self.assertIn('机器人定义的初始位置', '\n'.join(output))
            self.assertFalse(c.active)

    def test_dual_console_init_keeps_375_through_joint_start_and_cart_restore(self):
        profile = Path(__file__).resolve().parents[1] / 'configs/a10/a10_dual_arm_operator.py'
        with FakeController(initial_active=True) as c:
            c.motion_override = dual_motion()
            c.command_replies['read_motor --all'] = dict(ret_code=0, ret_context=[
                dict(motion_type='rotatory', max_acc=375., min_acc=-375.) for _ in range(12)])

            def start_joint():
                c.active, c.mode = True, 'Axis'
                return dict(ret_code=0, ret_context='')

            c.command_replies['follower_aj --vel_percent=100'] = start_joint

            def factory(**settings):
                self.assertEqual(settings['motor_acceleration_deg_s2'], 1000.)
                self.assertEqual(settings['init_motion']['motor_acceleration_deg_s2'], 375.)
                settings.pop('init_grippers_config')
                op = c.operator(**settings)
                op.init_position = {name: arm.actual_joints_rad.copy()
                                    for name, arm in op.get_state().arms.items()}
                return op

            output = []
            self.assertEqual(main(['--direct', '--allow-motion', '--config', str(profile),
                                   '--host', '127.0.0.1', '--ws-port', str(c.server.socket.getsockname()[1]),
                                   '--joint-udp-port', str(c.joint_udp.getsockname()[1]), 'm_init'],
                                  operator_factory=factory, output=output.append), 0, '\n'.join(output))
            self.assertTrue(c.operators[-1].last_init_motion['completed'])
            self.assertEqual(c.commands.count('read_motor --all'), 4)
            self.assertIn('set_jog_vel --vel_percent=5', c.commands)
            self.assertIn('follower_cart', c.commands)
            self.assertFalse(any(cmd.startswith('write_motor') for cmd in c.commands))

    def test_console_init_temporarily_switches_and_restores_mixed_acceleration(self):
        for outcome in ('complete', 'failed', 'cancelled', 'shared'):
            with (self.subTest(outcome=outcome), FakeController() as c,
                  TemporaryDirectory() as directory, ExitStack() as resources):
                snapshot = [dict(row, max_acc=400. + i, min_acc=-500. - i)
                            for i, row in enumerate(motors())]
                current = copy.deepcopy(snapshot)
                temporary = [dict(row, max_acc=375., min_acc=-375.) for row in snapshot]
                writes = []

                def get_status():
                    reply = status_reply(active=c.active, mode=c.mode)
                    ctx = reply['ret_context']
                    ctx['robot_msg'].update(activate=c.activation, motion=c.motion)
                    ctx['motion_msg'].update(dual_motion(), jog_coordinate=c.coordinate, jog_vel=c.jog_vel)
                    ctx['program_msg'] = dict(state='Unloaded', vel=c.pgm_vel)
                    ctx['user_management_msg'] = dict(user='Manufacturer')
                    return reply

                def write(index, row):
                    if c.activation != 'Disabled' or c.active or c.motion != 'Stopped':
                        return dict(ret_code=-1, ret_context='incorrect controller state')
                    writes.append(index)
                    current[index] = copy.deepcopy(row)
                    return dict(ret_code=0, ret_context='')

                def start_joint():
                    if current != temporary or c.activation != 'Enabled' or c.jog_vel != 5:
                        return dict(ret_code=-1, ret_context='incorrect initialization profile')
                    c.active, c.mode = True, 'Axis'
                    return dict(ret_code=0, ret_context='')

                c.command_replies.update({
                    'get': get_status,
                    'read_motor --all': lambda: dict(ret_code=0, ret_context=copy.deepcopy(current)),
                    'follower_aj --vel_percent=100': start_joint,
                })
                for rows in (snapshot, temporary):
                    for index, row in enumerate(rows):
                        payload = json.dumps(row, ensure_ascii=False, separators=(',', ':'))
                        c.command_replies[f'write_motor --id={index} --data={payload}'] = (
                            lambda i=index, value=row: write(i, value))
                options = {}
                if outcome == 'shared':
                    ws_port = c.server.socket.getsockname()[1]
                    relay = SharedControl('127.0.0.1', ws_port, ws_port, c.udp.getsockname()[1],
                                          c.joint_udp.getsockname()[1], timeout_s=.5)
                    relay.connect()
                    resources.callback(relay.close)
                    server = SharedControlServer(('127.0.0.1', 0), relay)
                    thread = threading.Thread(target=server.serve_forever,
                                              kwargs={'poll_interval': .01}, daemon=True)
                    thread.start()
                    resources.callback(server.server_close)
                    resources.callback(thread.join, 2)
                    resources.callback(server.shutdown)
                    options.update(status_ws_port=ws_port, shared_control=dict(
                        role='console', port=server.server_address[1]))
                op = c.operator(arms=RIGHT_FIRST_ARMS, cartesian_target_order=['right', 'left'],
                                init_motion={'motor_acceleration_deg_s2': 375.},
                                motor_acceleration_deg_s2=1000., **options)
                resources.callback(op.close)
                op.init_position = {name: arm.actual_joints_rad.copy()
                                    for name, arm in op.get_state().arms.items()}
                console = A10Console(op, lambda _: None)
                op.begin_console_command()

                def motion_error(*args, **kwargs):
                    if outcome == 'cancelled':
                        op.cancel_current_command()
                        raise InterruptedError('simulated operator cancellation')
                    raise RuntimeError('simulated motion failure')

                with patch('fluxvla.engines.operators.kaanh_commands.m_init_acceleration._BACKUP_ROOT',
                           Path(directory)):
                    if outcome in ('complete', 'shared'):
                        console.execute(parse_command(['m_init']))
                    else:
                        with patch.object(op, 'send_joints', side_effect=motion_error):
                            with self.assertRaisesRegex((RuntimeError, InterruptedError), 'simulated'):
                                console.execute(parse_command(['m_init']))
                self.assertEqual(writes, list(range(12)) * 2)
                self.assertEqual(current, snapshot)
                self.assertFalse(c.active)
                self.assertEqual(c.activation, 'Disabled' if outcome == 'cancelled' else 'Enabled')
                self.assertIn('set_jog_vel --vel_percent=5', c.commands)
                self.assertEqual(json.loads(next(Path(directory).glob('*/after.json')).read_text()), snapshot)

    def test_local_one_shot_and_raw_escape(self):
        with FakeController() as c:
            def factory(**_):
                op = c.operator()
                op.init_position = {'right': op.get_state().actual_joints_rad.copy()}
                return op
            self.assertEqual(main(['--direct', 'm_init'], operator_factory=factory,
                                  output=lambda _: None), 0)
            self.assertNotIn('m_init', c.commands)
            self.assertFalse(c.active)
        with FakeController() as c:
            c.command_replies['m_init'] = {'ret_code': 0, 'ret_context': 'native'}
            op = c.operator()
            console = A10Console(op, lambda _: None)
            console.execute(parse_command(['raw', 'm_init']))
            self.assertIn('m_init', c.commands)
            self.assertNotIn('follower_aj', c.commands)

    def test_local_failure_and_invalid_arguments_never_fall_back(self):
        for tokens in (['m_init', '--mode=1'], ['m_init', '--preset=0'], ['m_init', '1']):
            with self.assertRaises(CommandError):
                parse_command(tokens)
        for options, expected in (({'allow_motion': False}, PermissionError),
                                  ({}, ValueError)):
            with FakeController() as c:
                op = c.operator(init_position={'right': [0.] * 6}, **options)
                if not options:
                    op.init_position = {'right': [0.] * 7}
                console = A10Console(op, lambda _: None)
                with self.assertRaises(expected):
                    console.execute(parse_command(['m_init']))
                self.assertEqual(c.commands, [])

    def test_fault_interrupt_or_stop_failure_preserves_cleanup_obligation(self):
        for fault in ('fault', 'interrupt', 'stop'):
            with self.subTest(fault=fault), FakeController() as c:
                op = c.operator()
                op.init_position = {'right': op.get_state().actual_joints_rad.copy()}
                original = op.send_joints
                def send(targets):
                    original(targets)
                    if fault == 'fault':
                        c.driver_fault = True
                    elif fault == 'interrupt':
                        raise KeyboardInterrupt
                if fault == 'stop':
                    c.command_replies['stop_follower'] = {'ret_code': 0, 'ret_context': ''}
                with patch.object(op, 'send_joints', side_effect=send):
                    expected = (KeyboardInterrupt if fault == 'interrupt' else
                                TimeoutError if fault == 'stop' else A10ControllerError)
                    with self.assertRaises(expected):
                        move_to_init(op)
                self.assertIn('stop_follower', c.commands)
                self.assertNotIn('m_init', c.commands)
                self.assertFalse(op.last_init_motion['completed'])
                self.assertEqual(op.follower_stop_pending, fault == 'stop')
                self.assertIsNone(op._trajectory_owner)

    def test_missing_feedback_or_busy_trajectory_does_not_start(self):
        for problem in ('feedback', 'busy'):
            with FakeController() as c:
                op = c.operator(init_position={'right': [0.] * 6})
                if problem == 'feedback':
                    c.actual = False
                else:
                    op._trajectory_lock.acquire()
                try:
                    with self.assertRaises((ValueError, RuntimeError)):
                        move_to_init(op)
                finally:
                    if problem == 'busy':
                        op._trajectory_lock.release()
                self.assertNotIn('follower_aj', c.commands)
                self.assertNotIn('m_init', c.commands)
                self.assertFalse(op._init_motion_lock.locked())

    def test_disconnect_retains_stop_obligation_without_replaying_init(self):
        with FakeController() as c:
            op = c.operator()
            op.init_position = {'right': op.get_state().actual_joints_rad.copy()}
            original = op.send_joints
            def send(targets):
                original(targets)
                c.fault = 'disconnect'
            with patch.object(op, 'send_joints', side_effect=send):
                with self.assertRaises(InitMotionError) as caught:
                    move_to_init(op)
                self.assertIsNotNone(caught.exception.primary_error)
                self.assertIsNotNone(caught.exception.stop_error)
            self.assertTrue(op.follower_stop_pending)
            self.assertFalse(op.is_connected)
            self.assertFalse(op.last_init_motion['completed'])
            self.assertEqual(c.commands.count('follower_aj'), 1)
            self.assertNotIn('m_init', c.commands)
            self.assertIsNone(op._trajectory_owner)

    def test_own_cartesian_session_stops_before_joint_initialization(self):
        with FakeController() as c:
            op = c.operator()
            op.start_follower(mode='cartesian')
            op.init_position = {'right': op.get_state().actual_joints_rad.copy()}
            move_to_init(op)
            self.assertEqual([x for x in c.commands if x != 'get'],
                             ['follower_cart', 'stop_follower', 'set_jog_vel --vel_percent=5',
                              'start_follower', 'follower_aj', 'stop_follower',
                              'set_jog_vel --vel_percent=10', 'follower_cart'])
            self.assertTrue(c.active)
            self.assertEqual(op.last_init_motion['follower_mode'], 'Cart')


if __name__ == '__main__':
    unittest.main()
