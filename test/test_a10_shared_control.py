"""Shared console/inference ownership on loopback; never contacts a real robot."""
import socket
import threading
import time
import unittest
from unittest.mock import Mock, patch

from fluxvla.engines.operators.a10_operator import A10Operator, _FollowerConnection, A10NotReadyError, A10ControllerError
from fluxvla.engines.communication.a10_shared_connection import SharedFollowerConnection
from fluxvla.engines.communication.a10_shared_server import SharedControl, SharedControlServer, ConsoleSharedControl
from test_a10_operator import FakeController, status_reply
from fluxvla.engines.operators.kaanh_commands.m_init import move_to_init


class SharedControlTest(unittest.TestCase):
    def test_old_relay_byte_count_does_not_fabricate_udp_timestamp(self):
        from fluxvla.engines.communication.a10_shared_connection import _Datagram
        connection = Mock()
        connection.call.return_value = 3
        udp = _Datagram(connection, 'Axis')
        self.assertEqual(udp.send(b'abc'), 3)
        self.assertIsNone(udp.last_send_timing)

    def test_shared_dual_udp_preserves_targets_and_wire_timestamps(self):
        import numpy as np
        from test_a10_operator import DUAL_ARMS, dual_motion
        self.controller.motion_override = dual_motion()
        client = self.client('inference', arms=DUAL_ARMS,
            joint_target_order=['left', 'right'], cartesian_target_order=['left', 'right'])
        client.start_follower()
        try:
            state = client.get_state()
            targets = {name: arm.actual_joints_rad for name, arm in state.arms.items()}
            for _ in range(3):
                started = time.monotonic()
                timing = client.send_joints(targets)
                finished = time.monotonic()
                self.assertLessEqual(started, timing['udp_send_started_s'])
                self.assertLessEqual(timing['udp_send_started_s'], timing['udp_send_finished_s'])
                self.assertLessEqual(timing['udp_send_finished_s'], finished)
            packet = self.controller.joint_udp.recv(4096)[40:].decode()
            values = __import__('json').loads(packet[packet.index('['):])
            np.testing.assert_allclose(values, np.arange(12)*10, atol=1e-10)
        finally:
            client.stop_follower()

    def test_read_motor_config_does_not_claim_or_stop_active_owner(self):
        import json
        owner = self.client('teleop')
        reader = self.client('inference', allow_motion=False)
        owner.start_follower()
        motors = [dict(max_acc=1000., min_acc=-1000., slave_id=i+2) for i in range(12)]
        self.controller.command_replies['read_motor --all'] = dict(
            ret_code=0, ret_context=json.dumps(motors), ret_msg='')
        self.assertEqual(reader.read_motor_config(), motors)
        self.assertEqual(self.robot.owner, owner._connection.session_id)
        self.assertTrue(self.controller.active)
        self.assertNotIn('stop_follower', self.controller.commands)
        owner.stop_follower()

    def setUp(self):
        self.controller = FakeController().__enter__()
        self.ws_port = self.controller.server.socket.getsockname()[1]
        from websockets.sync.server import serve
        self.status_server = serve(self.controller.handle, '127.0.0.1', 0)
        self.status_port = self.status_server.socket.getsockname()[1]
        self.status_thread = threading.Thread(target=self.status_server.serve_forever, daemon=True)
        self.status_thread.start()
        self.robot = SharedControl('127.0.0.1', self.ws_port, self.status_port,
            self.controller.udp.getsockname()[1], self.controller.joint_udp.getsockname()[1], timeout_s=.6)
        self.robot.connect()
        self.server = SharedControlServer(('127.0.0.1', 0), self.robot)
        self.thread = threading.Thread(target=self.server.serve_forever, kwargs=dict(poll_interval=.01), daemon=True)
        self.thread.start()
        self.clients = []

    def tearDown(self):
        for client in reversed(self.clients):
            client.close()
        self.server.shutdown()
        self.thread.join(2)
        self.server.server_close()
        self.robot.close()
        self.status_server.shutdown()
        self.status_thread.join(2)
        self.controller.__exit__(None, None, None)

    def client(self, role, **kwargs):
        options = dict(host='127.0.0.1', ws_port=self.ws_port,
            status_ws_port=self.status_port, udp_port=self.controller.udp.getsockname()[1],
            joint_udp_port=self.controller.joint_udp.getsockname()[1], timeout_s=.6,
            command_mode='joint', allow_motion=True,
            shared_control=dict(host='127.0.0.1', port=self.server.server_address[1], role=role))
        options.update(kwargs)
        client = A10Operator(**options)
        self.clients.append(client)
        client.connect()
        return client

    def test_request_stop_sends_once_without_controller_result_or_state_reads(self):
        client = self.client('teleop')
        client.start_follower()
        self.controller.change_mode = False  # Native follower deliberately remains active.
        with patch.object(self.robot, 'read_state', side_effect=AssertionError('Stop queried state')),              patch.object(client, '_wait_mode', side_effect=AssertionError('Stop waited for mode')):
            result = client.request_stop_follower()
            self.assertEqual(result, dict(requested=True, confirmed=False))
            self.assertEqual(client._connection.call('stop_request'), result)  # No replay on duplicate.
        self.assertIsNone(self.robot.owner)
        self.assertIn(client._connection.session_id, self.robot.revoked)
        self.assertFalse(client.follower_stop_pending)
        self.assertFalse(client.last_stop_confirmed)
        deadline = time.monotonic() + 1.
        while 'stop_follower' not in self.controller.commands and time.monotonic() < deadline:
            time.sleep(.005)
        self.assertEqual(self.controller.commands.count('stop_follower'), 1)
        self.assertTrue(self.controller.active)  # No claim of actual native stop.
        with self.assertRaises(PermissionError):
            self.robot.claim('another-session', 'teleop')

    def test_request_stop_never_stops_another_owner(self):
        owner = self.client('teleop')
        other = self.client('teleop')
        owner.start_follower()
        with self.assertRaises(PermissionError):
            other._connection.call('stop_request')
        self.assertEqual(self.robot.owner, owner._connection.session_id)
        self.assertNotIn('stop_follower', self.controller.commands)
        owner.stop_follower()

    def speed_controller(self):
        values = dict(percent=40)
        def get():
            reply = status_reply(active=self.controller.active, mode=self.controller.mode)
            reply['ret_context']['motion_msg']['jog_vel'] = values['percent']
            return reply
        self.controller.command_replies['get'] = get
        for percent in (25, 40):
            def set_value(percent=percent):
                values['percent'] = percent
                return dict(ret_code=0, ret_context={}, ret_msg='')
            self.controller.command_replies[f'set_jog_vel --vel_percent={percent}'] = set_value
        return values

    def test_shared_speed_change_and_restore_after_owned_stop(self):
        self.speed_controller()
        client = self.client('inference')
        self.assertEqual(client.get_speed_override()['percent'], 40)
        self.assertEqual(client.set_speed_override(25, expected_mode='Manual')['percent'], 25)
        client.start_follower()
        with self.assertRaises(PermissionError):
            client.set_speed_override(40, expected_mode='Manual')
        client.stop_follower()
        self.assertEqual(client.set_speed_override(40, expected_mode='Manual', expected_percent=25)['percent'], 40)
        with self.assertRaises(PermissionError):
            client._connection.request('set_jog_vel --vel_percent=25')

    def test_shared_speed_restore_rejected_after_console_takeover(self):
        self.speed_controller()
        client, console = self.client('inference'), self.client('console')
        client.set_speed_override(25, expected_mode='Manual')
        client.start_follower()
        console.start_follower()
        console.stop_follower()
        with self.assertRaises(PermissionError):
            client.set_speed_override(40, expected_mode='Manual')

    def test_console_queries_while_inference_streams(self):
        console = self.client('console')
        inference = self.client('inference', status_poll_interval_s=.02)
        inference.start_follower()
        for _ in range(3):
            self.assertTrue(console.get_state().follower_active)
            inference.send_joints({'right': [.1] * 6})
            self.assertTrue(self.controller.joint_udp.recv(1024))
        self.assertNotIn('stop_follower', self.controller.commands)
        inference.stop_follower()
        self.assertFalse(console.get_state().follower_active)
        self.assertTrue(console.is_connected)

    def test_local_init_preempts_inference_and_stops_its_own_follower(self):
        inference, console = self.client('inference'), self.client('console')
        inference.start_follower()
        console.init_position = {'right': console.get_state().actual_joints_rad.copy()}
        result = move_to_init(console)
        self.assertTrue(result['completed'])
        commands = [c for c in self.controller.commands if c != 'get']
        self.assertEqual(commands, [
            'start_follower', 'follower_aj', 'stop_follower', 'set_jog_vel --vel_percent=5',
            'start_follower', 'follower_aj', 'stop_follower', 'set_jog_vel --vel_percent=10',
            'start_follower', 'follower_aj'])
        self.assertTrue(result['control_state_restored'])
        with self.assertRaises((InterruptedError, RuntimeError)):
            inference.send_joints({'right': [0.] * 6})

    def test_auto_reuses_service_for_control_and_status_without_extra_robot_connections(self):
        from unittest.mock import patch
        console = self.client('console')
        options = dict(mode='auto', host='127.0.0.1', port=self.server.server_address[1], role='inference')
        with patch.object(_FollowerConnection, 'connect', side_effect=AssertionError('extra robot connection')):
            inference = self.client('inference', shared_control=options, status_poll_interval_s=.02)
            self.assertIsInstance(inference._connection, SharedFollowerConnection)
            self.assertIsInstance(inference._status_connection, SharedFollowerConnection)
            inference.start_follower()
            self.assertTrue(console.get_state().follower_active)
            inference.send_joints({'right': [.1] * 6})
            self.assertTrue(self.controller.joint_udp.recv(1024))
            inference.stop_follower()

    def test_auto_goes_direct_when_service_absent_and_rechecks_on_reconnect(self):
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 0))
            address = probe.getsockname()
        options = dict(mode='auto', host=address[0], port=address[1], role='inference')
        inference = self.client('inference', shared_control=options, status_poll_interval_s=.02)
        self.assertIsInstance(inference._connection, _FollowerConnection)
        self.assertIsInstance(inference._status_connection, _FollowerConnection)
        self.assertIsNone(inference.shared_control)
        inference.start_follower()
        inference.send_joints({'right': [.1] * 6})
        self.assertTrue(self.controller.joint_udp.recv(1024))
        inference.stop_follower()
        inference.close()
        service = SharedControlServer(address, self.robot)
        thread = threading.Thread(target=service.serve_forever, kwargs=dict(poll_interval=.01), daemon=True)
        thread.start()
        try:
            inference.connect()
            self.assertIsInstance(inference._connection, SharedFollowerConnection)
            self.assertIsInstance(inference._status_connection, SharedFollowerConnection)
            self.assertIsNotNone(inference.get_state().actual_joints_rad)
        finally:
            inference.close()
            service.shutdown()
            thread.join(2)
            service.server_close()

    def test_auto_does_not_fallback_when_service_robot_endpoints_disagree(self):
        from unittest.mock import patch
        options = dict(mode='auto', host='127.0.0.1', port=self.server.server_address[1], role='inference')
        with patch.object(_FollowerConnection, 'connect', side_effect=AssertionError('unsafe fallback')):
            with self.assertRaisesRegex(RuntimeError, 'endpoints differ'):
                self.client('inference', ws_port=self.ws_port + 1, shared_control=options)

    def test_auto_start_reuses_existing_service_without_robot_connections(self):
        from unittest.mock import patch
        service = ConsoleSharedControl(self.server.server_address, host='127.0.0.1')
        with patch.object(SharedControl, 'connect', side_effect=AssertionError('second connection')):
            service.start()
        self.assertIsNone(service.server)
        service.close()
        console = self.client('console')
        self.assertIsNotNone(console.get_state().actual_joints_rad)

    def test_default_console_autostarts_service_and_inference_uses_it(self):
        from scripts.a10_control import main
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 0))
            port = probe.getsockname()[1]
        out = []
        lines = iter(['get', 'exit'])
        def read(_):
            line = next(lines)
            if line == 'get':
                inference = A10Operator(
                    host='127.0.0.1', ws_port=self.ws_port, status_ws_port=self.status_port,
                    udp_port=self.robot.udp_port, joint_udp_port=self.robot.joint_udp_port,
                    allow_motion=True, command_mode='joint', timeout_s=.6,
                    shared_control=dict(host='127.0.0.1', port=port, role='inference'))
                try:
                    inference.connect()
                    inference.start_follower()
                    inference.send_joints({'right': [.1] * 6})
                    self.assertTrue(self.controller.joint_udp.recv(1024))
                    inference.stop_follower()
                finally:
                    inference.close()
            return line
        result = main(['--host', '127.0.0.1', '--ws-port', str(self.ws_port),
                       '--status-ws-port', str(self.status_port), '--shared-port', str(port),
                       '--udp-port', str(self.robot.udp_port),
                       '--joint-udp-port', str(self.robot.joint_udp_port), 'shell'],
                      input_fn=read, output=out.append)
        self.assertEqual(result, 0, out)
        self.assertTrue(any('共享连接已自动启动' in line for line in out), out)
        with self.assertRaises(ConnectionRefusedError):
            socket.create_connection(('127.0.0.1', port), .2)

    def test_auto_start_failure_frees_port_for_explicit_reconnect(self):
        from unittest.mock import patch
        service = ConsoleSharedControl(('127.0.0.1', 0), output=lambda _: None,
            host='127.0.0.1', ws_port=self.ws_port, status_ws_port=self.status_port,
            udp_port=self.robot.udp_port, joint_udp_port=self.robot.joint_udp_port)
        with patch.object(SharedControl, 'connect', side_effect=ConnectionRefusedError('offline')):
            with self.assertRaises(ConnectionRefusedError):
                service.start()
        self.assertIsNone(service.server)
        service.start()
        self.assertTrue(service.thread.is_alive())
        service.close()
        self.assertFalse(service.thread.is_alive())

    def test_60hz_joint_stream_with_concurrent_console_queries(self):
        import numpy as np
        console = self.client('console')
        inference = self.client('inference', record_trajectory_trace=True, status_poll_interval_s=.02)
        inference.start_follower()
        done = threading.Event()
        failures = []
        def queries():
            while not done.is_set():
                try: console.get_state()
                except Exception as exc: failures.append(exc)
                done.wait(.01)
        query_thread = threading.Thread(target=queries)
        query_thread.start()
        try:
            inference.execute_trajectory({'right': np.full((3, 6), .1)}, dt=1 / 30, joint_target_repeat=2)
        finally:
            done.set()
            query_thread.join(2)
        self.assertFalse(failures)
        for _ in range(6):
            self.assertTrue(self.controller.joint_udp.recv(1024))
        trace = inference.last_trajectory['trace']
        gaps = np.diff([row['joint_send_started_s'] for row in trace])
        self.assertTrue(np.all(gaps > .01), gaps)
        self.assertLess(float(np.median(gaps)), .025)
        self.assertEqual(inference.last_trajectory['joint_packets_sent'], 6)
        self.assertNotIn('stop_follower', self.controller.commands)
        inference.stop_follower()

    def test_console_command_stops_inference_before_execution(self):
        inference, console = self.client('inference'), self.client('console')
        self.controller.command_replies['m_init'] = {'ret_code': 0, 'ret_context': '', 'ret_msg': 'done'}
        inference.start_follower()
        reply = console.send_controller_command('m_init')
        self.assertEqual(reply['ret_msg'], 'done')
        commands = self.controller.commands
        self.assertLess(commands.index('stop_follower'), commands.index('m_init'))
        with self.assertRaises(InterruptedError):
            inference.send_joints({'right': [.1] * 6})
        with self.assertRaises(socket.timeout):
            self.controller.joint_udp.recv(1024)
        inference.stop_follower()
        self.assertTrue(inference.last_stop_confirmed)
        self.assertEqual(commands.count('stop_follower'), 1)

    def test_old_inference_cleanup_does_not_stop_console_follower(self):
        inference, console = self.client('inference'), self.client('console')
        inference.start_follower()
        console.start_follower()
        self.assertTrue(self.controller.active)
        self.assertEqual(self.controller.commands.count('stop_follower'), 1)
        inference.stop_follower()
        inference.close()
        self.assertTrue(self.controller.active)
        self.assertTrue(inference.last_stop_confirmed)
        self.assertEqual(self.controller.commands.count('stop_follower'), 1)
        console.stop_follower()
        self.assertFalse(self.controller.active)

    def test_console_can_stop_inference_without_owning_its_start(self):
        inference, console = self.client('inference'), self.client('console')
        inference.start_follower()
        self.assertFalse(console.stop_follower().follower_active)
        with self.assertRaises(InterruptedError):
            inference.send_joints({'right': [.1] * 6})
        inference.stop_follower()
        self.assertEqual(self.controller.commands.count('stop_follower'), 1)

    def test_read_only_console_does_not_preempt(self):
        inference, console = self.client('inference'), self.client('console', allow_motion=False)
        inference.start_follower()
        with self.assertRaises(PermissionError):
            console.send_controller_command('m_init')
        self.assertTrue(self.controller.active)
        self.assertTrue(console.get_state().follower_active)
        inference.stop_follower()

    def test_builtin_console_commands_stop_inference_before_local_checks(self):
        inference, console = self.client('inference'), self.client('console')
        inference.start_follower()
        console.set_jog_coordinate('Tool')
        self.assertFalse(self.controller.active)
        commands = self.controller.commands
        self.assertLess(commands.index('stop_follower'), commands.index('set_jog_coordinate --tool'))
        inference.stop_follower()
        inference.start_follower()
        console.enable()
        self.assertFalse(self.controller.active)
        self.assertEqual(commands.count('stop_follower'), 2)
        inference.stop_follower()

    def test_console_can_stop_new_inference_after_its_own_previous_run(self):
        inference, console = self.client('inference'), self.client('console')
        console.start_follower()
        console.stop_follower()
        inference.start_follower()
        self.assertFalse(console.stop_follower().follower_active)
        inference.stop_follower()
        self.assertEqual(self.controller.commands.count('stop_follower'), 2)

    def test_queries_and_revocation_do_not_wait_for_long_console_reply(self):
        inference, console = self.client('inference'), self.client('console')
        entered, release = threading.Event(), threading.Event()
        def reply():
            entered.set()
            release.wait(2)
            return {'ret_code': 0, 'ret_context': ''}
        self.controller.command_replies['m_init'] = reply
        inference.start_follower()
        results = []
        def command():
            try: results.append(console.send_controller_command('m_init'))
            except Exception as exc: results.append(exc)
        worker = threading.Thread(target=command)
        worker.start()
        try:
            self.assertTrue(entered.wait(1))
            before = time.monotonic()
            self.assertFalse(inference.get_state().follower_active)
            with self.assertRaises((InterruptedError, RuntimeError)):
                inference.send_joints({'right': [.1] * 6})
            inference.stop_follower()
            self.assertLess(time.monotonic() - before, .4)
        finally:
            release.set()
            worker.join(3)
        self.assertEqual(results[0]['ret_code'], 0)

    def test_client_disconnect_stops_owned_follower(self):
        inference = self.client('inference')
        inference.start_follower()
        inference.close()
        deadline = time.monotonic() + 1.
        while self.controller.active and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertFalse(self.controller.active)

    def test_inference_cannot_take_over_console_and_service_refuses_unmanaged_follower(self):
        console, inference = self.client('console'), self.client('inference')
        console.start_follower()
        with self.assertRaises(PermissionError):
            inference.start_follower()
        self.assertTrue(self.controller.active)
        console.stop_follower()
        self.controller.active = True
        with self.assertRaises(PermissionError):
            inference.start_follower()
        self.assertTrue(self.controller.active)
        self.controller.active = False


    def test_recording_poll_error_is_visible_but_does_not_revoke_command_only_owner(self):
        import time
        client = self.client('teleop')
        client._connection.shared['require_feedback'] = False
        client.start_follower()
        session = client._connection.session_id
        self.robot.configure_feedback_poll(session, 30.)
        with patch.object(self.robot, 'read_state', side_effect=TimeoutError('recording GET failed')):
            deadline = time.monotonic() + 1.
            while self.robot._poll_error is None and time.monotonic() < deadline:
                time.sleep(.005)
            self.assertIsNotNone(self.robot._poll_error)
            self.assertNotIn(session, self.robot.revoked)
            self.assertEqual(self.robot.owner, session)
            with self.assertRaisesRegex(TimeoutError, 'recording GET failed'):
                client._connection.call('feedback_snapshot')
            client.send_joints({'right': [.1]*6})
            self.assertTrue(self.controller.joint_udp.recv(1024))
        client.stop_follower()


    def test_regular_owner_still_requires_healthy_feedback(self):
        client = self.client('teleop')
        client.start_follower()
        with self.robot._poll_lock:
            self.robot._poll_error = TimeoutError('regular GET failed')
        with self.assertRaisesRegex(TimeoutError, 'regular GET failed'):
            client.send_joints({'right': [.1]*6})
        client.stop_follower()



if __name__ == '__main__':
    unittest.main()


class ToolCoordinateTest(unittest.TestCase):
    def test_already_tool_skips_coordinate_command_even_while_native_stop_is_pending(self):
        with FakeController(initial_active=True) as controller:
            controller.motion = 'Running'
            operator = controller.operator()
            operator.ensure_tool_coordinate()
            self.assertTrue(all(c == 'get' for c in controller.commands))
            self.assertFalse(operator.follower_stop_pending)


    def test_real_coordinate_change_keeps_stopped_and_fault_checks(self):
        with FakeController() as controller:
            controller.coordinate = 'Axis'
            controller.motion = 'Running'
            operator = controller.operator()
            with self.assertRaises(A10NotReadyError):
                operator.ensure_tool_coordinate()
            self.assertTrue(all(c == 'get' for c in controller.commands))
            controller.motion = 'Stopped'
            operator.ensure_tool_coordinate()
            self.assertEqual(controller.coordinate, 'Tool')
            controller.driver_fault = True
            with self.assertRaises(A10ControllerError):
                operator.ensure_tool_coordinate()
