"""Console + dual VR integration using one-client loopback hardware only."""
import copy
import json
import socket
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from mmengine import Config

from fluxvla.engines.communication.a10_shared_connection import ControlTransferredError
from fluxvla.engines.communication.a10_shared_server import SharedControl, SharedControlServer
from fluxvla.engines.operators.a10_operator import A10Operator
from fluxvla.engines.operators.vr.runtime import build_runtime
from scripts.a10_control import A10Console, main, parse_command
from test_a10_operator import FakeController, DUAL_ARMS, dual_motion, status_reply
from test_vr_dual_runtime import HandInput, hand
from test_vr_controller_recording import reply as recording_reply

ROOT = Path(__file__).resolve().parents[1]


def free_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


class SingleClientController(FakeController):
    def __init__(self):
        super().__init__()
        self.current_connections = self.total_connections = self.max_connections = 0

    def handle(self, ws):
        with self.lock:
            self.current_connections += 1
            self.total_connections += 1
            self.max_connections = max(self.max_connections, self.current_connections)
            reject = self.current_connections > 1
        try:
            if reject:
                ws.close(code=1013, reason='One controller connection only')
                return
            super().handle(ws)
        finally:
            with self.lock:
                self.current_connections -= 1


class DualSharedIntegrationTest(unittest.TestCase):
    def test_snapshot_reader_is_fresh_during_stop_and_cannot_own_motion(self):
        robot = self.client('teleop')
        reader = robot.open_feedback_reader()
        self.addCleanup(reader.close)
        robot.start_follower('cartesian')
        stopped_after = time.monotonic()+.35
        def slow_stop_feedback():
            reply = status_reply(active=self.controller.active, mode=self.controller.mode)
            reply['ret_context']['motion_msg'] = dual_motion()
            reply['ret_context']['robot_msg']['motion'] = ('Running' if time.monotonic()<stopped_after else 'Stopped')
            return reply
        self.controller.command_replies['get'] = slow_stop_feedback
        errors = []
        def stop():
            try: robot.stop_follower()
            except Exception as exc: errors.append(exc)
        worker = threading.Thread(target=stop)
        worker.start()
        try:
            stamps = []
            for _ in range(7):
                time.sleep(.03)
                frame = robot.get_feedback_snapshot(reader)
                stamps.append(frame['received_monotonic_s'])
                self.assertEqual(set(frame['arms']), {'left', 'right'})
                self.assertLess(time.monotonic()-frame['received_monotonic_s'], .15)
            self.assertTrue(worker.is_alive())  # Reads did not wait for the stop transaction.
            self.assertGreater(len(set(stamps)), 3)
            with self.assertRaises(PermissionError): reader.send_command('follower_cart')
        finally:
            worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(self.controller.max_connections, 1)
        old = robot.get_feedback_snapshot(reader)['received_monotonic_s']
        time.sleep(.04)
        self.assertEqual(robot.get_feedback_snapshot(reader)['received_monotonic_s'], old)
        self.assertFalse(robot.follower_stop_pending)

    def setUp(self):
        # Fail immediately if a test accidentally inherits a real controller IP.
        connect = socket.socket.connect
        def loopback_only(sock, address):
            if isinstance(address, tuple) and address[0] not in ('127.0.0.1', '::1', 'localhost'):
                raise AssertionError(f'Real network forbidden in tests: {address}')
            return connect(sock, address)
        self.network_guard = patch.object(socket.socket, 'connect', loopback_only)
        self.network_guard.start()
        self.addCleanup(self.network_guard.stop)
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.controller = SingleClientController().__enter__()
        self.controller.motion_override = dual_motion()
        self.addCleanup(self.controller.__exit__, None, None, None)
        ws = self.controller.server.socket.getsockname()[1]
        self.options = dict(host='127.0.0.1', ws_port=ws, status_ws_port=ws,
                            udp_port=self.controller.udp.getsockname()[1],
                            joint_udp_port=self.controller.joint_udp.getsockname()[1],
                            timeout_s=1., allow_motion=True, arms=copy.deepcopy(DUAL_ARMS),
                            cartesian_target_order=['left', 'right'], joint_target_order=['left', 'right'],
                            feedback_position_unit='mm', cartesian_wire_format='pe321')
        self.relay = SharedControl('127.0.0.1', ws, ws, self.options['udp_port'],
                                   self.options['joint_udp_port'], timeout_s=1.)
        self.relay.connect()
        self.addCleanup(self.relay.close)
        self.server = SharedControlServer(('127.0.0.1', 0), self.relay)
        self.thread = threading.Thread(target=self.server.serve_forever, kwargs=dict(poll_interval=.01), daemon=True)
        self.thread.start()
        self.addCleanup(self.stop_server)
        self.seq = 0

    def stop_server(self):
        self.server.shutdown()
        self.thread.join(3)
        self.server.server_close()

    def client(self, role, **overrides):
        options = dict(self.options, shared_control=dict(mode='shared', role=role,
                       host='127.0.0.1', port=self.server.server_address[1]))
        options.update(overrides)
        op = A10Operator(**options)
        self.addCleanup(op.close)
        op.connect()
        return op

    def runtime(self, **overrides):
        cfg = Config.fromfile(str(ROOT / 'configs/a10/a10_dual_arm_vr_teleop.py')).teleop
        cfg.vr_only = False
        cfg.record_controller = False
        cfg.zx_grippers.enabled = False
        cfg.controller_speed_percent = None
        cfg.diagnostics_path = None
        cfg.operator.update(self.options)
        cfg.operator.shared_control = dict(mode='shared', role='teleop', host='127.0.0.1',
                                          port=self.server.server_address[1])
        cfg.update(overrides)
        if cfg.get('record_controller'):
            cfg.diagnostics_path = str(Path(self.temp.name) / 'vr.jsonl')
        runtime = build_runtime(cfg)
        runtime.vr = runtime.teleoperator.input = HandInput()
        self.addCleanup(runtime.close)
        runtime.vr.rows = {side: hand(1) for side in ('left', 'right')}
        runtime.connect()
        return runtime

    def tick(self, runtime, pressed=True):
        self.seq += 1
        runtime.vr.rows = {side: hand(self.seq, pressed, x=self.seq * .00005)
                                for side in ('left', 'right')}
        runtime.step()

    def wait_robot_disconnected(self):
        deadline = time.monotonic() + 2
        while self.controller.current_connections and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertEqual(self.controller.current_connections, 0)

    def test_new_vr_recording_recovers_known_closed_transport_with_console_still_open(self):
        console = self.client('console')
        self.relay.control.close()
        self.wait_robot_disconnected()
        self.controller.command_replies['record_start'] = recording_reply()
        runtime = self.runtime(record_controller=True)
        self.assertTrue(runtime._connected)
        self.assertTrue(console.is_connected)
        self.assertEqual(set(console.get_state().arms), {'left', 'right'})
        self.assertEqual(self.controller.commands.count('record_start'), 1)
        self.assertEqual(set(self.controller.commands), {'get', 'record_start'})
        self.assertEqual(self.controller.max_connections, 1)
        self.assertEqual(self.controller.total_connections, 2)

    def test_new_vr_recording_recovers_remotely_closed_socket_not_yet_used(self):
        self.client('console')
        # Closed WS object remains cached, matching a silent disconnect while idle.
        self.relay.control.ws.close()
        self.wait_robot_disconnected()
        self.assertIsNotNone(self.relay.control.ws)
        self.controller.command_replies['record_start'] = recording_reply()
        runtime = self.runtime(record_controller=True)
        self.assertTrue(runtime._connected)
        self.assertEqual(self.controller.commands.count('record_start'), 1)
        self.assertEqual(set(self.controller.commands), {'get', 'record_start'})
        self.assertEqual(self.controller.max_connections, 1)

    def test_uncertain_record_start_is_not_replayed_and_explicit_retry_can_recover(self):
        console = self.client('console')
        self.controller.command_replies['record_start'] = None  # Close instead of ACK.
        with self.assertRaises(Exception):
            self.runtime(record_controller=True)
        self.wait_robot_disconnected()
        self.assertEqual(self.controller.commands.count('record_start'), 1)
        self.controller.command_replies['record_start'] = recording_reply()
        # A deliberate new startup validates GET and sends exactly one new request.
        runtime = self.runtime(record_controller=True)
        self.assertTrue(runtime._connected)
        self.assertTrue(console.is_connected)
        self.assertEqual(self.controller.commands.count('record_start'), 2)
        self.assertNotIn('follower_cart', self.controller.commands)

    def test_transport_loss_while_owned_does_not_reconnect_or_resume_motion(self):
        console = self.client('console')
        runtime = self.runtime()
        self.tick(runtime, False)
        for _ in range(5): self.tick(runtime)
        self.relay.control.close()
        self.wait_robot_disconnected()
        connections = self.controller.total_connections
        try:
            # Fresh read-only recovery must revoke the old UDP stream, not
            # leave a permanently broken relay or silently resume motion.
            self.assertTrue(console.get_state().follower_active)
            self.assertEqual(self.controller.total_connections, connections + 1)
            self.assertIn(self.relay.owner, self.relay.revoked)
            self.tick(runtime)
            self.assertIsNotNone(runtime.shared_control_pause)
            self.assertTrue(runtime._connected)
            self.assertEqual(self.controller.commands.count('follower_cart'), 1)
        finally:
            console.stop_all_actions()
        self.assertFalse(console.get_state().follower_active)

    def test_console_reconnect_for_stop_does_not_wait_for_long_command_probe(self):
        console = self.client('console')
        entered, release = threading.Event(), threading.Event()
        errors = []
        def wait_reply():
            entered.set()
            release.wait(3)
            return dict(ret_code=0, ret_context='', ret_msg='done')
        self.controller.command_replies['native_wait'] = wait_reply
        def run():
            try: console.send_controller_command('native_wait')
            except Exception as exc: errors.append(exc)
        worker = threading.Thread(target=run)
        worker.start()
        try:
            self.assertTrue(entered.wait(1))
            started = time.monotonic()
            self.client('console')  # Local hello must not queue a robot GET here.
            self.assertLess(time.monotonic() - started, .4)
            self.assertEqual(self.controller.total_connections, 1)
        finally:
            release.set(); worker.join(4)
        self.assertEqual(errors, [])

    def test_queries_and_console_close_do_not_interrupt_live_vr(self):
        console = self.client('console')
        runtime = self.runtime()
        self.tick(runtime, False)
        for _ in range(5): self.tick(runtime)
        for _ in range(5):
            self.tick(runtime)
            self.assertTrue(console.get_state().follower_active)
            self.assertEqual(set(console.get_state().arms), {'left', 'right'})
        packet = self.controller.udp.recv(4096)
        self.assertEqual(len(json.loads(packet[40:])['pe']), 2)
        console.close()
        self.tick(runtime)
        self.assertTrue(runtime.robot.get_state().follower_active)
        self.assertTrue(runtime.last_reference_result['transmitted'])
        self.assertIs(self.relay.control, self.relay.status)
        self.assertEqual(self.controller.total_connections, 1)

    def test_console_takeover_keeps_vr_open_and_reanchors_only_after_release(self):
        runtime = self.runtime()
        console = self.client('console')
        self.tick(runtime, False)
        for _ in range(4): self.tick(runtime)
        console.start_follower('joint')
        starts = self.controller.commands.count('follower_cart')
        for _ in range(3): self.tick(runtime)  # Grip held cannot auto-restart old motion.
        self.assertTrue(runtime._connected)
        self.assertEqual(runtime.last_reference_result['state'], 'console_control')
        self.assertEqual(self.controller.commands.count('follower_cart'), starts)
        self.assertTrue(console.get_state().follower_active)
        console.stop_follower()
        self.tick(runtime)
        self.assertIsNotNone(runtime.shared_control_pause)
        self.tick(runtime, False)
        self.assertIsNone(runtime.shared_control_pause)
        for _ in range(4): self.tick(runtime)
        self.assertGreater(self.controller.commands.count('follower_cart'), starts)
        self.assertTrue(runtime.robot.get_state().follower_active)
        self.assertEqual(self.controller.max_connections, 1)

    def test_unused_stale_hand_does_not_block_release_after_console_takeover(self):
        runtime, console = self.runtime(), self.client('console')
        self.tick(runtime, False)
        for _ in range(5): self.tick(runtime)
        console.start_follower('cartesian')
        self.tick(runtime)
        self.assertEqual(runtime.vr.hud[-1]['phase'], 'console_control')
        console.stop_follower()
        self.seq += 1
        runtime.vr.rows = dict(left=hand(self.seq, age=10.), right=hand(self.seq))
        runtime.step()
        self.assertIsNone(runtime.shared_control_pause)
        self.assertEqual(runtime.vr.hud[-1]['phase'], 'idle')
        self.assertFalse(runtime.robot.follower_stop_pending)

    def test_vr_close_after_takeover_never_stops_console_follower(self):
        runtime = self.runtime()
        console = self.client('console')
        self.tick(runtime, False)
        for _ in range(5): self.tick(runtime)
        console.start_follower('cartesian')
        self.tick(runtime)
        stopped = self.controller.commands.count('stop_follower')
        runtime.close()
        self.assertTrue(console.get_state().follower_active)
        self.assertEqual(self.controller.commands.count('stop_follower'), stopped)
        console.stop_follower()

    def test_record_start_and_heartbeat_do_not_take_control_or_open_extra_socket(self):
        for cmd in ('record_start', 'record_heartbeat', 'record_status'):
            self.controller.command_replies[cmd] = recording_reply()
        runtime = self.runtime(record_controller=True)
        console = self.client('console')
        self.tick(runtime, False)
        for _ in range(5): self.tick(runtime)
        self.assertTrue(runtime.robot.heartbeat_runtime_recording()['recording'])
        self.assertTrue(console.get_state().follower_active)
        self.assertEqual(self.controller.total_connections, 1)
        with self.assertRaises(PermissionError): console.start_runtime_recording()
        self.tick(runtime)
        self.assertTrue(runtime.last_reference_result['transmitted'])

    def test_live_vr_can_open_while_console_owns_follower(self):
        console = self.client('console')
        console.start_follower('cartesian')
        runtime = self.runtime()
        self.tick(runtime)
        self.assertEqual(runtime.last_reference_result['state'], 'console_control')
        console.stop_follower()
        self.tick(runtime, False)
        for _ in range(3): self.tick(runtime)
        self.assertTrue(runtime.last_reference_result['transmitted'])

    def test_racing_old_target_is_rejected_after_console_takes_over(self):
        vr = self.client('teleop')
        console = self.client('console')
        vr.start_follower('cartesian')
        console.start_follower('cartesian')
        with self.assertRaises(ControlTransferredError):
            vr.send_eepose({name: [0, 0, 0, 0, 0, 0, 1] for name in vr.arm_names})
        self.assertTrue(console.get_state().follower_active)
        console.stop_follower()

    def test_concurrent_queries_and_vr_targets_share_one_robot_socket(self):
        runtime, console = self.runtime(), self.client('console')
        self.tick(runtime, False)
        for _ in range(5): self.tick(runtime)
        done = threading.Event()
        errors, queries = [], []
        def read():
            while not done.is_set():
                try: queries.append(console.get_state())
                except Exception as exc: errors.append(exc)
                done.wait(.002)
        worker = threading.Thread(target=read)
        worker.start()
        try:
            for _ in range(30):
                self.tick(runtime)
                time.sleep(.003)
        finally:
            done.set(); worker.join(3)
        self.assertFalse(errors)
        self.assertGreater(len(queries), 5)
        self.assertTrue(runtime.last_reference_result['transmitted'])
        self.assertNotIn('stop_follower', self.controller.commands)
        self.assertEqual(self.controller.max_connections, 1)

    def test_shared_get_timeout_retains_vr_interface_and_never_replays_targets(self):
        self.check_shared_get_timeout(feedback_poll_hz=None)

    def test_background_get_timeout_retains_vr_interface_and_never_replays_targets(self):
        self.check_shared_get_timeout(feedback_poll_hz=30)

    def check_shared_get_timeout(self, *, feedback_poll_hz):
        runtime = self.runtime(feedback_poll_hz=feedback_poll_hz)
        self.tick(runtime, False)
        for _ in range(5): self.tick(runtime)
        read = self.relay.read_state
        fail = [True]
        def once():
            if fail[0]:
                fail[0] = False
                raise TimeoutError('synthetic shared GET timeout')
            return read()
        with patch.object(self.relay, 'read_state', once):
            if feedback_poll_hz is not None:
                deadline = time.monotonic() + 1.
                while self.relay._poll_error is None and time.monotonic() < deadline:
                    time.sleep(.005)
                self.assertIsNotNone(self.relay._poll_error)
                while runtime.robot._feedback_cache[1] is None and time.monotonic() < deadline:
                    time.sleep(.005)
                self.assertIsNotNone(runtime.robot._feedback_cache[1])
            self.tick(runtime)
        self.assertIsNotNone(runtime.communication_pause)
        self.assertTrue(runtime._connected)
        self.assertEqual(runtime.robot.last_communication_failure['command'],
                         'get' if feedback_poll_hz is None else 'feedback_snapshot')
        # Relay disconnect cleanup may still GET/stop asynchronously; what
        # must not continue is the VR client's target/query/start loop.
        with patch.object(runtime.robot, 'get_frame', side_effect=AssertionError('unexpected query')), \
                patch.object(runtime.robot, 'start_follower', side_effect=AssertionError('unexpected restart')), \
                patch.object(runtime.robot, 'send_sdk_tool_pose', side_effect=AssertionError('unexpected replay')):
            for _ in range(4): self.tick(runtime)
        self.assertEqual(runtime.last_reference_result['state'], 'communication_paused')

    def test_reconnect_does_not_reuse_an_old_control_session_identity(self):
        console = self.client('console')
        first = console._connection.session_id
        console.close()
        console.connect()
        self.assertNotEqual(first, console._connection.session_id)
        console.start_follower('cartesian')
        self.relay.disconnected(first)  # Delayed callback from an old RPC socket.
        self.assertTrue(console.get_state().follower_active)
        console.stop_follower()

    def test_console_m_init_reservation_blocks_vr_until_full_restore_finishes(self):
        vr, console = self.client('teleop'), self.client('console')
        vr.start_follower('cartesian')
        console.begin_shared_console_operation()
        try:
            lease = vr.get_shared_control_status()
            self.assertTrue(lease['preempted'])
            self.assertFalse(lease['available'])
            with self.assertRaises(PermissionError): vr.acknowledge_shared_takeover()
            with self.assertRaises(ControlTransferredError): vr.start_follower('cartesian')
        finally:
            console.end_shared_console_operation()
        self.assertTrue(vr.get_shared_control_status()['available'])

    def test_full_console_m_init_restores_cartesian_mode_and_vr_can_reclutch(self):
        runtime = self.runtime()
        console = self.client('console')
        console.init_position = {name: arm.actual_joints_rad.copy()
                                 for name, arm in console.get_state().arms.items()}
        self.tick(runtime, False)
        for _ in range(5): self.tick(runtime)
        self.assertTrue(runtime.robot.get_state().follower_active)
        terminal = A10Console(console, output=lambda _: None)
        terminal.execute(parse_command(['m_init']))
        result = console.last_init_motion
        self.assertTrue(result['completed'])
        self.assertTrue(result['original_control_state']['follower_active'])
        self.assertEqual(result['original_control_state']['follower_mode'], 'Cart')
        self.assertTrue(result['control_state_restored'])
        self.tick(runtime)
        self.assertEqual(runtime.last_reference_result['state'], 'console_control')
        terminal.execute(parse_command(['stop_follower']))
        self.tick(runtime, False)
        for _ in range(5): self.tick(runtime)
        self.assertTrue(runtime.last_reference_result['transmitted'])
        self.assertEqual(self.controller.max_connections, 1)

    def test_dual_cli_accepts_shared_config_and_read_only_does_not_preempt(self):
        runtime = self.runtime()
        self.tick(runtime, False)
        for _ in range(5): self.tick(runtime)
        config = Path(self.temp.name) / 'robot.py'
        opts = dict(self.options, type='A10Operator', shared_control=dict(mode='managed', role='teleop',
                    host='127.0.0.1', port=self.server.server_address[1]))
        config.write_text('operator = ' + repr(opts))
        output = []
        result = main(['--config', str(config), '--read-only', 'get'], output=output.append)
        self.assertEqual(result, 0, output)
        self.assertTrue(runtime.robot.get_state().follower_active)
        self.assertEqual(self.controller.total_connections, 1)


class ManagedStartupTest(unittest.TestCase):
    def test_console_and_vr_open_in_either_order_then_relay_exits_after_last_client(self):
        for first in ('console', 'teleop'):
            with self.subTest(first=first), tempfile.TemporaryDirectory() as directory, SingleClientController() as controller:
                port = free_port()
                ws = controller.server.socket.getsockname()[1]
                shared = dict(mode='managed', host='127.0.0.1', port=port, idle_timeout_s=.4,
                              startup_timeout_s=15., log_path=str(Path(directory) / 'relay.log'))
                def make(role):
                    return A10Operator('127.0.0.1', ws_port=ws, status_ws_port=ws,
                           udp_port=controller.udp.getsockname()[1],
                           joint_udp_port=controller.joint_udp.getsockname()[1], timeout_s=1.,
                           shared_control=dict(shared, role=role), allow_motion=True)
                a, b = make(first), make('teleop' if first == 'console' else 'console')
                try:
                    a.connect()
                    b.connect()
                    self.assertEqual(controller.total_connections, 1)
                    a.close()
                    self.assertIsNotNone(b.get_state().actual_joints_rad)
                    self.assertEqual(controller.max_connections, 1)
                finally:
                    a.close(); b.close()
                deadline = time.monotonic() + 5
                while controller.current_connections and time.monotonic() < deadline:
                    time.sleep(.05)
                self.assertEqual(controller.current_connections, 0)
                self.assertFalse(controller.active)

    def test_simultaneous_start_only_one_process_connects_to_robot(self):
        with tempfile.TemporaryDirectory() as directory, SingleClientController() as controller:
            port = free_port()
            ws = controller.server.socket.getsockname()[1]
            clients = [A10Operator('127.0.0.1', ws_port=ws, status_ws_port=ws, timeout_s=1.,
                       udp_port=controller.udp.getsockname()[1],
                       joint_udp_port=controller.joint_udp.getsockname()[1],
                       shared_control=dict(mode='managed', role=role, host='127.0.0.1', port=port,
                                           idle_timeout_s=.4, log_path=str(Path(directory) / 'relay.log')))
                       for role in ('console', 'teleop')]
            errors = []
            barrier = threading.Barrier(2)
            def launch(client):
                try:
                    barrier.wait(2)
                    client.connect()
                except BaseException as exc: errors.append(exc)
            workers = [threading.Thread(target=launch, args=(client,)) for client in clients]
            try:
                for worker in workers: worker.start()
                for worker in workers: worker.join(20)
                self.assertFalse(any(worker.is_alive() for worker in workers))
                self.assertEqual(errors, [])
                self.assertEqual(controller.total_connections, 1)
                self.assertEqual(controller.max_connections, 1)
            finally:
                for client in clients: client.close()
                deadline = time.monotonic() + 5
                while controller.current_connections and time.monotonic() < deadline: time.sleep(.05)

    def test_offline_robot_fails_without_direct_fallback_or_motion_retry(self):
        with tempfile.TemporaryDirectory() as directory:
            port, ws = free_port(), free_port()
            robot = A10Operator('127.0.0.1', ws_port=ws, status_ws_port=ws, timeout_s=.3,
                shared_control=dict(mode='managed', host='127.0.0.1', port=port, idle_timeout_s=.3,
                                    log_path=str(Path(directory) / 'offline.log')))
            try:
                with self.assertRaises((OSError, RuntimeError)):
                    robot.connect()
                self.assertFalse(robot.is_connected)
                self.assertEqual(robot._connection.__class__.__name__, 'SharedFollowerConnection')
            finally:
                robot.close()


if __name__ == '__main__':
    unittest.main()
