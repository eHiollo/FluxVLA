"""Offline protocol, ownership and completion tests; loopback endpoints only."""
import json
import socket
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np

from fluxvla.engines.communication.a10_timed_aj import (
    ROW_TO_PHYSICAL, TimedAjTransport, encode_target, require_stream,
    timed_config, validate_status)
from fluxvla.engines.communication.a10_shared_server import SharedControl, SharedControlServer
from fluxvla.engines.operators.a10_operator import A10Operator
from fluxvla.engines.operators.a10_dual_operator import A10DualOperator
from test_a10_dual_inference import Clock, SETTINGS, b, live_device_fixture
from test_a10_operator import DUAL_ARMS, FakeController, status_reply, wire_reply


def taj(**overrides):
    return dict(dict(ret_code=0, implementation='joint_tracking.TimedAj', input_source='udp',
                     stream_id=7, active=True, ready=True, accepting=True, state=2, reason=0,
                     stop_requested=False, received=0, consumed=0, queue_depth=0,
                     model_s=0., parameters={}, parameter_policy='fixed_A',
                     motion_limits=dict(source='fixed_A'), gap_policy='hold_and_resume',
                     waiting_for_input=False, hold_transition=False, end_received=False,
                     final_reference_reached=False, pause_s=0., hold_count=0), **overrides)


class ProtocolTests(unittest.TestCase):
    def test_plain_json_absolute_radians_and_axis_mapping(self):
        row = np.arange(14, dtype=float) * .01
        packet = encode_target(9, 3, row[ROW_TO_PHYSICAL], True)
        expected = ('{"stream_id":9,"index":3,"position_rad":' +
                    json.dumps(row[ROW_TO_PHYSICAL].tolist(), separators=(',', ':')) +
                    ',"final":true}').encode()
        self.assertEqual(packet, expected)
        self.assertNotIn(b'\0', packet)
        self.assertEqual(json.loads(packet)['position_rad'],
                         [row[j] for j in [7, 8, 9, 10, 11, 12, 0, 1, 2, 3, 4, 5]])

    def test_invalid_wire_inputs(self):
        for sid, index, values, final in (
                (True, 0, [0.]*12, False), (0, 0, [0.]*12, False),
                (2**64, 0, [0.]*12, False), (1, -1, [0.]*12, False),
                (1, True, [0.]*12, False), (1, 0, [0.]*14, False),
                (1, 0, [float('nan')]*12, False), (1, 0, ['0']*12, False),
                (1, 0, [False]*12, False), (1, 0, [0.]*12, 1)):
            with self.subTest(sid=sid, index=index, values=values, final=final):
                with self.assertRaises(ValueError):
                    encode_target(sid, index, values, final)

    def test_status_is_top_level_and_zero_queue_is_normal(self):
        self.assertEqual(require_stream(taj(queue_depth=0), 7)['reason'], 0)
        # final accepted is expected while draining/holding.
        require_stream(taj(accepting=False, state=3), 7)
        for bad in (dict(ret_code=0, ret_context=taj()), taj(implementation='other'),
                    taj(active=1), taj(stream_id=True), taj(state=7), taj(reason=9),
                    taj(parameter_policy='tunable'), taj(motion_limits=dict(source='xml')),
                    taj(gap_policy='abort'), taj(final_reference_reached=1)):
            with self.assertRaises((ValueError, RuntimeError)):
                validate_status(bad)

    def test_interruption_reasons_foreign_stream_and_final_send_gate(self):
        for bad in [taj(reason=i) for i in range(1, 9)] + [taj(stream_id=8), taj(active=False),
                taj(stop_requested=True), taj(state=4), taj(state=5), taj(state=6),
                taj(ready=False), taj(accepting=False)]:
            with self.subTest(status=bad), self.assertRaises(RuntimeError):
                require_stream(bad, 7, accepting=True)

    def test_network_and_configuration_guards(self):
        transport = TimedAjTransport(Mock(), Mock(), '127.0.0.1', 5999, 5999, .1)
        with self.assertRaisesRegex(RuntimeError, 'loopback'):
            transport.start({})
        with self.assertRaisesRegex(ValueError, 'dedicated'):
            transport.start(dict(network_ready=True, plan_ws_port=5999))
        for cfg in (dict(omega=30), dict(udp_port=True), dict(network_ready=1)):
            with self.assertRaises(ValueError):
                timed_config(cfg)

    def test_send_has_no_status_queries_automatically_numbers_and_rejects_after_final(self):
        t = TimedAjTransport(Mock(), Mock(), 'localhost', 5999, 5999, .1)
        t.stream_id, t.started_s, t.udp = 7, time.monotonic(), Mock()
        t.last_status = taj()
        t.udp.send.side_effect = len
        t.send([0.]*12)
        t.send([0.]*12, final=True)
        packets = [json.loads(c.args[0]) for c in t.udp.send.call_args_list]
        self.assertEqual([p['index'] for p in packets], [0, 1])
        self.assertEqual([p['stream_id'] for p in packets], [7, 7])
        with self.assertRaises(RuntimeError):
            t.send([0.]*12)
        self.assertEqual(t.udp.send.call_count, 2)
        t.command.assert_not_called()

    def test_first_target_can_wait_for_a_long_inference(self):
        t = TimedAjTransport(Mock(), Mock(), 'localhost', 5999, 5999, .1)
        t.stream_id, t.started_s, t.udp = 7, time.monotonic()-3600., Mock()
        t.last_status = taj(state=0)
        t.udp.send.side_effect = len
        t.send([0.]*12)
        self.assertEqual(t.index, 1)
        t.command.assert_not_called()

    def test_stop_ack_does_not_mean_completed_or_stop_foreign_stream(self):
        command = Mock(return_value=dict(ret_code=0))
        state = Mock(return_value=dict(ret_context=dict(robot_msg=dict(motion='Running'))))
        t = TimedAjTransport(command, state, 'localhost', 5999, 5999, .025)
        t.stream_id = 7
        t.status = Mock(return_value=taj())
        with self.assertRaisesRegex(TimeoutError, 'ACK'):
            t.stop()
        command.assert_called_once_with('taj_stop', raw=True)
        command.reset_mock()
        t.status.return_value = taj(stream_id=8)
        with self.assertRaisesRegex(RuntimeError, 'foreign'):
            t.stop()
        command.assert_not_called()


class LiveAdapterTests(unittest.TestCase):
    def device(self):
        d, state = live_device_fixture()
        d.cfg = dict(bridge=dict(SETTINGS, timed_aj=dict(enabled=True, network_ready=True)))
        state.raw['ret_context']['robot_msg'].update(op_mode='Manual', status='Normal')
        # A custom Plan need not activate the native follower.
        state.follower_active, state.follower_mode = False, None
        for arm in state.arms.values():
            arm.target_joints_rad = arm.actual_joints_rad.copy()
        d.robot.read_motor_config.return_value = [dict(max_acc=250., min_acc=-250.)]*12
        d.robot.start_timed_aj.return_value = taj()
        d.robot.timed_aj_status.return_value = taj()
        d.robot.stop_timed_aj.return_value = dict(active=False)
        return d, state

    def test_mapping_single_send_final_and_no_legacy_writes(self):
        d, state = self.device()
        d.start()
        row = np.r_[state.arms['right'].actual_joints_rad, .25,
                    state.arms['left'].actual_joints_rad, .75]
        row[:6] += .001
        d.robot.reset_mock()
        d.send(row, final=True)
        args = d.robot.send_timed_joint_target.call_args
        self.assertEqual(len(args.args), 1)
        np.testing.assert_array_equal(args.args[0], row[ROW_TO_PHYSICAL])
        self.assertIs(args.kwargs['final'], True)
        self.assertTrue(d.stop())
        d.robot.get_state.assert_not_called()
        d.robot.timed_aj_status.assert_not_called()
        for name in ('start_follower', 'send_joints', 'stop_follower',
                     'ensure_motor_acceleration', 'set_speed_override'):
            getattr(d.robot, name).assert_not_called()

    def test_fixed_A_does_not_require_or_rewrite_motor_acceleration_250(self):
        d, _ = self.device()
        d.robot.read_motor_config.return_value[0] = dict(max_acc=375., min_acc=-375.)
        d.start()
        d.robot.start_timed_aj.assert_called_once()
        d.robot.read_motor_config.assert_not_called()
        d.robot.ensure_motor_acceleration.assert_not_called()

    def test_initial_command_continuity_and_fault_guard(self):
        d, state = self.device()
        d.start()
        row = np.r_[state.arms['right'].actual_joints_rad, .25,
                    state.arms['left'].actual_joints_rad, .75]
        row[0] += .036
        with self.assertRaisesRegex(RuntimeError, '0.035'):
            d.send(row)
        d.robot.send_timed_joint_target.assert_not_called()
        d.robot.timed_aj_status.return_value = taj(reason=2, state=4)
        with self.assertRaisesRegex(RuntimeError, 'interrupted'):
            d.check()

    def test_final_received_and_consumed_are_not_actual_arrival(self):
        d, _ = self.device()
        d.start()
        d._timed_index, d._timed_final = 20, True
        d.robot.timed_aj_status.return_value = taj(received=20, consumed=20, state=2, accepting=False)
        self.assertFalse(d.arrival_feedback()['reference_finished'])
        d.robot.timed_aj_status.return_value['state'] = 3
        self.assertFalse(d.arrival_feedback()['reference_finished'])  # state=3 may be temporary hold
        d.robot.timed_aj_status.return_value['end_received'] = True
        self.assertFalse(d.arrival_feedback()['reference_finished'])
        d.robot.timed_aj_status.return_value['final_reference_reached'] = True
        self.assertTrue(d.arrival_feedback()['reference_finished'])
        d.robot.timed_aj_status.return_value['stream_id'] = 8
        with self.assertRaises(RuntimeError):
            d.arrival_feedback()


class TimedBlockDevice:
    """Constructed trajectories and idealized feedback, never a tracking simulation."""
    def __init__(self, clock):
        self.clock, self.packets, self.stops = clock, [], []
        self.active, self.stream_id = False, 0
        self.command_never_arrives = self.actual_never_arrives = False
        self.robot = Mock()
        self.robot.read_motor_config.return_value = [dict(min_pos=-180., max_pos=180.)]*12

    def start(self):
        self.active = True
        self.stream_id += 1
        self.index = 0

    def check(self):
        raise AssertionError('No per-point check in the timed sending loop')

    def send(self, row, *, final=False):
        self.packets.append((self.stream_id, self.index, self.clock.now, row.copy(), final))
        self.index += 1
        self.last = row.copy()
        self.last_send = self.clock.now
        return dict(udp_send_started_s=self.clock.now, udp_send_finished_s=self.clock.now)

    def arrival_feedback(self):
        age = self.clock.now - self.last_send
        q = self.last[[0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12]]
        command = q + (.02 if age < .25 or self.command_never_arrives else 0)
        actual = q + (.02 if age < .35 or self.actual_never_arrives else 0)
        return dict(joints=actual, command_joints=command, stamp=self.clock.now,
                    has_error=False, follower_active=False, follower_mode=None,
                    activation='Enabled', motion='Running' if self.active else 'Stopped',
                    operation_mode='Manual', robot_status='Normal',
                    reference_finished=age >= .15,
                    timed_status=taj(stream_id=self.stream_id, state=3 if age >= .15 else 2,
                                     consumed=self.index, active=self.active))

    def stop(self):
        self.stops.append(self.clock.now)
        self.active = False
        return True


class BlockLifecycleTests(unittest.TestCase):
    def operator(self):
        clock = Clock()
        d = TimedBlockDevice(clock)
        settings = dict(SETTINGS, timed_aj=dict(enabled=True), follower_tail=dict(
            enabled=True, arrival_timeout_s=1., poll_hz=50.,
            position_tolerance_deg=.5, speed_tolerance_deg_s=.5, stable_duration_s=.04))
        op = A10DualOperator(mode='motion', devices=d, clock=clock, settings=settings,
            config=dict(operator=dict(arms=DUAL_ARMS, joint_target_order=['left', 'right'])))
        op.last_frame = dict(qpos=np.zeros(14))
        return clock, d, op

    def test_real_model_selected_block_and_constructed_1000_points_send_once(self):
        real = np.load(Path(__file__).parent / 'data/a10_model_action_sample/actions_executed.npy')
        self.assertEqual(real.shape, (20, 14))
        clock, d, op = self.operator()
        op._execute_rows(real, 1/30)
        np.testing.assert_array_equal([p[3] for p in d.packets], real)
        self.assertEqual(d.stops, [])
        # Separate constructed input: bounded reversal; not continuous model output.
        clock, d, op = self.operator()
        constructed = np.zeros((1000, 14))
        constructed[:, 0] = .02*np.sin(np.arange(1000)/30)
        for i in range(50):
            op._execute_rows(constructed[i*20:(i+1)*20], 1/30)
        self.assertEqual(len(d.packets), 1000)
        self.assertEqual([p[1] for p in d.packets], list(range(1000)))
        self.assertEqual({p[0] for p in d.packets}, {1})
        self.assertTrue(all(not p[4] for p in d.packets))
        np.testing.assert_allclose([p[2]-d.packets[0][2] for p in d.packets], np.arange(1000)/30,
                                   atol=1e-12)
        self.assertAlmostEqual(clock.now-d.packets[0][2], 1000/30)
        self.assertEqual(d.stops, [])

    def test_continuous_blocks_one_stream_no_block_wait_or_stop(self):
        clock, d, op = self.operator()
        rows = np.zeros((20, 14)); rows[:, 0] = np.arange(20)*.001
        op._execute_rows(rows, 1/30)
        self.assertFalse(op._arrival_pending)
        self.assertEqual(d.stops, [])
        op.stream_final = True
        op._execute_rows(rows, 1/30)
        self.assertEqual(len(d.packets), 40)
        for chunk in (d.packets[:20], d.packets[20:]):
            np.testing.assert_allclose([p[2]-chunk[0][2] for p in chunk], np.arange(20)/30)
            np.testing.assert_array_equal([p[3] for p in chunk], rows)
        self.assertEqual([p[1] for p in d.packets], list(range(40)))
        self.assertEqual([p[4] for p in d.packets], [False]*39 + [True])
        self.assertEqual(d.packets[20][0], d.packets[0][0])
        self.assertEqual(d.stops, [])
        op.finish_timed_stream()
        self.assertGreaterEqual(d.stops[0]-d.packets[39][2], .39)
        self.assertFalse(op._arrival_pending)

    def test_reference_hold_and_command_convergence_cannot_replace_actual_arrival(self):
        for field in ('command_never_arrives', 'actual_never_arrives'):
            _, d, op = self.operator()
            setattr(d, field, True)
            op.stream_final = True
            op._execute_rows(np.zeros((20, 14)), 1/30)
            with self.subTest(field=field), self.assertRaises(TimeoutError):
                op.finish_timed_stream()
            self.assertTrue(op._arrival_pending)
            self.assertEqual(d.stops, [])  # Completion path never stops on final receipt alone.

    def test_inference_gap_holds_without_new_packets_and_next_block_resumes_at_30hz(self):
        clock, d, op = self.operator()
        op._execute_rows(np.zeros((20, 14)), 1/30)
        first_end = clock.now
        clock.sleep(7.)  # A long model request needs no repeated/holding targets.
        self.assertEqual(len(d.packets), 20)
        op._execute_rows(np.zeros((20, 14)), 1/30)
        self.assertEqual(len(d.packets), 40)
        self.assertAlmostEqual(d.packets[20][2]-first_end, 7.)
        self.assertEqual([p[1] for p in d.packets], list(range(40)))
        np.testing.assert_allclose([p[2]-d.packets[20][2] for p in d.packets[20:]], np.arange(20)/30)
        self.assertEqual(d.stops, [])

    def test_within_block_deadline_failure_does_not_catch_up(self):
        clock, d, op = self.operator()
        send = d.send
        def delayed(row, **kwargs):
            result = send(row, **kwargs)
            clock.sleep(.2)
            return result
        d.send = delayed
        with self.assertRaisesRegex(RuntimeError, 'deadline'):
            op._execute_rows(np.zeros((20, 14)), 1/30)
        self.assertEqual(len(d.packets), 1)


class SharedTimedLoopbackTests(unittest.TestCase):
    def setUp(self):
        from websockets.sync.server import serve
        self.controller = FakeController().__enter__()
        self.current = taj(active=False, ready=False, accepting=False, state=5)
        self.done = threading.Event()
        self.stop_changes_state = True
        self.plan_commands = []
        self.controller.command_replies['taj_status'] = lambda: dict(self.current)
        self.controller.command_replies['taj_stop'] = self.stop_reply
        def get():
            reply = status_reply()
            reply['ret_context']['robot_msg'].update(status='Normal', motion=self.controller.motion)
            return reply
        self.controller.command_replies['get'] = get
        self.plan_server = serve(self.plan, '127.0.0.1', 0)
        self.plan_thread = threading.Thread(target=self.plan_server.serve_forever, daemon=True)
        self.plan_thread.start()
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.bind(('127.0.0.1', 0)); self.udp.settimeout(.1)
        ws_port = self.controller.server.socket.getsockname()[1]
        self.service = SharedControl('127.0.0.1', ws_port, ws_port,
            self.controller.udp.getsockname()[1], self.controller.joint_udp.getsockname()[1], .2)
        self.service.connect()
        self.server = SharedControlServer(('127.0.0.1', 0), self.service)
        self.thread = threading.Thread(target=self.server.serve_forever,
                                        kwargs=dict(poll_interval=.01), daemon=True)
        self.thread.start()
        self.clients = []

    def tearDown(self):
        self.stop_changes_state = True
        for c in self.clients:
            c.close()
        self.server.shutdown(); self.thread.join(1); self.server.server_close()
        self.service.close()
        self.done.set(); self.plan_server.shutdown(); self.plan_thread.join(1)
        self.udp.close()
        self.controller.__exit__(None, None, None)

    def client(self, role='inference'):
        c = A10Operator('127.0.0.1', ws_port=self.service.ws_port,
            status_ws_port=self.service.ws_port, udp_port=self.service.udp_port,
            joint_udp_port=self.service.joint_udp_port, timeout_s=.3, allow_motion=True,
            shared_control=dict(host='127.0.0.1', port=self.server.server_address[1], role=role))
        c.connect(); self.clients.append(c)
        return c

    def start(self, c):
        return c.start_timed_aj(dict(enabled=True, network_ready=True,
            plan_ws_port=self.plan_server.socket.getsockname()[1], udp_port=self.udp.getsockname()[1]))

    def plan(self, ws):
        try:
            packet = ws.recv()
            command = packet[40:].decode().rstrip('\0')
            self.plan_commands.append(command)
            assert command == 'timed_aj'
            sid = self.current['stream_id'] + getattr(self, 'id_increment', 1)
            self.done.clear()
            self.current.update(stream_id=sid, active=True, ready=True, accepting=True,
                                state=0, reason=0, stop_requested=False, received=0, consumed=0,
                                end_received=False, final_reference_reached=False)
            self.controller.motion = 'Running'
            self.done.wait(3)
            ws.send(wire_reply(dict(ret_code=0, ret_context='')))
        except Exception:
            pass  # Production closes this dedicated socket only after stop.

    def stop_reply(self):
        if self.stop_changes_state:
            self.current.update(active=False, accepting=False, state=5, reason=1)
            self.controller.motion = 'Stopped'
            self.done.set()
        return dict(ret_code=0, ret_context='')

    def test_async_plan_plain_udp_and_new_stream_without_legacy_commands(self):
        c = self.client()
        started = self.start(c)
        self.assertEqual(started['stream_id'], 8)
        for i in range(3):
            c.timed_aj_status()
            with patch.object(self.service.control, 'request', side_effect=AssertionError('No per-point WS checks')):
                c.send_timed_joint_target([i*.001]*12, final=i == 2)
            packet = json.loads(self.udp.recv(4096))
            self.assertEqual(packet, dict(stream_id=8, index=i, position_rad=[i*.001]*12, final=i == 2))
        self.assertEqual(c.get_state().follower_active, False)
        c.stop_timed_aj()
        self.assertIsNone(self.service.owner)
        self.assertIsNone(self.service.timed.stream_id)
        self.assertIsNone(self.service.timed.last_status)
        self.assertEqual(self.service.timed.index, 0)
        self.assertIsNone(self.service.timed.udp)
        self.assertEqual(self.start(c)['stream_id'], 9)
        c.stop_timed_aj()
        self.assertEqual(self.plan_commands, ['timed_aj', 'timed_aj'])
        self.assertFalse(any(x.startswith(('start_follower', 'follower_aj', 'stop_follower',
            'set_jog_vel', 'write_motor')) for x in self.controller.commands))

    def test_fault_revokes_targets_and_confirmed_stop_allows_restart(self):
        c = self.client(); self.start(c)
        self.current.update(reason=2, state=4)
        c.timed_aj_status()
        with self.assertRaisesRegex(RuntimeError, 'interrupted'):
            c.send_timed_joint_target([0.]*12)
        self.assertIn(c._connection.session_id, self.service.revoked)
        with self.assertRaises(socket.timeout):
            self.udp.recv(4096)
        c.stop_timed_aj()
        self.assertEqual(self.start(c)['stream_id'], 9)
        c.stop_timed_aj()

    def test_controller_allocates_nonconsecutive_ids_and_temporary_hold_accepts_resume(self):
        self.id_increment = 17
        c = self.client()
        self.assertEqual(self.start(c)['stream_id'], 24)
        c.send_timed_joint_target([.001]*12)
        self.assertEqual(json.loads(self.udp.recv(4096))['index'], 0)
        self.current.update(state=3, waiting_for_input=True, hold_transition=False,
                            final_reference_reached=False, pause_s=7., hold_count=1)
        c.timed_aj_status()
        c.send_timed_joint_target([.002]*12, final=True)
        packet = json.loads(self.udp.recv(4096))
        self.assertEqual((packet['stream_id'], packet['index']), (24, 1))
        c.stop_timed_aj()
        self.assertEqual(self.start(c)['stream_id'], 41)
        c.send_timed_joint_target([.003]*12, final=True)
        packet = json.loads(self.udp.recv(4096))
        self.assertEqual((packet['stream_id'], packet['index']), (41, 0))
        c.stop_timed_aj()

    def test_console_preemption_old_cleanup_cannot_stop_new_owner(self):
        a, console = self.client(), self.client('console')
        self.start(a)
        self.start(console)
        count = self.controller.commands.count('taj_stop')
        a.stop_timed_aj()
        self.assertEqual(self.controller.commands.count('taj_stop'), count)
        with self.assertRaises(InterruptedError):
            a.send_timed_joint_target([0.]*12)
        self.assertEqual(self.service.owner, console._connection.session_id)
        console.stop_timed_aj()

    def test_stop_timeout_retains_lease_and_blocks_other_client(self):
        c, other = self.client(), self.client()
        self.start(c)
        self.stop_changes_state = False
        with self.assertRaisesRegex(TimeoutError, 'ACK'):
            c.stop_timed_aj()
        self.assertEqual(self.service.owner, c._connection.session_id)
        with self.assertRaises(PermissionError):
            self.start(other)
        self.stop_changes_state = True
        c.stop_timed_aj()

    def test_disconnect_stops_custom_plan_and_releases_lease(self):
        c = self.client(); self.start(c)
        c.close()
        deadline = time.monotonic()+1
        while self.service.owner is not None and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertIsNone(self.service.owner)
        self.assertIn('taj_stop', self.controller.commands)


if __name__ == '__main__':
    unittest.main()
