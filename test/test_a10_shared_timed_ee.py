"""EE relay ownership and operator tests; no hardware or network connections."""
import base64
import threading
import unittest
from unittest.mock import Mock

from fluxvla.engines.communication.a10_shared_connection import (
    ControlTransferredError, SharedFollowerConnection)
from fluxvla.engines.communication.a10_shared_server import SharedControl, SharedControlServer
from fluxvla.engines.operators.a10_operator import A10Operator
from test_a10_operator import status_reply


POSES = [[.3, -.1, .5, 0., 0., 0., 1.], [.3, .1, .5, 0., 0., 0., 1.]]


class SharedTimedEeTests(unittest.TestCase):
    def setUp(self):
        self.service = SharedControl('127.0.0.1')
        self.service.read_state = Mock(return_value=status_reply())
        self.transport = self.service.timed_ee = Mock()
        self.transport.stream_id = 17
        self.transport.start_attempted = False
        self.transport.start.return_value = dict(stream_id=17, command_pose_m_quat=POSES)
        self.service.timed = Mock()

    def request(self, op, sid='one', **kwargs):
        return self.service.handle(dict(op=op, role='inference', session_id=sid, **kwargs))

    def start(self):
        return self.request('timed_ee_start', options={})

    def test_start_target_final_and_finish_preserve_one_owned_stream(self):
        self.start()
        self.assertEqual(self.service.owner, 'one')
        self.assertEqual(self.service.timed_ee_owner, 'one')
        self.request('timed_ee_target', stream_id=17, pose_m_quat=POSES, final=False)
        self.request('timed_ee_target', stream_id=17, pose_m_quat=POSES, final=True)
        self.transport.send.assert_any_call(POSES, False)
        self.transport.send.assert_any_call(POSES, True)
        self.transport.status.assert_not_called()
        self.request('timed_ee_finish', stream_id=17, finish_timeout_s=2.)
        self.transport.finish.assert_called_once_with(timeout_s=2.)
        self.transport.stop.assert_not_called()
        self.assertEqual(self.service.owner, 'one')
        self.request('timed_ee_stop', stream_id=17)
        self.transport.stop.assert_called_once_with()
        self.assertIsNone(self.service.owner)
        self.assertIsNone(self.service.timed_ee_owner)
        self.service.timed.assert_not_called()

    def test_targets_finish_stop_require_exact_session_and_stream_id(self):
        self.start()
        for op in ('timed_ee_target', 'timed_ee_finish', 'timed_ee_stop'):
            for sid, stream_id in (('other', 17), ('one', 16), ('one', True), ('one', None)):
                with self.subTest(op=op, sid=sid, stream_id=stream_id):
                    with self.assertRaises(ControlTransferredError):
                        self.request(op, sid=sid, stream_id=stream_id,
                                     pose_m_quat=POSES, final=False)
        self.transport.send.assert_not_called()
        self.transport.finish.assert_not_called()
        self.transport.stop.assert_not_called()
        self.assertEqual(self.service.owner, 'one')

    def test_ee_joint_and_follower_leases_are_mutually_exclusive(self):
        self.start()
        for op, kwargs in (
                ('timed_start', dict(options={})),
                ('request', dict(command='follower_cart')),
                ('request', dict(command='follower_aj')),
                ('lease', dict(operation='acknowledge')),
                ('target', dict(mode='Cart', packet=base64.b64encode(b'x').decode()))):
            with self.subTest(op=op), self.assertRaises((PermissionError, RuntimeError)):
                self.request(op, **kwargs)
        with self.assertRaises(PermissionError):
            self.request('timed_ee_start', sid='other', options={})
        self.service.timed.start.assert_not_called()

    def test_joint_only_status_does_not_query_ee(self):
        self.request('timed_status')
        self.service.timed.status.assert_called_once_with()
        self.transport.status.assert_not_called()

    def test_status_never_adopts_another_stream(self):
        self.request('timed_ee_status', sid='observer', status_channel=True)
        self.transport.status.assert_called_once_with()
        self.assertIsNone(self.service.owner)
        self.assertIsNone(self.service.timed_ee_owner)

    def test_raw_status_reads_without_validation_or_ownership(self):
        self.service.control.request = Mock(return_value=dict(ret_code=0, active=False, stream_id=0))
        self.assertEqual(self.request('timed_ee_status', raw=True)['stream_id'], 0)
        self.service.control.request.assert_called_once_with('tee_status', raw=True)
        self.transport.status.assert_not_called()
        self.assertIsNone(self.service.owner)

    def test_status_only_cannot_start_send_finish_or_stop(self):
        for op in ('timed_ee_start', 'timed_ee_target', 'timed_ee_finish', 'timed_ee_stop'):
            with self.subTest(op=op), self.assertRaises(PermissionError):
                self.request(op, status_channel=True, options={}, stream_id=17)
        self.transport.start.assert_not_called()

    def test_uncertain_stop_retains_lease_but_revokes_targets(self):
        self.start()
        self.transport.stop.side_effect = TimeoutError('not confirmed')
        with self.assertRaises(TimeoutError):
            self.request('timed_ee_stop', stream_id=17)
        self.assertEqual(self.service.owner, 'one')
        self.assertEqual(self.service.timed_ee_owner, 'one')
        with self.assertRaises(ControlTransferredError):
            self.request('timed_ee_target', stream_id=17, pose_m_quat=POSES, final=False)
        self.transport.stop.side_effect = None
        self.request('timed_ee_stop', stream_id=17)
        self.assertIsNone(self.service.owner)

    def test_failure_before_start_releases_lease_but_uncertain_start_retains_it(self):
        self.transport.start.side_effect = RuntimeError('start failed')
        for attempted in (False, True):
            self.transport.start_attempted = attempted
            with self.assertRaises(RuntimeError) as error:
                self.start()
            self.assertEqual(self.service.owner, 'one' if attempted else None)
            self.assertEqual(self.service.timed_ee_owner, 'one' if attempted else None)
            self.assertEqual(error.exception.timed_ee_start_state['start_attempted'], attempted)
            self.assertEqual(error.exception.timed_ee_start_state['lease_released'], not attempted)

    def test_console_stop_and_disconnect_stop_only_ee_transport(self):
        self.start()
        self.service.disconnected('one')
        self.transport.stop.assert_called_once_with()
        self.service.timed.stop.assert_not_called()
        self.start()
        self.service.handle(dict(op='stop', role='console', session_id='console'))
        self.assertEqual(self.transport.stop.call_count, 2)
        self.assertIsNone(self.service.owner)

    def test_finish_does_not_hold_locks_and_cannot_succeed_after_takeover(self):
        self.start()
        def finish(**kwargs):
            acquired = []
            def other_thread():
                with self.service._control_lock, self.service._target_lock:
                    acquired.append(True)
            thread = threading.Thread(target=other_thread)
            thread.start(); thread.join(.5)
            self.assertEqual(acquired, [True])
            self.service.stop_owner('console_stop')
            return dict(finished=True)
        self.transport.finish.side_effect = finish
        with self.assertRaises(ControlTransferredError):
            self.request('timed_ee_finish', stream_id=17)


class SharedEeRpcLoopbackTests(unittest.TestCase):
    def test_binary_wire_values_and_stream_id_cross_the_actual_local_rpc(self):
        # Only this temporary loopback server is opened; robot transports remain
        # mocked and no controller connection is established.
        service = SharedControl('127.0.0.1')
        service.read_state = Mock(return_value=status_reply())
        service._probe_control = Mock(return_value=status_reply())
        service.timed_ee = Mock()
        service.timed_ee.stream_id = 23
        service.timed_ee.start.return_value = dict(stream_id=23, command_pose_m_quat=POSES)
        service.timed_ee.send.return_value = dict(stream_id=23, index=0, final=True)
        server = SharedControlServer(('127.0.0.1', 0), service)
        thread = threading.Thread(target=server.serve_forever,
                                  kwargs=dict(poll_interval=.01), daemon=True)
        thread.start()
        client = SharedFollowerConnection('127.0.0.1', 5999, 9998, .5,
            shared=dict(host='127.0.0.1', port=server.server_address[1], role='inference'))
        try:
            client.connect()
            start = client.call('timed_ee_start', options={})
            self.assertEqual(start['command_pose_m_quat'], POSES)
            sent = client.call('timed_ee_target', stream_id=start['stream_id'],
                               pose_m_quat=POSES, final=True)
            self.assertEqual(sent, dict(stream_id=23, index=0, final=True))
            service.timed_ee.send.assert_called_once_with(POSES, True)
            with self.assertRaises(ControlTransferredError):
                client.call('timed_ee_stop', stream_id=24)
            client.call('timed_ee_stop', stream_id=23)
            self.assertIsNone(service.owner)
            service.timed_ee.start.side_effect = RuntimeError('preflight rejected before Plan')
            service.timed_ee.start_attempted = False
            service.timed_ee.stream_id = None
            with self.assertRaisesRegex(RuntimeError, 'preflight rejected') as error:
                client.call('timed_ee_start', options={})
            self.assertEqual(error.exception.timed_ee_start_state,
                dict(start_attempted=False, stream_id=None, lease_released=True))
            self.assertIsNone(service.owner)
        finally:
            client.close()
            server.shutdown(); thread.join(1.); server.server_close()


class OperatorTimedEeTests(unittest.TestCase):
    def operator(self):
        op = A10Operator('127.0.0.1', allow_motion=True,
            shared_control=dict(host='127.0.0.1', port=5870))
        op._connection = Mock(spec=SharedFollowerConnection)
        op._connection.ws = object()
        op._connection.udp = object()
        op._connection.joint_udp = object()
        op._connection.call.return_value = dict(stream_id=19, command_pose_m_quat=POSES)
        return op

    def test_operator_keeps_private_id_and_sends_full_bimanual_pose(self):
        op = self.operator()
        op.last_stop_confirmed = True
        op.start_timed_ee({})
        self.assertFalse(op.last_stop_confirmed)
        op.send_timed_ee_target(POSES, final=True)
        op._connection.call.assert_called_with('timed_ee_target', stream_id=19,
            pose_m_quat=POSES, final=True)
        op.finish_timed_ee(timeout_s=2.)
        op._connection.call.assert_called_with('timed_ee_finish', stream_id=19,
            finish_timeout_s=2., timeout_s=max(op.timeout_s, 2.) + 1.)
        op.stop_timed_ee()
        op._connection.call.assert_called_with('timed_ee_stop', allow_cancelled=True, stream_id=19)
        self.assertIsNone(op._own_timed_ee_stream_id)
        self.assertTrue(op.last_stop_confirmed)

    def test_uncertain_stop_is_not_reported_as_confirmed(self):
        op = self.operator()
        op._own_timed_ee_stream_id = 19
        op.last_stop_confirmed = True
        op._connection.call.side_effect = TimeoutError("stop not confirmed")
        with self.assertRaises(TimeoutError):
            op.stop_timed_ee()
        self.assertFalse(op.last_stop_confirmed)
        self.assertEqual(op._own_timed_ee_stream_id, 19)

    def test_close_clears_old_stream_identity(self):
        op = self.operator()
        op._own_timed_ee_stream_id = 19
        op.close()
        self.assertIsNone(op._own_timed_ee_stream_id)

    def test_invalid_pose_rejected_before_rpc(self):
        op = self.operator()
        for pose in (POSES[:1], [[0.]*7, [0.]*7]):
            with self.assertRaises(ValueError):
                op.send_timed_ee_target(pose)
        op._connection.call.assert_not_called()

    def test_cancel_blocks_new_target_but_own_stop_remains_available(self):
        op = self.operator()
        op._own_timed_ee_stream_id = 19
        op._command_cancelled.set()
        with self.assertRaises(InterruptedError):
            op.send_timed_ee_target(POSES)
        op.stop_timed_ee()
        op._connection.call.assert_called_once_with(
            'timed_ee_stop', allow_cancelled=True, stream_id=19)

    def test_rejected_start_does_not_send_stop_without_owned_task(self):
        op = self.operator()
        error = RuntimeError('preflight status mismatch')
        error.timed_ee_start_state = dict(start_attempted=False, stream_id=None, lease_released=True)
        op._connection.call.side_effect = error
        with self.assertRaisesRegex(RuntimeError, 'preflight'):
            op.start_timed_ee({})
        self.assertFalse(op.timed_ee_start_attempted)
        self.assertTrue(op.stop_timed_ee()['no_task_started'])
        self.assertEqual(op._connection.call.call_count, 1)

    def test_uncertain_start_keeps_known_id_or_refuses_unidentified_stop(self):
        for stream_id in (None, 23):
            op = self.operator(); error = TimeoutError('start uncertain')
            error.timed_ee_start_state = dict(start_attempted=True, stream_id=stream_id, lease_released=False)
            op._connection.call.side_effect = error
            with self.assertRaises(TimeoutError): op.start_timed_ee({})
            self.assertTrue(op.timed_ee_start_attempted)
            self.assertEqual(op._own_timed_ee_stream_id, stream_id)
            if stream_id is None:
                with self.assertRaisesRegex(RuntimeError, 'unidentified'): op.stop_timed_ee()
                self.assertEqual(op._connection.call.call_count, 1)
            else:
                op._connection.call.side_effect = None
                op.stop_timed_ee()
                op._connection.call.assert_called_with('timed_ee_stop', allow_cancelled=True, stream_id=23)


if __name__ == '__main__':
    unittest.main()
