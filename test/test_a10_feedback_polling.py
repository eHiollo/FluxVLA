"""Single upstream GET worker: cached telemetry never serializes UDP targets."""
import threading
import time
import unittest
from unittest.mock import patch

from test_a10_operator import FakeController, status_reply
from fluxvla.engines.operators.a10_operator import A10Operator, A10ControllerError
from fluxvla.engines.communication.a10_shared_server import SharedControl, SharedControlServer


class FeedbackPollingTest(unittest.TestCase):
    def setUp(self):
        self.fake = FakeController().__enter__()
        port = self.fake.server.socket.getsockname()[1]
        self.service = SharedControl('127.0.0.1', port, port,
            self.fake.udp.getsockname()[1], self.fake.joint_udp.getsockname()[1], timeout_s=.5)
        self.service.connect()
        self.server = SharedControlServer(('127.0.0.1', 0), self.service)
        self.thread = threading.Thread(target=self.server.serve_forever,
            kwargs=dict(poll_interval=.01), daemon=True)
        self.thread.start()
        self.client = A10Operator(host='127.0.0.1', ws_port=port, status_ws_port=port,
            udp_port=self.fake.udp.getsockname()[1], joint_udp_port=self.fake.joint_udp.getsockname()[1],
            timeout_s=.5, command_mode='joint', allow_motion=True,
            shared_control=dict(host='127.0.0.1', port=self.server.server_address[1], role='teleop'))
        self.client.connect()

    def tearDown(self):
        self.client.close()
        self.server.shutdown()
        self.thread.join(1)
        self.server.server_close()
        self.service.close()
        self.fake.__exit__(None, None, None)
        if self.service._poll_thread:
            self.assertFalse(self.service._poll_thread.is_alive())

    def test_slow_get_does_not_block_cache_or_udp_and_stop_is_fresh(self):
        self.client.start_follower()
        blocked, release = threading.Event(), threading.Event()
        def slow_get():
            blocked.set()
            release.wait(.3)
            return status_reply(active=self.fake.active, mode=self.fake.mode)
        self.fake.command_replies['get'] = slow_get
        self.client.start_feedback_polling(30)
        self.assertTrue(blocked.wait(.2))
        try:
            before = time.monotonic()
            first = self.client.get_cached_frame()
            self.client.send_joints({'right': [.1] * 6})
            self.assertTrue(self.fake.joint_udp.recv(1024))
            second = self.client.get_cached_frame()
            self.assertLess(time.monotonic() - before, .10)
            self.assertEqual(first['controller_feedback_sequence'], second['controller_feedback_sequence'])
            self.assertEqual(first['received_monotonic_s'], second['received_monotonic_s'])
            self.assertTrue(second['follower_active'])
        finally:
            release.set()
        # A cached active snapshot must never satisfy stop confirmation.
        self.client.stop_follower()
        self.assertFalse(self.client.get_state().follower_active)
        self.assertIn('stop_follower', self.fake.commands)

    def test_poll_rate_subtracts_query_time_and_subscribers_share_one_worker(self):
        def slow_get():
            time.sleep(.012)
            return status_reply()
        self.fake.command_replies['get'] = slow_get
        self.client.start_feedback_polling(30)
        thread = self.service._poll_thread
        self.service.configure_feedback_poll('another-subscriber', 20)
        self.assertIs(self.service._poll_thread, thread)
        time.sleep(.45)
        snapshot = self.client.get_cached_frame()
        self.assertGreaterEqual(snapshot['feedback_polling']['count'], 12)
        self.assertEqual(snapshot['feedback_polling']['requested_hz'], 30)
        self.client.stop_feedback_polling()
        self.assertEqual(self.service.feedback_snapshot()['polling']['requested_hz'], 20)
        self.service.configure_feedback_poll('another-subscriber', None)

    def test_known_controller_fault_replaces_cache_and_revokes_targets(self):
        self.client.start_follower()
        self.client.start_feedback_polling(30)
        def fault():
            reply = status_reply(active=True, mode='Axis')
            reply['ret_context']['log_msg']['has_error'] = True
            return reply
        self.fake.command_replies['get'] = fault
        deadline = time.monotonic() + .4
        while self.client._feedback_cache[1] is None and time.monotonic() < deadline:
            time.sleep(.005)
        with self.assertRaises(A10ControllerError):
            self.client.get_cached_frame()
        with self.assertRaises((InterruptedError, RuntimeError)):
            self.client.send_joints({'right': [0.] * 6})
        self.fake.command_replies.pop('get')
        self.client.stop_follower()

    def test_fault_releases_get_lock_before_waiting_for_target_lock(self):
        entered, release = threading.Event(), threading.Event()
        def fault():
            entered.set()
            release.wait(.3)
            reply = status_reply()
            reply['ret_context']['log_msg']['has_error'] = True
            return reply
        self.fake.command_replies['get'] = fault
        self.client.start_feedback_polling(30)
        self.assertTrue(entered.wait(.2))
        with self.service._target_lock:
            release.set()
            # Stop/command paths can own target_lock before reading status.
            # A failed poll must not invert these two locks.
            acquired = self.service._status_lock.acquire(timeout=.15)
            try:
                self.assertTrue(acquired)
            finally:
                if acquired:
                    self.service._status_lock.release()
        self.fake.command_replies.pop('get')

    def test_latest_failure_includes_cache_and_survives_reader_cleanup(self):
        self.client.start_feedback_polling(30)
        command_failure = dict(command='get', failed_wall_ns=10)
        cache_failure = dict(command='feedback_snapshot', failed_wall_ns=20)
        self.client._connection.last_request_failure = command_failure
        self.client._feedback_reader.last_request_failure = cache_failure
        self.assertIs(self.client.last_communication_failure, cache_failure)
        self.client.stop_feedback_polling()
        self.assertIs(self.client.last_communication_failure, cache_failure)
        command_failure = dict(command='stop_follower', failed_wall_ns=30)
        self.client._connection.last_request_failure = command_failure
        self.assertIs(self.client.last_communication_failure, command_failure)

    def test_invalid_rate_does_not_start_worker(self):
        for rate in (0, -1, True, float('nan'), 101):
            with self.assertRaises(ValueError):
                self.client.start_feedback_polling(rate)
        self.assertIsNone(self.service._poll_thread)

    def test_memory_read_does_not_wait_for_local_rpc_or_parse_and_is_isolated(self):
        self.client.start_feedback_polling(30)
        reader = self.client._feedback_reader
        first = self.client.get_cached_frame()
        entered, release = threading.Event(), threading.Event()
        original = reader.call
        def delayed(*args, **kwargs):
            entered.set()
            release.wait(.5)
            return original(*args, **kwargs)
        try:
            with patch.object(reader, 'call', delayed):
                self.assertTrue(entered.wait(.2))
                first = self.client.get_cached_frame()
                with patch.object(self.client, '_parse_feedback', side_effect=AssertionError('foreground parse')):
                    started = time.monotonic()
                    rows = [self.client.get_cached_frame() for _ in range(90)]
                    self.assertLess(time.monotonic()-started, .1)
                    self.assertEqual({r['received_monotonic_s'] for r in rows}, {first['received_monotonic_s']})
                    self.assertEqual(rows[-1]['motion'], 'Stopped')
                    self.assertEqual(rows[-1]['activation'], 'Enabled')
                    rows[-1]['joints'][0] = 999.
                    self.assertNotEqual(self.client.get_cached_frame()['joints'][0], 999.)
        finally:
            release.set()

    def test_duplicate_sequence_parses_once_and_cannot_refresh_time(self):
        snapshot = self.service.feedback_snapshot()
        with patch.object(self.client, '_parse_feedback', wraps=self.client._parse_feedback) as parse:
            first = self.client._frame_from_feedback_snapshot(snapshot)
            for _ in range(10):
                second = self.client._frame_from_feedback_snapshot(snapshot, previous_frame=first)
            self.assertEqual(parse.call_count, 1)
        self.assertEqual(first['received_monotonic_s'], second['received_monotonic_s'])
        altered = dict(snapshot, received_monotonic_s=snapshot['received_monotonic_s']-.01)
        with self.assertRaisesRegex(ValueError, 'different timestamp'):
            self.client._frame_from_feedback_snapshot(altered, previous_frame=first)

    def test_stop_interrupts_local_snapshot_wait_and_repeated_start_reuses_worker(self):
        self.client.start_feedback_polling(30)
        worker = self.client._feedback_worker
        self.client.start_feedback_polling(60)
        self.assertIs(self.client._feedback_worker, worker)
        entered, release = threading.Event(), threading.Event()
        original = self.service.feedback_snapshot
        def delayed():
            entered.set()
            release.wait(1.)
            return original()
        try:
            with patch.object(self.service, 'feedback_snapshot', delayed):
                self.assertTrue(entered.wait(.2))
                started = time.monotonic()
                self.client.stop_feedback_polling()
                self.assertLess(time.monotonic()-started, .25)
                self.assertFalse(worker.is_alive())
                self.assertIsNone(self.client._feedback_reader)
                self.assertFalse(self.service._poll_rates)
        finally:
            release.set()
        self.client.start_feedback_polling(30)
        self.assertIsNot(self.client._feedback_worker, worker)
        self.assertIsNone(self.client._feedback_cache[1])

    def test_worker_error_is_reported_when_consumed_and_never_auto_reconnects(self):
        self.client.start_feedback_polling(30)
        reader = self.client._feedback_reader
        failure = dict(command='feedback_snapshot', failed_wall_ns=time.time_ns(), error='test timeout')
        def fail(*args, **kwargs):
            reader.last_request_failure = failure
            raise TimeoutError('test timeout')
        with patch.object(reader, 'call', side_effect=fail) as call:
            self.client._feedback_worker.join(.01)  # Worker is normally still polling.
            deadline = time.monotonic()+.25
            while self.client._feedback_cache[1] is None and time.monotonic()<deadline:
                time.sleep(.005)
            consumed = time.time_ns()
            with self.assertRaisesRegex(TimeoutError, 'test timeout'):
                self.client.get_cached_frame()
            self.assertGreaterEqual(self.client.last_communication_failure['failed_wall_ns'], consumed)
            self.assertEqual(self.client.last_communication_failure['background_failed_wall_ns'],
                             failure['failed_wall_ns'])
            self.client._feedback_worker.join(.2)
            self.assertFalse(self.client._feedback_worker.is_alive())
            self.assertEqual(call.call_count, 1)

    def test_sync_start_and_stop_confirmations_supersede_old_background_reply(self):
        self.client.start_feedback_polling(30)
        entered, release = threading.Event(), threading.Event()
        reader = self.client._feedback_reader
        original = reader.call
        old_snapshot = self.service.feedback_snapshot()
        def delayed(*args, **kwargs):
            entered.set()
            release.wait(.8)
            return old_snapshot
        try:
            with patch.object(reader, 'call', delayed):
                self.assertTrue(entered.wait(.2))
                self.client.start_follower()
                active = self.client.get_cached_frame()
                self.assertTrue(active['follower_active'])
                self.assertEqual(active['follower_mode'], 'Axis')
                self.assertIsNone(active['controller_feedback_sequence'])
                self.assertEqual(active['feedback_source'], 'synchronous_confirmation')
                release.set()
                time.sleep(.05)  # Old inactive replies arrive after start confirmation.
                self.assertTrue(self.client.get_cached_frame()['follower_active'])
                self.assertGreaterEqual(self.client.get_cached_frame()['received_monotonic_s'],
                                        active['received_monotonic_s'])
                self.client.stop_follower()
                stopped = self.client.get_cached_frame()
                self.assertFalse(stopped['follower_active'])
                self.assertEqual(stopped['motion'], 'Stopped')
        finally:
            release.set()

    def test_sync_confirmation_does_not_hide_a_known_worker_error(self):
        self.client.start_feedback_polling(30)
        old = self.client.get_cached_frame()
        self.client._feedback_worker_stop.set()
        self.client._feedback_worker.join(.2)
        error = TimeoutError('known background failure')
        self.client._feedback_cache = (old, error, None)
        self.client.get_state()
        with self.assertRaisesRegex(TimeoutError, 'known background failure'):
            self.client.get_cached_frame()


if __name__ == '__main__':
    unittest.main()
