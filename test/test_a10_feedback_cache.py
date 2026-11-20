"""Feedback cache ownership and freshness; no controller connection is opened."""
import copy
import threading
import unittest
from unittest.mock import patch

from fluxvla.engines.communication.a10_shared_server import SharedControl, _copy_feedback


class FeedbackCacheTest(unittest.TestCase):
    def setUp(self):
        self.robot = SharedControl('127.0.0.1')
        self.reply = {'ret_code': 0, 'ret_context': {
            'motion_msg': {'actual_pos': [[1., 2.], [3., 4.]]},
            'flags': [True, False, None, '运行'], 'value': 7}}

    def test_copy_matches_deepcopy_and_isolates_nested_values(self):
        result = _copy_feedback(self.reply)
        self.assertEqual(result, copy.deepcopy(self.reply))
        result['ret_context']['motion_msg']['actual_pos'][0][0] = 99
        self.assertEqual(self.reply['ret_context']['motion_msg']['actual_pos'][0][0], 1.)

    def test_non_json_mutable_values_still_get_copied(self):
        value = {'extra': bytearray(b'abc')}
        result = _copy_feedback(value)
        result['extra'][0] = 0
        self.assertEqual(value['extra'], bytearray(b'abc'))

    def test_publication_and_each_reader_have_independent_copies(self):
        self.assertIs(self.robot._remember_state(self.reply), self.reply)
        self.reply['ret_context']['motion_msg']['actual_pos'][0][0] = 99
        first = self.robot.feedback_snapshot()
        self.assertEqual(first['reply']['ret_context']['motion_msg']['actual_pos'][0][0], 1.)
        first['reply']['ret_context']['motion_msg']['actual_pos'][0][0] = 88
        self.assertEqual(self.robot.feedback_snapshot()['reply']['ret_context']
                         ['motion_msg']['actual_pos'][0][0], 1.)

    def test_reads_do_not_refresh_timestamp_or_sequence(self):
        with patch('fluxvla.engines.communication.a10_shared_server.time.monotonic', return_value=10.):
            self.robot._remember_state(self.reply)
        first = self.robot.feedback_snapshot()
        second = self.robot.feedback_snapshot()
        self.assertEqual(first['received_monotonic_s'], 10.)
        self.assertEqual(second['received_monotonic_s'], 10.)
        self.assertEqual(first['sequence'], second['sequence'])
        self.robot._remember_state(self.reply)
        self.assertEqual(self.robot.feedback_snapshot()['sequence'], first['sequence'] + 1)

    def test_known_poll_fault_is_not_hidden_by_cache(self):
        self.robot._remember_state(self.reply)
        self.robot._poll_error = RuntimeError('feedback failed')
        with self.assertRaisesRegex(RuntimeError, 'feedback failed'):
            self.robot.feedback_snapshot()

    def test_slow_reader_does_not_hold_publication_lock(self):
        self.robot._remember_state(self.reply)
        entered, release = threading.Event(), threading.Event()
        original = _copy_feedback
        results = []

        def delayed(value):
            if threading.current_thread().name == 'cache-reader' and value is self.robot._snapshot:
                entered.set()
                if not release.wait(2.):
                    raise TimeoutError('reader was not released')
            return original(value)

        with patch('fluxvla.engines.communication.a10_shared_server._copy_feedback', side_effect=delayed):
            reader = threading.Thread(target=lambda: results.append(self.robot.feedback_snapshot()),
                                      name='cache-reader')
            reader.start()
            try:
                self.assertTrue(entered.wait(2.))
                self.assertTrue(self.robot._snapshot_lock.acquire(timeout=.2))
                self.robot._snapshot_lock.release()
                self.robot._remember_state({'ret_code': 1})
            finally:
                release.set()
                reader.join(2.)
        self.assertEqual(results[0]['reply']['ret_code'], 0)
        self.assertEqual(self.robot.feedback_snapshot()['reply']['ret_code'], 1)


if __name__ == '__main__':
    unittest.main()
