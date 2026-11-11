"""Distinguish missing replies from a delayed empty ACK without changing stop traffic."""
import unittest

from fluxvla.engines.operators.a10_operator import _FollowerConnection
from test_a10_operator import status_reply, wire_reply


class ScriptedWS:
    def __init__(self, replies):
        self.replies = iter(replies)
        self.sent = []
        self.closed = False

    def send(self, packet):
        self.sent.append(packet[40:].decode())

    def recv(self, timeout):
        item = next(self.replies, TimeoutError('no response'))
        if isinstance(item, Exception):
            raise item
        return wire_reply(item)

    def close(self):
        self.closed = True


class WSDiagnosticsTest(unittest.TestCase):
    def connection(self, replies):
        connection = _FollowerConnection('127.0.0.1', 5999, 9998, .1)
        ws = ScriptedWS(replies)
        connection.ws = ws
        return connection, ws

    def test_missing_reply_and_empty_ack_have_different_failure_evidence(self):
        for replies, expected in (([], []),
                                  ([dict(ret_code=0, ret_context='', ret_msg='')], ['empty_ack'])):
            with self.subTest(expected=expected):
                connection, ws = self.connection(replies)
                connection.send_command('stop_follower')
                with self.assertLogs('fluxvla.engines.operators.a10_operator', level='ERROR'):
                    with self.assertRaisesRegex(TimeoutError, "'get' reply timed out after 0.100s"):
                        connection.request('get', status=True)
                failure = connection.last_request_failure
                self.assertEqual([e['kind'] for e in failure['events']
                                  if e['event'] == 'received'], expected)
                self.assertEqual(ws.sent, ['stop_follower', 'get'])
                self.assertTrue(ws.closed)
                self.assertIsNone(connection.ws)
                self.assertEqual(failure['port'], 5999)

    def test_empty_ack_then_status_still_confirms_without_retry(self):
        connection, ws = self.connection([
            dict(ret_code=0, ret_context='', ret_msg=''), status_reply()])
        connection.send_command('stop_follower')
        reply = connection.request('get', status=True)
        self.assertIn('motion_msg', reply['ret_context'])
        self.assertEqual(ws.sent, ['stop_follower', 'get'])
        self.assertFalse(ws.closed)
        self.assertIsNone(connection.last_request_failure)

    def test_history_is_bounded_and_keeps_immediately_preceding_stop(self):
        connection, ws = self.connection([status_reply()] * 50)
        for _ in range(50):
            connection.request('get', status=True)
        connection.send_command('stop_follower')
        with self.assertLogs('fluxvla.engines.operators.a10_operator', level='ERROR'):
            with self.assertRaises(TimeoutError):
                connection.request('get', status=True)
        events = connection.last_request_failure['events']
        self.assertLessEqual(len(events), 32)
        self.assertEqual([e['command'] for e in events[-2:]], ['stop_follower', 'get'])
        self.assertEqual(ws.sent.count('stop_follower'), 1)


if __name__ == '__main__':
    unittest.main()
