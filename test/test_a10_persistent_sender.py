"""A single tick owner accepts latest predictions without cancelling sends."""
import threading
import time
import unittest
from unittest.mock import Mock

import numpy as np

from fluxvla.engines.utils.action_fusion import ExponentialActionFusion
from test_a10_async_execution import RecordingDevice, operator, state
from test_a10_dual_inference import Policy, SETTINGS
from fluxvla.engines.runners.a10_inference_runner import A10DualInferenceRunner


class ManualClock:
    def __init__(self):
        self.now = 0.
        self.condition = threading.Condition()
        self.cancelled = lambda: False
        self.waiting_until = None

    def monotonic(self):
        with self.condition:
            return self.now

    def sleep(self, duration):
        with self.condition:
            end = self.now+duration
            self.waiting_until = end
            self.condition.notify_all()
            try:
                while self.now < end-1e-10 and not self.cancelled():
                    self.condition.wait(.01)
            finally:
                self.waiting_until = None

    def advance_to(self, stamp):
        with self.condition:
            self.now = stamp
            self.condition.notify_all()


class PersistentSenderTests(unittest.TestCase):
    def test_public_runner_keeps_one_sender_through_overlapping_model_calls(self):
        class SlowPolicy(Policy):
            def infer(self, obs):
                time.sleep(.035)
                return super().infer(obs)

        device, log = RecordingDevice(), Mock()
        runner = A10DualInferenceRunner.from_components(device, SlowPolicy(),
            dict(SETTINGS, action_dt=.01, execute_horizon=0, async_execution=True,
                 action_fusion=dict(enabled=True, method='velocity', handover_steps=6,
                                    tail_steps=12, continuous_clock=True, persistent_sender=True)),
            log, cycles=4)
        try:
            runner.run()
            starts = [c.kwargs for c in log.call_args_list if c.args[0] == 'stream_publisher_start']
            sends = [c.kwargs for c in log.call_args_list if c.args[0] == 'sent']
            self.assertEqual(len(starts), 1)
            self.assertEqual({e['cycle'] for e in sends}, {0, 1, 2, 3})
            self.assertTrue(all(e['persistent_sender'] for e in sends))
        finally:
            runner.cleanup()

    def fixture(self, dt=.01):
        clock = ManualClock()
        device = RecordingDevice()
        op = operator(device, dt=dt, timed=True)
        op.clock = clock
        clock.cancelled = lambda: op._traj_stop_event.is_set()
        op.action_fusion = ExponentialActionFusion(dict(enabled=True, method='velocity',
            handover_steps=6, tail_steps=12, gripper_alpha=.35,
            continuous_clock=True, persistent_sender=True))
        condition = threading.Condition()

        def send(row, final=False):
            with condition:
                stamp = clock.monotonic()
                device.records.append((stamp, row.copy(), final))
                condition.notify_all()
            return dict(stream_id=7, index=len(device.records)-1, final=final,
                        udp_send_started_s=stamp)

        device.send = send

        def wait_points(count):
            with condition:
                self.assertTrue(condition.wait_for(lambda: len(device.records) >= count, 2))
            op.raise_trajectory_error()
            # Advance virtual time only once the worker has scheduled its
            # next wait, so the test does not race duration-based fake sleep.
            with clock.condition:
                self.assertTrue(clock.condition.wait_for(
                    lambda: clock.waiting_until is not None and clock.waiting_until > clock.now+1e-10, 2))

        return op, device, clock, wait_points

    def submit(self, op, rows, reference):
        op.execute_trajectory(dict(left=rows[:, :6], right=rows[:, 7:13]),
            dict(left=rows[:, 6], right=rows[:, 13]), dt=op.settings['action_dt'],
            async_exec=True, reference_time=reference)

    def test_thousand_points_and_many_replacements_keep_one_worker_and_no_missing_ticks(self):
        op, device, clock, wait = self.fixture(dt=1/30)
        rows = np.tile(state(device), (30, 1))
        try:
            self.submit(op, rows, 0.)
            wait(1)
            worker, cancel = op._traj_thread, op._traj_stop_event
            for tick in range(1, 1000):
                if tick % 7 == 0:
                    # The update arrives just before the old tick. A producer
                    # must not cancel/join that sleeping writer to replan.
                    clock.advance_to(tick/30-.0001)
                    op.cycle += 1
                    self.submit(op, rows, clock.monotonic())
                    self.assertIs(op._traj_thread, worker)
                    self.assertIs(op._traj_stop_event, cancel)
                    self.assertFalse(cancel.is_set())
                clock.advance_to(tick/30)
                wait(tick+1)
            times = np.array([r[0] for r in device.records])
            np.testing.assert_allclose(times, np.arange(1000)/30, atol=1e-9)
            sent = [c.kwargs for c in op.log.call_args_list if c.args[0] == 'sent']
            self.assertEqual([p['stream_index'] for p in sent], list(range(1000)))
            self.assertTrue(all(p['persistent_sender'] for p in sent))
            self.assertEqual(sum(c.args[0] == 'stream_publisher_start' for c in op.log.call_args_list), 1)
            self.assertEqual(device.stops, 0)
        finally:
            op.close()
        self.assertIsNone(op._stream_pending)
        self.assertFalse(op._stream_accepting)
        self.assertIsNone(op.reference_clock.origin_s)

    def test_latest_mailbox_coalesces_during_inflight_send_and_final_belongs_to_latest_plan(self):
        op, device, clock, wait = self.fixture()
        rows = np.tile(state(device), (8, 1))
        entered, release = threading.Event(), threading.Event()
        send = device.send

        def blocked_send(row, final=False):
            if len(device.records) == 1:
                entered.set()
                if not release.wait(2):
                    raise TimeoutError('Test did not release fake transport')
            return send(row, final)

        device.send = blocked_send
        try:
            self.submit(op, rows, 0.)
            wait(1)
            worker = op._traj_thread
            clock.advance_to(.01)
            self.assertTrue(entered.wait(1))
            for cycle in range(1, 101):
                op.cycle, op.stream_final = cycle, cycle == 100
                self.submit(op, rows, .01)
            self.assertIs(op._traj_thread, worker)
            self.assertEqual(op._stream_pending['cycle'], 100)
            release.set()
            wait(2)
            for tick in range(2, 9):
                clock.advance_to(tick*.01)
                wait(tick+1)
            clock.advance_to(.1)
            op.wait_trajectory(1)
            sent = [c.kwargs for c in op.log.call_args_list if c.args[0] == 'sent']
            self.assertEqual({p['cycle'] for p in sent}, {0, 100})
            self.assertEqual([p['final'] for p in sent], [False]*(len(sent)-1)+[True])
            self.assertEqual([p['stream_index'] for p in sent], list(range(len(sent))))
            self.assertEqual(device.stops, 0)
        finally:
            release.set()
            op.close()

    def test_exhaustion_holds_without_repeating_then_resumes_and_stop_clears_pending(self):
        op, device, clock, wait = self.fixture()
        rows = np.tile(state(device), (4, 1))
        try:
            self.submit(op, rows, 0.)
            wait(1)
            for tick in range(1, 4):
                clock.advance_to(tick*.01)
                wait(tick+1)
            clock.advance_to(.04)
            op.wait_trajectory(1)
            self.assertFalse(op._stream_accepting)
            clock.advance_to(.5)
            self.assertEqual(len(device.records), 4)
            self.submit(op, rows, .5)
            wait(5)
            self.assertEqual(device.stops, 0)
            op.stop_trajectory('test_pause')
            self.assertIsNone(op._stream_pending)
            self.assertIsNone(op.reference_clock.origin_s)
            self.assertEqual(device.stops, 1)
        finally:
            op.close()

    def test_persistent_background_failure_stops_and_propagates(self):
        device = RecordingDevice(fail_at=2)
        op = operator(device, dt=.01, timed=True)
        op.action_fusion = ExponentialActionFusion(dict(enabled=True, method='velocity',
            continuous_clock=True, persistent_sender=True))
        rows = np.tile(state(device), (30, 1))
        try:
            self.submit(op, rows, time.monotonic())
            self.assertTrue(device.stopped.wait(1))
            with self.assertRaises(OSError):
                op.wait_trajectory(1)
            self.assertIsNone(op._stream_pending)
            self.assertFalse(op._stream_accepting)
            self.assertIsNone(op.reference_clock.origin_s)
        finally:
            op.close()


if __name__ == '__main__':
    unittest.main()
