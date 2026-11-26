"""Keep the model reference grid stable across publication replacements."""
import time
import unittest

import numpy as np

from fluxvla.engines.utils.chunk_execution import StreamReferenceClock
from fluxvla.engines.utils.action_fusion import ExponentialActionFusion
from test_a10_async_execution import RecordingDevice, operator, state
from test_a10_dual_inference import Clock


class ReferenceClockTests(unittest.TestCase):
    def test_send_lateness_does_not_reanchor_the_model_reference(self):
        dt=1/30
        c=StreamReferenceClock(dt)
        first=c.next_slot(100.)
        c.record(first)
        # The previous packet was 8.85ms late. New data is ready before the
        # next nominal tick; its reference advances 33.33ms, not 42.18ms.
        following=c.next_slot(100.+.025)
        self.assertAlmostEqual(following-first,dt)
        c.record(following)
        # A 1s inference gap does not become an accumulated execution queue.
        resumed=c.next_slot(101.013)
        self.assertGreaterEqual(resumed,101.013)
        self.assertLess(resumed-101.013,dt)
        self.assertAlmostEqual((resumed-first)/dt,round((resumed-first)/dt))
        c.record(resumed)
        with self.assertRaises(ValueError):c.record(resumed)
        c.reset()
        self.assertEqual(c.next_slot(300.),300.)

    def test_long_stream_keeps_one_grid_and_does_not_accumulate_jitter(self):
        dt=1/30;c=StreamReferenceClock(dt);first=c.next_slot(10.)
        for tick in range(1000):
            expected=first+tick*dt
            slot=c.next_slot(expected-.002 if tick else first)
            self.assertAlmostEqual(slot,expected,places=9)
            c.record(slot)
        self.assertAlmostEqual(c.last_s-first,999*dt)

    def test_late_slots_are_skipped_once_without_changing_wire_numbering_or_final(self):
        clock=Clock();d=RecordingDevice();op=operator(d,timed=True,dt=.01)
        op.clock=clock
        def send(row,final=False):
            # The first local send stalls by 25ms; no commands may be burst
            # out afterwards just to drain the expired 10/20ms targets.
            if not d.records:clock.sleep(.025)
            d.records.append((clock.now,row.copy(),final))
            return dict(stream_id=7,index=len(d.records)-1,final=final,
                        udp_send_started_s=clock.now)
        d.send=send
        rows=np.tile(state(d),(6,1));rows[:,0]+=np.arange(6)*.001
        start=op.reference_clock.next_slot(clock.now)
        try:
            op._send_rows(rows,.01,start_s=start,continuous_clock=True,final=True)
            sent=[c.kwargs for c in op.log.call_args_list if c.args[0]=='sent']
            self.assertEqual([e['frame'] for e in sent],[1,4,5,6])
            self.assertEqual([e['stream_index'] for e in sent],list(range(4)))
            self.assertEqual([e['final'] for e in sent],[False]*3+[True])
            self.assertEqual([r[1][0] for r in d.records],[rows[i,0] for i in [0,3,4,5]])
            self.assertTrue(all(e['continuous_clock'] for e in sent))
            self.assertTrue(np.all(np.diff([r[0] for r in d.records])>=.005-1e-9))
        finally:op.close()

    def test_shared_clock_publisher_error_stops_and_clears_state(self):
        d=RecordingDevice(fail_at=2);op=operator(d,timed=True,dt=.01)
        op.action_fusion=ExponentialActionFusion(dict(enabled=True,method='velocity',
                                                    continuous_clock=True))
        rows=np.tile(state(d),(30,1))
        try:
            op.execute_trajectory(dict(left=rows[:,:6],right=rows[:,7:13]),
                dict(left=rows[:,6],right=rows[:,13]),dt=.01,async_exec=True,
                reference_time=time.monotonic())
            self.assertTrue(d.stopped.wait(1))
            with self.assertRaises(OSError):op.wait_trajectory(1)
            self.assertIsNone(op.reference_clock.origin_s)
            self.assertIsNone(op.action_fusion.plan)
        finally:op.close()
        self.assertIsNone(op.reference_clock.origin_s)

    def test_real_background_replacements_preserve_phase_with_slow_transport(self):
        class SlowDevice(RecordingDevice):
            def send(self,row,final=False):
                time.sleep(.003)
                result=super().send(row,final)
                result['udp_send_started_s']=self.records[-1][0]
                return result
        d=SlowDevice();op=operator(d,timed=True,dt=.02)
        op.action_fusion=ExponentialActionFusion(dict(enabled=True,method='velocity',
            handover_steps=6,tail_steps=12,continuous_clock=True))
        rows=np.tile(state(d),(30,1))
        try:
            for cycle in range(3):
                op.cycle=cycle;op.stream_final=cycle==2
                op.execute_trajectory(dict(left=rows[:,:6],right=rows[:,7:13]),
                    dict(left=rows[:,6],right=rows[:,13]),dt=.02,async_exec=True,
                    reference_time=time.monotonic())
                if cycle<2:time.sleep(.085)
            op.wait_trajectory(2)
            sent=[c.kwargs for c in op.log.call_args_list if c.args[0]=='sent']
            refs=np.array([e['scheduled_s'] for e in sent]);ticks=(refs-refs[0])/.02
            np.testing.assert_allclose(ticks,np.round(ticks),atol=1e-7)
            self.assertTrue(np.all(np.diff(ticks)>=1-1e-7))
            self.assertEqual([e['stream_index'] for e in sent],list(range(len(sent))))
            self.assertEqual([e['final'] for e in sent],[False]*(len(sent)-1)+[True])
            self.assertEqual(d.stops,0)
        finally:op.close()


if __name__=='__main__':unittest.main()
