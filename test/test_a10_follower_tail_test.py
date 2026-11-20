import unittest
import json
import tempfile
from pathlib import Path
import numpy as np
from scripts.a10_follower_tail_test import profile,check_envelope,repeated_packets,policy_offsets,observe_without_targets
class FollowerTailTest(unittest.TestCase):
    def test_profile_moves_one_joint_only_and_exact_endpoints(self):
        origin=np.arange(12,dtype=float)
        for delta in (.8,-.8):
            rows=profile(origin,3,delta)
            np.testing.assert_array_equal(rows[0],origin)
            self.assertAlmostEqual(rows[-1,3],origin[3]+delta)
            np.testing.assert_array_equal(np.delete(rows,3,axis=1),np.tile(np.delete(origin,3),(20,1)))
    def test_invalid_displacement_and_nonfinite_rejected(self):
        for delta in (0,1,float('nan')):
            with self.assertRaises(ValueError):profile(np.zeros(12),3,delta)
        with self.assertRaises(ValueError):profile(np.full(12,np.nan),3,.8)
    def test_guard_checks_both_actual_and_internal_and_other_axes(self):
        origin=np.zeros(12);target=origin.copy();target[3]=.8
        check_envelope(origin,target,origin,target)
        for internal in (False,True):
            q=origin.copy();q[0]=.2
            with self.assertRaises(RuntimeError):
                check_envelope(origin if internal else q,q if internal else origin,origin,target)

    def test_repeat_twice_preserves_model_duration_and_holds_exact_endpoint(self):
        rows=profile(np.zeros(12),3,.8)
        packets=repeated_packets(rows,2,0)
        self.assertEqual(packets.shape,(40,12))
        np.testing.assert_array_equal(packets[::2],rows)
        np.testing.assert_array_equal(packets[1::2],rows)
        self.assertAlmostEqual(len(packets)/60,len(rows)/30)
        held=repeated_packets(rows,2,.5)
        self.assertEqual(len(held),70)
        np.testing.assert_array_equal(held[40:],np.tile(rows[-1],(30,1)))

    def test_recorded_policy_is_reanchored_and_uniformly_bounded(self):
        source=np.tile(np.linspace(10,23,14),(20,1))
        source[:,2]+=np.linspace(0,.2,20)
        source[:,10]-=np.linspace(0,.08,20)
        events=[dict(event='observation',cycle=0,state=np.arange(14).tolist()),
                dict(event='prediction',cycle=0,converted_actions=source.tolist())]
        with tempfile.TemporaryDirectory() as folder:
            Path(folder,'events.jsonl').write_text('\n'.join(json.dumps(e) for e in events))
            offsets=policy_offsets(folder,.5)
        np.testing.assert_array_equal(offsets[0],np.zeros(12))
        self.assertAlmostEqual(np.max(abs(offsets)),.5)
        self.assertAlmostEqual(offsets[-1,9],-.2)
        expected=np.rad2deg(source[:,[0,1,2,3,4,5,7,8,9,10,11,12]]-source[0,[0,1,2,3,4,5,7,8,9,10,11,12]])
        np.testing.assert_allclose(offsets,expected*(.5/np.max(abs(expected))))

    def test_recorded_policy_displacement_limit_rejected_before_reading_file(self):
        for amplitude in (0,.6,float('nan')):
            with self.assertRaises(ValueError):policy_offsets('/not/a/log',amplitude)

    def test_no_send_observation_reads_until_deadline_without_stopping(self):
        from types import SimpleNamespace
        class Clock:
            now=0.
            def monotonic(self):return self.now
            def sleep(self,dt):self.now+=dt
        clock=Clock();reads=[];records=[]
        def read(expected):
            reads.append(expected)
            q=np.full(12,.3 if clock.now<.05 else .5)
            return SimpleNamespace(follower_active=True),q,q.copy()
        samples,first,stopped=observe_without_targets(read,.1,np.full(12,.5),
            lambda event,**fields:records.append(event),clock)
        self.assertEqual(reads,[None,None,None])
        self.assertAlmostEqual(first,2/30)
        self.assertFalse(stopped)
        self.assertEqual(len(samples),3)
        self.assertTrue(all(e=='active_no_send_feedback' for e in records))

    def test_no_send_observation_records_unrequested_mode_exit(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        read=Mock(return_value=(SimpleNamespace(follower_active=False),np.zeros(12),np.zeros(12)))
        records=[]
        samples,first,stopped=observe_without_targets(read,3,np.zeros(12),
            lambda event,**fields:records.append(event))
        self.assertTrue(stopped)
        self.assertEqual(read.call_count,1)
        self.assertIn('follower_stopped_without_request',records)
