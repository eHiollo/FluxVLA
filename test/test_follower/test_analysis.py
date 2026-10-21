"""Offline tests: known delays, causal alignment and plot artifacts."""
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

try:
    from .analyze import analyze, estimate_delay, step_metrics, load_signals
    from .run import make_chunk, main
except ImportError:
    from analyze import analyze, estimate_delay, step_metrics, load_signals
    from run import make_chunk, main


class AnalysisTest(unittest.TestCase):
    def test_step_delay_and_settling_from_known_response(self):
        t=np.arange(0,2,.02)
        actual=5*np.clip((t-.12)/.5,0,1)
        report=step_metrics(t,actual,0,5)
        self.assertAlmostEqual(report['response_delay_s'],.14,delta=.021)
        self.assertAlmostEqual(report['t90_s'],.58,delta=.021)
        self.assertAlmostEqual(report['rise_10_90_s'],.4,delta=.021)
        self.assertAlmostEqual(report['settling_time_s'],.6,delta=.021)
        self.assertEqual(report['overshoot_deg'],0.)
        self.assertIsNone(step_metrics(t,actual*0,0,5)['t90_s'])

    def test_lag_fit_finds_known_delay_and_does_not_invent_motion(self):
        st=np.arange(0,4,.01)
        target=np.tile((2*np.sin(st*1.5))[:,None],(1,6))
        qt=np.arange(0,4,.02)
        actual=target[np.maximum(0,np.searchsorted(st,qt-.12,side='right')-1)]
        result=estimate_delay(st,target,qt,actual,3)
        self.assertTrue(result['valid'])
        self.assertAlmostEqual(result['estimated_delay_s'],.12,delta=.01)
        self.assertLess(result['delay_compensated_rmse_deg'],.02)
        self.assertFalse(estimate_delay(st,target,qt,actual*0,3)['valid'])

    def test_chunk_is_bounded_smooth_and_returns_to_origin(self):
        origin=np.array([1.,2.,3.,4.,5.,6.])
        targets=make_chunk(origin,4,5,120)
        np.testing.assert_allclose(targets[0],origin)
        np.testing.assert_allclose(targets[-1],origin)
        self.assertLessEqual(float(np.max(targets[:,3])),9.)
        self.assertLess(float(np.max(abs(np.diff(targets[:,3])))),.14)
        np.testing.assert_array_equal(targets[:,[0,1,2,4,5]],np.tile(origin[[0,1,2,4,5]],(120,1)))

    def test_plot_artifacts_and_partial_phases_are_marked(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);rows=[]
            def event(e,t,**kw):rows.append(dict(event=e,monotonic_s=t,**kw))
            event('phase_start',10.,phase='step',start_deg=[0.]*6,target_deg=[5.]*6)
            for i in range(100):
                t=10+i*.02;q=5*min(1.,max(0.,(i*.02-.1)/.5))
                event('send',t,phase='step',send_started_s=t,target_deg=[5.]*6)
                event('state',t+.005,received_monotonic_s=t+.005,actual_deg=[q]*6)
            event('phase_result',12.,phase='step')
            event('phase_start',12.01,phase='chunk',start_deg=[5.]*6,target_deg=None)
            event('send',12.02,phase='chunk',send_started_s=12.02,target_deg=[4.]*6)
            event('state',12.03,received_monotonic_s=12.03,actual_deg=[4.5]*6)
            event('test_error',12.04,error='injected interrupt')
            (root/'events.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
            (root/'summary.json').write_text(json.dumps(dict(dropped_events=0,write_errors=0)))
            report=analyze(root)
            self.assertTrue(report['phases'][0]['completed'])
            self.assertFalse(report['phases'][1]['completed'])
            self.assertEqual(len(report['errors']),1)
            for name in ('overview.png','step.png','chunk.png','overview.svg','aligned.csv','metrics.json'):
                self.assertGreater((root/'follower_report'/name).stat().st_size,100)
            signals=load_signals(root)
            self.assertEqual(signals['reference'][-1,0],4.)

    def test_invalid_arguments_do_not_connect(self):
        import contextlib,io
        from unittest.mock import patch
        for args in (['--delta-deg','50'],['--chunk-gap','-1'],['--repeat','0']):
            with self.subTest(args=args),patch('socket.socket',side_effect=AssertionError('network')), \
                    contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
                main(args)


if __name__=='__main__':
    unittest.main()
