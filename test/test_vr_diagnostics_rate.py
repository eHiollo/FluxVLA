"""10Hz snapshots and immediate state events; synthetic robots only."""
import copy
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from fluxvla.engines.operators.vr.diagnostics import DiagnosticsRateGate
import test_vr_dual_runtime as dual


class DiagnosticsRateTest(unittest.TestCase):
    def test_90hz_input_produces_10hz_periodic_snapshots(self):
        gate = DiagnosticsRateGate()
        self.assertEqual(sum(gate.ready(('running',), now=i/90) for i in range(900)), 100)

    def test_pause_recovery_error_and_explicit_events_do_not_wait_for_period(self):
        gate = DiagnosticsRateGate()
        self.assertTrue(gate.ready(('running',), now=0.))
        self.assertFalse(gate.ready(('running',), now=.01))
        for t, state in ((.02, 'paused'), (.03, 'ready'), (.04, 'resumed'), (.05, 'error')):
            self.assertTrue(gate.ready((state,), now=t))
        self.assertFalse(gate.ready(('error',), now=.06))
        self.assertTrue(gate.ready(('error',), now=.1))
        for hz in (0, -1, True, float('nan'), float('inf')):
            with self.subTest(hz=hz), self.assertRaises(ValueError): DiagnosticsRateGate(hz)

    def test_runtime_skips_snapshot_copy_and_records_control_gap_immediately(self):
        fixture = dual.DualSDKRuntimeTest('test_sdk_initializes_once_and_reclutch_never_resets_other_arm')
        fixture.setUp(); self.addCleanup(fixture.doCleanups)
        fixture.clock.advance = lambda: setattr(fixture.clock, 'value', fixture.clock.value+1/90)
        fixture.move()
        records = []
        writer = SimpleNamespace(write=lambda row: records.append(copy.deepcopy(row)), close=lambda: None)
        runtime = fixture.runtime
        runtime._diagnostics = writer
        for _ in range(90): fixture.tick()
        self.assertEqual(len(records), 10)
        # An ordinary skipped tick must not retrieve full mapper diagnostics.
        getters = [Mock(wraps=m.get_last_diagnostics) for m in fixture.vr.mappers.values()]
        originals = [m.get_last_diagnostics for m in fixture.vr.mappers.values()]
        for m, getter in zip(fixture.vr.mappers.values(), getters): m.get_last_diagnostics = getter
        fixture.tick()
        for getter in getters: getter.reset_mock()
        count = len(records)
        fixture.tick()
        self.assertEqual(len(records), count)
        for getter in getters: getter.assert_not_called()
        fixture.clock.value += .11
        fixture.tick()
        self.assertEqual(len(records), count+1)
        self.assertEqual(records[-1]['input_pause']['reason'], 'control_gap')
        self.assertTrue(records[-1]['input_pause']['stopped'])
        for m, original in zip(fixture.vr.mappers.values(), originals): m.get_last_diagnostics = original


if __name__ == '__main__': unittest.main()
