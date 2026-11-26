"""Shared chunk scheduling, real worker concurrency and loopback dual-arm RTC."""

import contextlib
import io
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import torch

from test_a10_dual_inference import b, Device, Policy, SETTINGS, Clock
from fluxvla.engines.operators.a10_dual_operator import A10DualOperator
from fluxvla.engines.runners.a10_inference_runner import A10DualInferenceRunner
from fluxvla.engines.runners.aloha_inference_runner import AlohaInferenceRunner
from fluxvla.engines.runners.base_inference_runner import BaseInferenceRunner
from fluxvla.engines.utils.a10_rtc import inject_rtc, resolve_rtc_config
from fluxvla.transforms.a10_openpi import A10RTCDenormalizeAction


def state(device):
    return b.observation(device.sample(), SETTINGS)['observation/state']


class RecordingDevice(Device):
    def __init__(self, fail_at=None):
        super().__init__()
        self.records = []
        self.started = threading.Event()
        self.stopped = threading.Event()
        self.fail_at = fail_at

    def send(self, row, final=False):
        if self.fail_at is not None and len(self.records) >= self.fail_at:
            raise OSError('uncertain send')
        self.records.append((time.monotonic(), np.array(row, copy=True), final))
        super().send(row)
        self.started.set()
        return dict(stream_id=7, index=len(self.records)-1, final=final)

    def stop(self):
        result = super().stop()
        self.stopped.set()
        return result


def operator(device, *, dt=.01, timed=False):
    settings = dict(SETTINGS, action_dt=dt, timed_aj=dict(enabled=timed),
                    follower_tail=dict(enabled=False))
    op = A10DualOperator(devices=device, mode='motion' if timed else 'simulate',
                         settings=settings, log=Mock())
    op.last_frame = dict(qpos=state(device))
    return op


def publish(op, rows, asynchronous=True):
    op.execute_trajectory(
        dict(left=rows[:, :6], right=rows[:, 7:13]),
        dict(left=rows[:, 6], right=rows[:, 13]),
        dt=op.settings['action_dt'], async_exec=asynchronous)


class AsyncOperatorTests(unittest.TestCase):
    def test_replacement_joins_old_writer_and_preserves_stream_without_queue(self):
        d = RecordingDevice()
        d.check = Mock(wraps=d.check)
        op = operator(d, timed=True)
        first = np.tile(state(d), (30, 1)); first[:, 0] += .01
        second = first.copy(); second[:, 0] += .02
        try:
            publish(op, first)
            self.assertTrue(d.started.wait(1))
            time.sleep(.025)
            op.cycle = 1
            op.stream_final = True
            publish(op, second[:4])
            op.wait_trajectory(1)
            rows = np.array([record[1] for record in d.records])
            first_new = np.flatnonzero(rows[:, 0] > first[0, 0] + .01)[0]
            self.assertLess(first_new, 30)
            np.testing.assert_allclose(rows[first_new:], second[:4])
            self.assertEqual([r[2] for r in d.records], [False] * (len(rows)-1) + [True])
            self.assertEqual(d.stops, 0)
            # SimDevices.send performs one local active check itself. The
            # timed publishing loop must not add feedback checks around it.
            self.assertEqual(d.check.call_count, len(d.records))
            self.assertTrue(np.all(np.diff([r[0] for r in d.records]) >= .008))
            sent = [c.kwargs for c in op.log.call_args_list if c.args[0] == 'sent']
            self.assertEqual([r['stream_index'] for r in sent], list(range(len(rows))))
            self.assertEqual({r['stream_id'] for r in sent}, {7})
            self.assertEqual({r['cycle'] for r in sent[:first_new]}, {0})
            self.assertEqual({r['cycle'] for r in sent[first_new:]}, {1})
        finally:
            op.close()
        self.assertFalse(op.is_trajectory_running())
        count = len(d.records)
        time.sleep(.025)
        self.assertEqual(len(d.records), count)
        self.assertEqual(d.stops, 1)

    def test_background_error_stops_hardware_without_waiting_for_next_model_reply(self):
        d = RecordingDevice(fail_at=2)
        op = operator(d)
        rows = np.tile(state(d), (30, 1))
        try:
            publish(op, rows)
            self.assertTrue(d.stopped.wait(1))
            with self.assertRaisesRegex(OSError, 'uncertain send'):
                op.get_frame()
            with self.assertRaises(OSError):
                publish(op, rows)
            self.assertEqual(len(d.records), 2)
        finally:
            op.close()
        self.assertEqual(d.stops, 1)

    def test_stop_interrupts_synchronous_publisher_and_closed_operator_cannot_restart(self):
        d = RecordingDevice()
        op = operator(d)
        rows = np.tile(state(d), (30, 1))
        errors = []
        def execute():
            try:
                publish(op, rows, False)
            except BaseException as exc:
                errors.append(exc)
        worker = threading.Thread(target=execute)
        worker.start()
        self.assertTrue(d.started.wait(1))
        op.close()
        worker.join(1)
        self.assertFalse(worker.is_alive())
        self.assertTrue(errors and isinstance(errors[0], InterruptedError))
        with self.assertRaisesRegex(RuntimeError, 'closed'):
            publish(op, rows)


class FrameworkAsyncTests(unittest.TestCase):
    def test_first_controller_handshake_precedes_async_action_clock(self):
        clock, d = Clock(), Device()
        d.start = lambda: clock.sleep(.25)
        r = A10DualInferenceRunner.from_components(
            d, Policy(), dict(SETTINGS, async_execution=True), Mock(), cycles=5, clock=clock)
        r.ros_operator.execute_trajectory = Mock()
        r._run_action_chunk('fold')
        self.assertAlmostEqual(r._prev_ctx.action_timestamp, .25)
        self.assertAlmostEqual(clock.now, .25 + 10 / 30)
        r.cleanup()

    def test_keyboard_pause_cancels_background_tail_and_resets_shared_context(self):
        d = RecordingDevice()
        r = A10DualInferenceRunner.from_components(
            d, Policy(), dict(SETTINGS, action_dt=.01, execute_horizon=5,
                             async_execution=True), Mock(), cycles=5)
        r._poll_keyboard_pause = lambda: True
        with contextlib.redirect_stdout(io.StringIO()):
            r.run()
        self.assertIsNone(r._prev_ctx)
        self.assertIsNone(r.observation_window)
        self.assertLess(d.sent, 30)
        self.assertFalse(r.ros_operator.is_trajectory_running())
        self.assertTrue(d.closed)

    def test_aloha_and_a10_share_first_chunk_and_fractional_replacement_semantics(self):
        outputs = []
        for kind in ('aloha', 'a10'):
            clock, d = Clock(), Device()
            actions = np.tile(state(d), (30, 1))
            actions[:, 0] += np.arange(30) * .001
            sent = []
            if kind == 'a10':
                runner = A10DualInferenceRunner.from_components(
                    d, Policy(), dict(SETTINGS, async_execution=True), Mock(),
                    cycles=5, clock=clock)
                runner.ros_operator.last_frame = dict(qpos=state(d), robot_time=0.)
                runner.observation_window = [dict(robot_time=0.)]
                def execute(arm_trajectories, gripper_trajectories, *, dt, async_exec):
                    self.assertTrue(async_exec)
                    sent.append(np.column_stack([arm_trajectories['left'], gripper_trajectories['left'],
                                                 arm_trajectories['right'], gripper_trajectories['right']]))
                runner.ros_operator.execute_trajectory = execute
            else:
                def execute(left, right, **kwargs):
                    self.assertTrue(kwargs['async_exec'])
                    sent.append(np.c_[left, right])
                op = SimpleNamespace(execute_trajectory=execute)
                with patch('fluxvla.engines.runners.base_inference_runner.build_operator_from_cfg', return_value=op), \
                        patch.object(BaseInferenceRunner, '_init_zmq_client'):
                    runner = AlohaInferenceRunner(operator={'type': 'fake'}, remote_inference={},
                        action_chunk=30, publish_rate=30, execute_horizon=10,
                        async_execution=True, enable_mixed_precision=False)
            runner._action_ctx = SimpleNamespace(state=state(d), inference_start=0.)
            with patch('fluxvla.engines.runners.aloha_inference_runner.time.time', side_effect=clock.monotonic), \
                    patch('fluxvla.engines.runners.aloha_inference_runner.time.sleep', side_effect=clock.sleep):
                runner._execute_actions(actions.copy(), None)
                runner._prev_ctx = runner._action_ctx
                runner._action_ctx = SimpleNamespace(state=state(d), inference_start=clock.now)
                clock.sleep(2.5 / 30)
                runner._execute_actions(actions.copy(), None)
            outputs.append(sent)
        self.assertEqual(len(outputs[0][0]), 30)
        self.assertEqual(len(outputs[0][1]), 28)
        for left, right in zip(*outputs):
            np.testing.assert_allclose(left, right)
        self.assertAlmostEqual(outputs[0][1][0, 0], actions[0, 0] + .0025)

    def test_model_inference_overlaps_actual_sends_and_old_tail_is_replaced(self):
        d = RecordingDevice()
        policy = Policy()
        entered, release = threading.Event(), threading.Event()
        original = policy.infer
        calls = []
        def infer(obs):
            calls.append(len(d.records))
            if len(calls) == 2:
                entered.set()
                if not release.wait(2):
                    raise TimeoutError('test release missing')
            return original(obs)
        policy.infer = infer
        runner = A10DualInferenceRunner.from_components(
            d, policy, dict(SETTINGS, action_dt=.01, execute_horizon=5,
                            async_execution=True), Mock(), cycles=2)
        errors = []
        def run():
            try:
                with contextlib.redirect_stdout(io.StringIO()):
                    runner.run()
            except BaseException as exc:
                errors.append(exc)
        worker = threading.Thread(target=run)
        worker.start()
        try:
            self.assertTrue(entered.wait(2))
            count = len(d.records)
            time.sleep(.035)
            self.assertGreater(len(d.records), count)
        finally:
            release.set()
            worker.join(3)
        self.assertFalse(worker.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(len(calls), 2)
        self.assertTrue(d.closed and policy.closed)
        self.assertLess(len(d.records), 60)
        self.assertEqual(d.stops, 1)


RTC = dict(enabled=True, method='guidance', prefix_len=12, decay_end=18,
           schedule='exp', max_guidance_weight=5., use_vjp=False)


def transform():
    return A10RTCDenormalizeAction(
        {'a10_dual': {'action': {'q01': [-1.] * 14, 'q99': [1.] * 14}}},
        state_dim=14, statistic_name='a10_dual', norm_type='quantile')


class DualRTCPolicy(torch.nn.Module):
    n_action_steps, max_action_dim = 30, 32

    def __init__(self):
        super().__init__()
        self.prefixes = []

    def predict_action(self, states, prev_actions=None, prefix_len=0, rtc_config=None, **kwargs):
        self.prefixes.append(None if prev_actions is None else prev_actions.clone())
        result = torch.zeros(1, 30, 32)
        if prev_actions is not None:
            if rtc_config['method'] == 'prefix':
                from fluxvla.models.vlas.pi0_flowmatching import PI0FlowMatching
                return PI0FlowMatching._predict_action_prefix_rtc(
                    self, result, lambda x, t: torch.zeros_like(x), 1,
                    torch.tensor(-.1), 'cpu', prev_actions, prefix_len)
            from fluxvla.engines.utils.rtc_guidance import apply_rtc_guidance, compute_prefix_weights
            result = apply_rtc_guidance(result, torch.zeros_like, prev_actions,
                compute_prefix_weights(30, prefix_len, rtc_config['decay_end']), .5, 5.,
                use_vjp=rtc_config.get('use_vjp', False))
        return result


class DualRTCTests(unittest.TestCase):
    def test_standard_zero_horizon_and_automatic_prefix_configuration(self):
        r = A10DualInferenceRunner(
            dict(protocol='fluxvla_zmq', server_host='127.0.0.1', server_port=5556),
            async_execution=True, execute_horizon=0,
            rtc_config=dict(enabled=True, method='guidance', prefix_len=None))
        r.run_setup()
        self.assertEqual(r.execute_horizon, 0)
        config = resolve_rtc_config(r.rtc_config, elapsed_s=.277, dt=1/30,
                                    available=20, action_chunk=30)
        self.assertEqual(config['prefix_len'], 10)
        self.assertEqual(config['decay_end'], 20)
        config = resolve_rtc_config(r.rtc_config, elapsed_s=32., dt=1/30,
                                    available=18, action_chunk=30)
        self.assertEqual(config['prefix_len'], 18)
        self.assertEqual(config['decay_end'], 30)

    def test_prefix_and_vjp_use_existing_model_algorithms_with_dual_padding(self):
        for method, vjp in [('prefix', False), ('guidance', True)]:
            with self.subTest(method=method):
                t, model = transform(), DualRTCPolicy()
                targets = np.full((18, 14), .25)
                batch = dict(states=torch.zeros(1, 32))
                inject_rtc(batch, dict(config=dict(RTC, method=method, use_vjp=vjp),
                                      absolute_actions=targets),
                           state=np.zeros(14), denormalizer=t, model=model)
                with torch.no_grad():
                    result = model.predict_action(**batch)
                self.assertGreater(float(result[0, 0, 0]), .2)
                self.assertEqual(tuple(result.shape), (1, 30, 32))

    def acknowledged_runner(self, clock):
        d, p = Device(), Policy()
        p.metadata = dict(rtc=dict(absolute_prefix_version=1, state_dims=[14], methods=['guidance']))
        original = p.infer
        def infer(obs):
            result = original(obs)
            prefix = obs.get('a10_rtc_actions')
            result['debug'] = dict(rtc=dict(enabled=True, method='guidance',
                applied=prefix is not None,
                prefix_len=0 if prefix is None else min(12, len(prefix))))
            return result
        p.infer = infer
        r = A10DualInferenceRunner.from_components(
            d, p, dict(SETTINGS, action_dt=.01, execute_horizon=5,
                       async_execution=True, rtc_config=RTC), Mock(), clock=clock,
            remote_inference=dict(protocol='fluxvla_zmq', server_host='127.0.0.1', server_port=5556))
        return r, d, p

    def test_rtc_over_budget_and_unapplied_prefix_never_submit_a_new_chunk(self):
        for failure in ('over_budget', 'unapplied'):
            with self.subTest(failure=failure):
                clock = Clock()
                r, d, p = self.acknowledged_runner(clock)
                r._prev_ctx = SimpleNamespace(actions=np.tile(state(d), (30, 1)),
                                               action_timestamp=0.)
                original = p.infer
                def infer(obs):
                    result = original(obs)
                    if failure == 'over_budget':
                        clock.sleep(.13)  # Twelve locked points cover only 120ms.
                    else:
                        result['debug']['rtc']['applied'] = False
                    return result
                p.infer = infer
                r.ros_operator.execute_trajectory = Mock()
                with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(RuntimeError):
                    r.run()
                r.ros_operator.execute_trajectory.assert_not_called()
                self.assertEqual(d.sent, 0)

    def test_expired_async_prediction_is_rejected_instead_of_starting_from_row_zero(self):
        clock = Clock()
        r, d, p = self.acknowledged_runner(clock)
        r.rtc_config = dict(enabled=False)
        r._prev_ctx = SimpleNamespace()
        original = p.infer
        def infer(obs):
            clock.sleep(.31)  # More than the full 30 x 10ms model horizon.
            return original(obs)
        p.infer = infer
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaisesRegex(RuntimeError, 'expired'):
            r.run()
        self.assertEqual(d.sent, 0)

    def test_joint_prefix_is_renormalized_at_new_state_and_padded_for_real_model_width(self):
        t, model = transform(), DualRTCPolicy()
        old = np.tile(np.arange(14) * .01, (18, 1))
        new_state = np.arange(14) * -.02
        batch = dict(states=torch.zeros(1, 32))
        info = inject_rtc(batch, dict(config=RTC, absolute_actions=old),
                          state=new_state, denormalizer=t, model=model)
        self.assertEqual(tuple(batch['prev_actions'].shape), (1, 18, 32))
        restored = t(dict(action=batch['prev_actions'].numpy(), state=new_state))
        np.testing.assert_allclose(restored, old, atol=1e-6)
        self.assertEqual(info['prefix_len'], 12)
        self.assertTrue(torch.all(batch['prev_actions'][:, :, 14:] == 0))

    def test_real_zmq_client_server_transports_dual_prefix_and_acknowledgement(self):
        import zmq
        from fluxvla.engines.runners.serving.zmq_server import create_server
        model = DualRTCPolicy()
        server = create_server(model, dataset=lambda obs: dict(states=torch.zeros(1, 32)),
            denormalize_action=transform(), device='cpu', host='127.0.0.1', port=0)
        address = server.socket.getsockopt_string(zmq.LAST_ENDPOINT)
        worker = threading.Thread(target=server.run)
        worker.start()
        policy = None
        try:
            policy = b.ZmqPolicy('127.0.0.1', int(address.rsplit(':', 1)[1]), timeout_s=2)
            d = RecordingDevice()
            runner = A10DualInferenceRunner.from_components(
                d, policy, dict(SETTINGS, action_dt=.01, execute_horizon=0,
                                async_execution=True, rtc_config=RTC), Mock(), cycles=4,
                remote_inference=dict(protocol='fluxvla_zmq', server_host='127.0.0.1', server_port=5556))
            with contextlib.redirect_stdout(io.StringIO()):
                runner.run()
            self.assertIsNone(model.prefixes[0])
            self.assertEqual(len(model.prefixes), 4)
            for prefix in model.prefixes[1:]:
                self.assertEqual(prefix.shape[-1], 32)
                self.assertGreater(prefix.shape[1], 12)
            records = [c.kwargs for c in runner.log.call_args_list if c.args[0] == 'rtc_guidance']
            self.assertEqual([r['applied'] for r in records], [False, True, True, True])
            self.assertEqual(records[1]['execution'], 'asynchronous_time_aligned')
            self.assertTrue(d.closed)
        finally:
            if policy is not None and not policy.socket.closed:
                policy.close()
            server.close()
            worker.join(3)
        self.assertFalse(worker.is_alive())

    def test_unsupported_servers_fail_before_starting_motion(self):
        with self.assertRaisesRegex(ValueError, 'absolute-prefix'):
            A10DualInferenceRunner(dict(protocol='longwam_http'), rtc_config=RTC)
        d = RecordingDevice()
        policy = Policy()
        policy.metadata = {}
        runner = A10DualInferenceRunner.from_components(
            d, policy, dict(SETTINGS, rtc_config=RTC), Mock(),
            remote_inference=dict(protocol='fluxvla_zmq', server_host='127.0.0.1', server_port=5556))
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaisesRegex(RuntimeError, 'advertise'):
            runner.run()
        self.assertEqual(d.sent, 0)

    def test_openpi_rtc_checks_real_capability_before_connecting_robot(self):
        d, policy = RecordingDevice(), Policy()
        policy.metadata = {'model': {'name': 'OpenPI without RTC'}}
        runner = A10DualInferenceRunner.from_components(
            d, policy, dict(SETTINGS, rtc_config=RTC, async_execution=True, execute_horizon=0), Mock(),
            remote_inference=dict(protocol='openpi_ws', server_host='127.0.0.1', server_port=8001))
        runner.ros_operator.connect = Mock()
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaisesRegex(RuntimeError, 'openpi_ws.*RTC'):
            runner.run()
        runner.ros_operator.connect.assert_not_called()
        self.assertEqual(d.sent, 0)
        self.assertFalse(d.active)

    def test_capable_server_must_acknowledge_rtc_application(self):
        d = RecordingDevice()
        policy = Policy()
        policy.metadata = dict(rtc=dict(absolute_prefix_version=1, state_dims=[14], methods=['guidance']))
        runner = A10DualInferenceRunner.from_components(
            d, policy, dict(SETTINGS, rtc_config=RTC), Mock(),
            remote_inference=dict(protocol='fluxvla_zmq', server_host='127.0.0.1', server_port=5556))
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaisesRegex(RuntimeError, 'acknowledge'):
            runner.run()
        self.assertEqual(d.sent, 0)


if __name__ == '__main__':
    unittest.main()
