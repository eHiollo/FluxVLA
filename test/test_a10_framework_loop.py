"""Framework parity and receive-time diagnostics; no hardware or external server."""

import tempfile
import json
import io
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import torch

from test_a10_dual_inference import b, SETTINGS, Clock, Device, Policy, live_device_fixture
from fluxvla.engines.runners.base_inference_runner import BaseInferenceRunner
from fluxvla.engines.runners.aloha_inference_runner import AlohaInferenceRunner
from fluxvla.engines.runners.a10_inference_runner import A10DualInferenceRunner
from fluxvla.engines.operators.a10_dual_operator import A10DualOperator
from fluxvla.engines.utils.a10_execution_trace import TailExecutionTrace


def components(device=None, policy=None, clock=None, **settings):
    return A10DualInferenceRunner.from_components(
        device or Device(), policy or Policy(), dict(SETTINGS, **settings),
        Mock(), cycles=2, clock=clock or Clock(), bridge=SimpleNamespace(**vars(b)))


class FrameworkParityTests(unittest.TestCase):
    def test_actual_aloha_and_a10_use_same_phases_and_synchronous_timing(self):
        traces = []
        for kind in ('aloha', 'a10'):
            clock, d, events = Clock(), Device(), []
            policy = Policy()
            if kind == 'a10':
                runner = components(d, policy, clock)
            else:
                def execute(left, right, *, dt, async_exec, **kwargs):
                    self.assertFalse(async_exec)
                    d.start()
                    for l, r in zip(left, right):
                        d.send(np.r_[l, r])
                        clock.sleep(dt)

                operator = SimpleNamespace(execute_trajectory=execute)
                with patch('fluxvla.engines.runners.base_inference_runner.build_operator_from_cfg', return_value=operator), \
                        patch.object(BaseInferenceRunner, '_init_zmq_client'):
                    runner = AlohaInferenceRunner(operator={'type': 'fake'}, remote_inference={},
                                                  action_chunk=30, publish_rate=30,
                                                  execute_horizon=10, enable_mixed_precision=False)
                runner._poll_keyboard_pause = lambda: False
                runner.update_observation_window = lambda: dict(
                    qpos=b.observation(d.sample(), SETTINGS)['observation/state'])
                def predict(**inputs):
                    clock.sleep(.326)
                    return torch.as_tensor(policy.infer({'observation/state': inputs['qpos']})['actions'])
                runner.vla = SimpleNamespace(predict_action=predict)
            for name in ('_preprocess', '_predict_action', '_postprocess_actions', '_execute_actions'):
                original = getattr(runner, name)
                def wrapped(*args, original=original, name=name, **kwargs):
                    events.append(name)
                    if kind == 'a10' and name == '_predict_action':
                        clock.sleep(.326)
                    return original(*args, **kwargs)
                setattr(runner, name, wrapped)
            for cycle in range(2):
                if kind == 'a10':
                    runner._cycle = runner.ros_operator.cycle = cycle
                runner._run_action_chunk('fold')
            traces.append((events, clock.now, d.sent))
            if kind == 'a10':
                runner.cleanup()
        self.assertEqual(traces[0][0], traces[1][0])
        self.assertAlmostEqual(traces[0][1], traces[1][1])
        self.assertAlmostEqual(traces[0][1], 2 * (.326 + 10 / 30))
        self.assertEqual(traces[0][2], 20)
        self.assertEqual(traces[1][2], 20)

    def test_shared_pause_resets_context_and_stops_at_chunk_boundary(self):
        runner = components()
        runner._poll_keyboard_pause = lambda: True
        runner.run()
        self.assertEqual(runner.ros_operator.devices.sent, 10)
        self.assertEqual(runner._cycle, 1)
        self.assertIsNone(runner._prev_ctx)
        self.assertIsNone(runner.observation_window)

    def test_instruction_and_observation_window_use_base_contract(self):
        policy = Policy()
        policy.infer = Mock(wraps=policy.infer)
        runner = components(policy=policy)
        runner._run_action_chunk('custom instruction')
        self.assertEqual(policy.infer.call_args.args[0]['prompt'], 'custom instruction')
        self.assertEqual(runner.observation_window[-1]['qpos'].shape, (14,))
        self.assertEqual(runner._prev_ctx.instruction, 'custom instruction')
        self.assertEqual(runner.get_observation_statistics()['state_dim'], 14)
        runner.reset_inference_history()
        self.assertIsNone(runner.observation_window)
        runner.cleanup()

    def test_preview_does_not_open_hardware_or_policy(self):
        with patch('fluxvla.projects.a10.inference.LiveDevices') as hardware, \
                patch('fluxvla.projects.a10.inference.RemotePolicy') as policy:
            runner = A10DualInferenceRunner(dict(protocol='openpi_ws', server_host='127.0.0.1', server_port=8001))
            runner.run()
            hardware.assert_not_called()
            policy.assert_not_called()

    def test_production_simulate_writes_complete_async_log_without_hardware(self):
        policy = Policy()
        policy.metadata = {'model': {'config': 'pi05', 'checkpoint_name': '100k'}}
        with tempfile.TemporaryDirectory() as root, \
                patch('fluxvla.projects.a10.inference.RemotePolicy', return_value=policy), \
                patch('fluxvla.projects.a10.inference.LiveDevices') as hardware:
            runner = A10DualInferenceRunner(
                dict(protocol='openpi_ws', server_host='127.0.0.1', server_port=8001),
                mode='simulate', cycles=2, execute_horizon=2, log_root=root)
            runner.run()
            hardware.assert_not_called()
            directory = next(Path(root).iterdir())
            events = [json.loads(s) for s in (directory / 'events.jsonl').read_text().splitlines()]
            identity = json.loads((directory / 'model.json').read_text())
            self.assertEqual(identity, dict(backend='openpi_ws', server='openpi_ws://127.0.0.1:8001',
                                            source='server', model=policy.metadata['model']))
            self.assertEqual(next(e for e in events if e['event'] == 'connected')['model_identity'], identity)
            counts = {kind: sum(e['event'] == kind for e in events)
                      for kind in ('sent', 'complete', 'tail_execution', 'chunk_boundary')}
            self.assertEqual(counts, dict(sent=4, complete=1, tail_execution=2, chunk_boundary=1))
            summary = json.loads((directory / 'summary.json').read_text())
            self.assertEqual(summary['dropped_events'], 0)
            self.assertEqual(summary['write_errors'], 0)
            self.assertTrue(policy.closed)

    def test_existing_live_feedback_queries_log_correct_physical_arm_mapping(self):
        device, _ = live_device_fixture()
        runner = components(device, clock=time, execute_horizon=2)
        runner._run_action_chunk('fold')
        records = [c for c in runner.log.call_args_list if c.args[0] == 'feedback']
        self.assertGreaterEqual(len(records), 4)
        actual = records[-1].kwargs['actual_joints_rad']
        np.testing.assert_allclose(actual, [.1] * 6 + [-.2] * 6)
        runner.cleanup()

    def test_base_ros_episode_dispatches_through_shared_chunk_method(self):
        runner = object.__new__(BaseInferenceRunner)
        runner.publish_rate, runner.max_publish_step = 30, 20
        runner.execute_horizon = 10
        runner._get_user_task_instruction = lambda _: ['fold', 'fold']
        runner._run_action_chunk = Mock(return_value=(np.zeros((30, 14)), False))
        ros = SimpleNamespace(Rate=lambda hz: hz, is_shutdown=lambda: False)
        with patch.dict('sys.modules', rospy=ros):
            runner._run_episode('fold')
        self.assertEqual(runner._run_action_chunk.call_count, 2)

    def test_standard_configuration_aliases_do_not_create_another_clock(self):
        remote = dict(protocol='openpi_ws', server_host='127.0.0.1', server_port=8001)
        runner = A10DualInferenceRunner(remote, publish_rate=30, action_chunk=30,
                                        state_dim=14, async_execution=False)
        self.assertEqual(runner.publish_rate, 30)
        asynchronous = A10DualInferenceRunner(remote, async_execution=True)
        self.assertTrue(asynchronous.async_execution)
        for kwargs in (dict(publish_rate=60), dict(async_execution='yes'), dict(action_dt=0)):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                A10DualInferenceRunner(remote, **kwargs)

    def test_operator_rejects_mispartitioned_arms_before_starting(self):
        d = Device()
        d.start = Mock(wraps=d.start)
        runner = components(d)
        with self.assertRaisesRegex(ValueError, r'\[T,6\]'):
            runner.ros_operator.execute_trajectory(
                {'left': np.zeros((2, 7)), 'right': np.zeros((2, 5))},
                {'left': np.zeros(2), 'right': np.zeros(2)})
        d.start.assert_not_called()
        self.assertEqual(d.sent, 0)
        runner.cleanup()


class TailTraceTests(unittest.TestCase):
    def setUp(self):
        self.events = []
        self.trace = TailExecutionTrace(lambda event, **kw: self.events.append((event, kw)))
        self.trace.observe(np.zeros(12), 0.)
        self.trace.arm(3, np.zeros(14), 0.)

    def test_measured_stable_arrival_is_separate_from_send_completion(self):
        for k in range(1, 14):
            self.trace.observe(np.zeros(12), k / 30)
        self.trace.finish('next_chunk', .5)
        result = self.events[-1][1]
        self.assertFalse(result['censored'])
        self.assertAlmostEqual(result['first_in_tolerance_s'], 1 / 30)
        self.assertGreaterEqual(result['settled_s'], .3)
        self.assertTrue(result['physical_arrival_confirmed'])

    def test_slow_robot_is_censored_when_new_chunk_replaces_target(self):
        for k in range(1, 8):
            self.trace.observe(np.full(12, np.deg2rad(2)), k / 30)
        self.trace.finish('next_chunk', .3)
        result = self.events[-1][1]
        self.assertTrue(result['censored'])
        self.assertIsNone(result['settled_s'])
        self.assertAlmostEqual(result['last_error_deg'], 2.)
        self.assertAlmostEqual(result['observation_window_s'], .3)

    def test_feedback_gap_and_duplicate_samples_cannot_prove_settling(self):
        self.trace.observe(np.zeros(12), .033)
        self.trace.observe(np.zeros(12), .5)
        for _ in range(10):
            self.trace.observe(np.zeros(12), .5)
        self.trace.finish('run_exit', .51)
        self.assertIsNone(self.events[-1][1]['settled_s'])
        self.assertEqual(self.events[-1][1]['samples'], 2)

    def test_reversal_breaks_stability(self):
        self.trace.observe(np.zeros(12), .033)
        self.trace.observe(np.full(12, np.deg2rad(.1)), .066)
        self.trace.finish('next_chunk', .1)
        self.assertIsNone(self.events[-1][1]['settled_s'])


class InferenceSamplingTests(unittest.TestCase):
    def sampler_device(self):
        device = Device()
        device.robot = SimpleNamespace(timeout_s=.2)
        device.sending = False
        count = []
        def read():
            self.assertFalse(device.sending)
            count.append(threading.get_ident())
            return SimpleNamespace()
        device._robot_state = Mock(side_effect=read)
        return device, count

    def test_sampler_only_runs_during_prediction_and_is_joined_before_send(self):
        d, count = self.sampler_device()
        policy = Policy()
        original = policy.infer
        policy.infer = lambda obs: (time.sleep(.08), original(obs))[1]
        runner = components(d, policy, time, trace_inference_feedback=True)
        send = d.send
        def checked_send(row):
            d.sending = True
            try:
                self.assertFalse(any(t.name == 'a10-inference-measurement' for t in threading.enumerate()))
                send(row)
            finally:
                d.sending = False
        d.send = checked_send
        runner.run()
        self.assertGreaterEqual(len(count), 4)
        self.assertFalse(any(t.name == 'a10-inference-measurement' for t in threading.enumerate()))

    def test_query_failure_prevents_any_new_target(self):
        d, _ = self.sampler_device()
        d._robot_state.side_effect = TimeoutError('feedback unavailable')
        runner = components(d, trace_inference_feedback=True)
        with self.assertRaisesRegex(ConnectionError, 'Diagnostic feedback'):
            runner.run()
        self.assertEqual(d.sent, 0)
        self.assertTrue(d.closed)

    def test_disabled_sampler_adds_no_queries(self):
        d, count = self.sampler_device()
        runner = components(d)
        runner.run()
        self.assertEqual(count, [])

    def test_model_return_timestamp_excludes_sampler_shutdown_time(self):
        from contextlib import contextmanager

        clock = Clock()
        runner = components(clock=clock)
        infer = runner._policy.infer
        runner._policy.infer = lambda obs: (clock.sleep(.326), infer(obs))[1]
        @contextmanager
        def delayed_join():
            yield
            clock.sleep(.012)
        runner.ros_operator.inference_feedback = delayed_join
        runner._run_action_chunk('fold')
        returned = next(c.kwargs for c in runner.log.call_args_list if c.args[0] == 'inference_end')
        ready = next(c.kwargs for c in runner.log.call_args_list if c.args[0] == 'inference_feedback_end')
        self.assertAlmostEqual(returned['request_s'], .326)
        self.assertAlmostEqual(ready['sampler_shutdown_s'], .012)
        runner.cleanup()

    def test_interrupt_during_inference_joins_diagnostic_worker(self):
        d, _ = self.sampler_device()
        policy = Policy(error=KeyboardInterrupt())
        runner = components(d, policy=policy, trace_inference_feedback=True)
        with self.assertRaises(KeyboardInterrupt):
            runner.run()
        self.assertFalse(any(t.name == 'a10-inference-measurement' for t in threading.enumerate()))
        self.assertTrue(d.closed and policy.closed)

    def test_second_prediction_feedback_failure_stops_owned_follower(self):
        d, _ = self.sampler_device()
        def read():
            if d.sent:
                raise TimeoutError('late feedback failure')
        d._robot_state.side_effect = read
        runner = components(d, trace_inference_feedback=True)
        with self.assertRaises(ConnectionError):
            runner.run()
        self.assertEqual(d.sent, 10)
        self.assertEqual(d.stops, 1)
        self.assertFalse(d.active)


class TransportLoopbackTests(unittest.TestCase):
    def test_native_zmq_preserves_observation_and_absolute_action_contract(self):
        import zmq
        from fluxvla.engines.runners.serving.serializers import (
            MsgSerializer, decode_predict_request, encode_predict_response)

        ctx = zmq.Context()
        sock = ctx.socket(zmq.REP)
        sock.setsockopt(zmq.RCVTIMEO, 3000)
        sock.setsockopt(zmq.LINGER, 0)
        port = sock.bind_to_random_port('tcp://127.0.0.1')
        received, errors = [], []
        model = {'checkpoint_dir': '/tmp/fluxvla-checkpoint'}
        def serve():
            try:
                self.assertEqual(MsgSerializer.from_bytes(sock.recv())['endpoint'], 'get_status')
                sock.send(MsgSerializer.to_bytes({'status': 'ready', 'model': model}))
                for _ in range(2):
                    fmt, obs, _ = decode_predict_request(sock.recv())
                    received.append(obs)
                    data = io.BytesIO()
                    np.save(data, np.tile(obs['qpos'], (30, 1)), allow_pickle=False)
                    sock.send(encode_predict_response(data.getvalue(), .001, fmt=fmt))
            except Exception as exc:
                errors.append(exc)
            finally:
                sock.close()
        worker = threading.Thread(target=serve, daemon=True)
        worker.start()
        policy = b.ZmqPolicy('127.0.0.1', port, timeout_s=2.)
        runner = components(policy=policy)
        try:
            runner.run('native instruction')
        finally:
            policy.close()
            worker.join(4)
            ctx.term()
        self.assertFalse(errors)
        self.assertEqual(policy.metadata['model'], model)
        self.assertEqual(len(received), 2)
        self.assertEqual(received[0]['task_description'], 'native instruction')
        np.testing.assert_allclose(received[0]['qpos'], [.1] * 6 + [.25] + [-.2] * 6 + [.75])
        self.assertEqual(received[0]['base_0_rgb'].shape, (224, 224, 3))
        self.assertEqual(runner.ros_operator.devices.sent, 20)

    def test_openpi_websocket_runs_the_same_framework_loop(self):
        from websockets.sync.server import serve
        codec = b.load_file('_framework_test_codec', b.ROOT.parent / 'openpi/packages/openpi-client/src/openpi_client/msgpack_numpy.py')
        received = []
        def handler(ws):
            ws.send(codec.packb({'test': 'metadata'}))
            for _ in range(2):
                obs = codec.unpackb(ws.recv())
                received.append(obs)
                ws.send(codec.packb({'actions': np.tile(obs['observation/state'], (30, 1))}))
        server = serve(handler, '127.0.0.1', 0)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        policy = b.RemotePolicy('127.0.0.1', server.socket.getsockname()[1], timeout_s=2.,
                                client_root=b.ROOT.parent / 'openpi/packages/openpi-client/src')
        runner = components(policy=policy)
        try:
            runner.run('openpi instruction')
        finally:
            policy.close()
            server.shutdown()
            worker.join(3)
        self.assertEqual(len(received), 2)
        self.assertEqual(received[0]['prompt'], 'openpi instruction')
        self.assertEqual(set(received[0]), {'observation/state', 'observation/images/top',
                                         'observation/images/left', 'observation/images/right', 'prompt'})
        self.assertEqual(runner.ros_operator.devices.sent, 20)


if __name__ == '__main__':
    unittest.main()
