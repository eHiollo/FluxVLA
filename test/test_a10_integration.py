"""A10Operator integration with loopback follower, fake cameras and USB servo."""

import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from fluxvla.engines.operators.a10_operator import A10Operator
from test_a10_operator import FakeController
from test_gripper_operator import FakeSerial


class A10PeripheralTest(unittest.TestCase):
    def setUp(self):
        self.serial = FakeSerial()
        self.capture = MagicMock()
        self.capture.isOpened.return_value = True
        self.capture.read.return_value = (True, np.full((6, 8, 3), [10, 20, 30], dtype=np.uint8))
        for name, value in [('serial.Serial', self.serial),
                            ('cv2.VideoCapture', self.capture)]:
            mock = patch(name, return_value=value)
            mock.start()
            self.addCleanup(mock.stop)
        self.controller = FakeController().__enter__()
        self.addCleanup(self.controller.__exit__, None, None, None)

    def operator(self, **overrides):
        options = dict(command_mode='joint', gripper_transport='usb',
                       gripper=dict(port='/fake/gripper', timeout=.015),
                       cameras=dict(base_0_rgb=dict(device='/fake/base'),
                                    right_wrist_0_rgb=dict(device='/fake/wrist')))
        options.update(overrides)
        return self.controller.operator(**options)

    def test_real_feedback_images_units_and_independent_times(self):
        operator = self.operator()
        self.serial.follow = False
        operator.send_gripper({'right': 50})
        frame = operator.get_frame()
        self.assertEqual(frame['gripper_mm'], 40)  # Actual, not the sent 50.
        self.assertEqual(frame['qpos'][-1], 40)
        np.testing.assert_allclose(frame['qpos'][:6], [0, np.pi/2, -np.pi/2, np.pi, 0, np.pi/4])
        np.testing.assert_array_equal(frame['base_0_rgb'][0, 0], [30, 20, 10])
        self.assertEqual(frame['right_wrist_0_rgb'].dtype, np.uint8)
        self.assertLessEqual(frame['received_monotonic_s'], frame['gripper_received_monotonic_s'])
        self.assertLessEqual(frame['gripper_received_monotonic_s'], frame['camera_received_monotonic_s'])
        self.assertIsNone(operator.get_state().qpos)  # Controller-only unchanged.
        self.assertFalse(any(c.startswith('g_move') for c in self.controller.commands))
        operator.close()
        self.assertFalse(self.serial.is_open)
        self.capture.release.assert_called()
        self.assertFalse(operator.is_connected)

    def test_usb_wait_range_and_read_only_gate(self):
        operator = self.operator()
        self.assertAlmostEqual(operator.send_gripper({'right': 55}, wait=True).position_mm, 55, delta=.05)
        count = len(self.serial.writes)
        with self.assertRaises(ValueError):
            operator.send_gripper({'right': 86})
        self.assertEqual(count, len(self.serial.writes))
        operator.allow_motion = False
        with self.assertRaises(PermissionError):
            operator.send_gripper({'right': 40})
        self.assertEqual(count, len(self.serial.writes))

    def test_serial_connect_failure_releases_network(self):
        self.serial.override = b''
        operator = A10Operator(host='127.0.0.1',
            ws_port=self.controller.server.socket.getsockname()[1],
            gripper_transport='usb', gripper=dict(port='/fake/gripper'))
        with self.assertRaises(Exception):
            operator.connect()
        self.assertFalse(operator.is_connected)
        self.assertFalse(self.serial.is_open)

    def test_sensor_failure_is_not_a_synthetic_frame(self):
        for sensor in ('usb', 'camera'):
            with self.subTest(sensor=sensor):
                self.serial.is_open = True
                self.serial.override = None
                self.capture.isOpened.return_value = True
                self.capture.read.return_value = (True, np.zeros((6, 8, 3), dtype=np.uint8))
                operator = self.operator()
                if sensor == 'usb':
                    self.serial.override = b''
                else:
                    self.capture.read.return_value = (False, None)
                with self.assertRaises(Exception):
                    operator.get_frame()
                self.assertFalse(operator.is_connected)
                self.assertFalse(self.serial.is_open)

    def test_invalid_configuration_no_io(self):
        for options in (dict(gripper_transport='usb'),
                        dict(gripper_transport='controller', gripper={}),
                        dict(cameras={'qpos': dict(device=0)})):
            with self.subTest(options=options), self.assertRaises(ValueError):
                A10Operator(host='127.0.0.1', **options)

    def test_close_attempts_every_resource_on_error(self):
        operator = self.operator()
        with patch.object(operator.camera_operator, 'close', side_effect=OSError('camera cleanup')):
            with self.assertRaisesRegex(RuntimeError, 'cleanup failed'):
                operator.close()
        self.assertFalse(operator.is_connected)
        self.assertFalse(self.serial.is_open)


class A10TrajectoryTest(unittest.TestCase):
    setUp = A10PeripheralTest.setUp
    operator = A10PeripheralTest.operator

    def started_operator(self, **options):
        operator = self.operator(**options)
        operator.start_follower()
        return operator

    def execute(self, operator, count=3, **overrides):
        options = dict(arm_trajectories={'right': np.full((count, 6), .1)},
                       gripper_trajectories={'right': np.arange(count) + 40.}, dt=.04)
        options.update(overrides)
        return operator.execute_trajectory(**options)

    def test_paced_loopback_rows_and_usb_packets(self):
        import json
        import struct
        import time
        operator = self.started_operator()
        times, packets = [], []
        original = operator.send_joints
        def send(target):
            times.append(time.monotonic())
            original(target)
        with patch.object(operator, 'send_joints', side_effect=send):
            before = time.monotonic()
            self.execute(operator)
            elapsed = time.monotonic() - before
        for _ in range(3):
            packets.append(self.controller.joint_udp.recv(1024))
        self.assertGreaterEqual(elapsed, .115)
        self.assertTrue(all(t >= .02 for t in np.diff(times)))
        for packet in packets:
            length, *fields = struct.unpack('<IIQqqq', packet[:40])
            self.assertEqual(fields, [0, 0, 0, 0, 0])
            self.assertEqual(length, len(packet) - 40)
            values = json.loads(packet[40:].decode().split('=', 1)[1])
            np.testing.assert_allclose(values, np.rad2deg([.1] * 6))
        targets = [p for p in self.serial.writes if p[4] == 0x83]
        self.assertEqual(len(targets), 3)
        self.assertEqual([int.from_bytes(p[8:10], 'little') for p in targets],
                         [operator.gripper_operator.position_to_raw(x) for x in (40, 41, 42)])
        self.assertEqual(operator.last_trajectory['sent'], 3)
        self.assertTrue(operator.last_trajectory['completed'])
        self.assertFalse(operator.last_trajectory['physical_arrival_confirmed'])
        self.assertFalse(operator.is_trajectory_running())
        self.assertTrue(self.controller.active)  # Runner/caller owns stop.
        operator.stop_follower()

    def test_whole_chunk_validation_sends_nothing(self):
        operator = self.started_operator()
        initial_writes = len(self.serial.writes)
        bad_joint = np.zeros((3, 6)); bad_joint[-1, 2] = np.nan
        bad_grip = np.array([40., 40., 86.])
        for options in (dict(arm_trajectories={'right': bad_joint}),
                        dict(arm_trajectories={'right': np.zeros((3, 7))}),
                        dict(arm_trajectories={'left': np.zeros((3, 6))}),
                        dict(gripper_trajectories={'right': bad_grip}),
                        dict(gripper_trajectories={'right': [40]}),
                        dict(gripper_trajectories={'right': [40, float('nan'), 40]}),
                        dict(dt=0), dict(async_exec=True), dict(head_trajectory=[0])):
            with self.subTest(options=options), self.assertRaises((ValueError, NotImplementedError)):
                self.execute(operator, **options)
        with self.assertRaises(TimeoutError):
            self.controller.joint_udp.recv(1024)
        self.assertEqual(initial_writes, len(self.serial.writes))

    def test_unowned_cartesian_and_legacy_gripper_are_rejected(self):
        operator = self.operator()
        with self.assertRaisesRegex(RuntimeError, 'owned joint'):
            self.execute(operator)
        operator.start_follower(mode='cartesian')
        with self.assertRaisesRegex(RuntimeError, 'owned joint'):
            self.execute(operator)
        operator.stop_follower()
        operator = self.controller.operator(command_mode='joint')
        operator.start_follower()
        with self.assertRaisesRegex(NotImplementedError, 'USB'):
            self.execute(operator)
        operator.stop_follower()

    def test_feedback_fault_or_mode_loss_stops_future_sends(self):
        for failure in ('fault', 'mode', 'enabled', 'disconnect', 'usb'):
            with self.subTest(failure=failure):
                self.serial.is_open = True
                self.serial.override = None
                self.controller.fault = None
                self.controller.driver_fault = False
                self.controller.activation = 'Enabled'
                self.controller.active = False
                operator = self.started_operator(trajectory_feedback_interval_s=.02)
                original = operator.send_joints
                def send(target):
                    original(target)
                    if failure == 'fault': self.controller.driver_fault = True
                    if failure == 'mode': self.controller.mode = 'Cart'
                    if failure == 'enabled': self.controller.activation = 'Disabled'
                    if failure == 'disconnect': self.controller.fault = 'disconnect'
                    if failure == 'usb': self.serial.override = b''
                with patch.object(operator, 'send_joints', side_effect=send):
                    with self.assertRaises(Exception):
                        self.execute(operator)
                self.assertEqual(operator.last_trajectory['sent'], 1)
                self.assertFalse(operator.last_trajectory['completed'])
                self.controller.joint_udp.recv(1024)
                with self.assertRaises(TimeoutError): self.controller.joint_udp.recv(1024)
                with self.assertRaises((RuntimeError, ConnectionError)):
                    operator.send_joints({'right': [0] * 6})
                operator.close()

    def test_slow_write_aborts_without_burst(self):
        import time
        operator = self.started_operator()
        original = operator.gripper_operator.set_position_mm
        def slow(*args, **kwargs):
            result = original(*args, **kwargs)
            time.sleep(.05)
            return result
        with patch.object(operator.gripper_operator, 'set_position_mm', side_effect=slow):
            with self.assertRaisesRegex(TimeoutError, 'time budget'):
                self.execute(operator, dt=.02)
        self.assertEqual(operator.last_trajectory['sent'], 1)
        self.controller.joint_udp.recv(1024)
        with self.assertRaises(TimeoutError): self.controller.joint_udp.recv(1024)
        operator.stop_follower()

    def test_cancel_and_exclusive_target_stream(self):
        import threading
        operator = self.started_operator()
        sent = threading.Event()
        errors = []
        original = operator.send_joints
        def send(target):
            original(target)
            sent.set()
        def run():
            try: self.execute(operator, count=10, dt=.05)
            except BaseException as exc: errors.append(exc)
        with patch.object(operator, 'send_joints', side_effect=send):
            thread = threading.Thread(target=run)
            thread.start()
            try:
                self.assertTrue(sent.wait(1))
                self.assertTrue(operator.is_trajectory_running())
                with self.assertRaisesRegex(RuntimeError, 'owns'):
                    operator.send_gripper({'right': 30})
                with self.assertRaisesRegex(RuntimeError, 'already running'):
                    self.execute(operator)
                operator.stop_trajectory()
            finally:
                operator._traj_stop_event.set()
                thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertEqual(len(errors), 1)
        self.assertIsInstance(errors[0], InterruptedError)
        self.assertFalse(self.controller.active)
        self.assertFalse(operator.is_trajectory_running())
        self.assertLess(operator.last_trajectory['sent'], 10)

    def test_arm_only_and_final_fault_check(self):
        operator = self.started_operator()
        original = operator.send_joints
        def fault(target):
            original(target)
            self.controller.driver_fault = True
        with patch.object(operator, 'send_joints', side_effect=fault):
            with self.assertRaisesRegex(RuntimeError, 'fault'):
                self.execute(operator, count=1, gripper_trajectories=None)
        self.assertEqual(operator.last_trajectory['sent'], 1)
        self.assertFalse(operator.last_trajectory['completed'])
        self.assertFalse(any(p[4] == 0x83 for p in self.serial.writes))


class A10RunnerIntegrationTest(unittest.TestCase):
    setUp = A10PeripheralTest.setUp

    def make_runner(self, *, dry_run=False, policy_failure=False):
        import queue
        import threading
        import torch
        import zmq
        from fluxvla.engines import build_runner_from_cfg
        from fluxvla.engines.runners.serving.zmq_server import create_server

        self.predictions = []
        observed = self.predictions
        class Policy(torch.nn.Module):
            def predict_action(self, states, **kwargs):
                observed.append(states.numpy().copy())
                if policy_failure and len(observed) > 1:
                    raise RuntimeError('simulated inference failure')
                delta = torch.tensor([.01] * 6 + [.5])
                return (states + delta).reshape(1, 1, 7).repeat(1, 4, 1)
        def preprocess(obs):
            for name in ('base_0_rgb', 'right_wrist_0_rgb'):
                self.assertEqual(obs[name].shape, (6, 8, 3))
                self.assertEqual(obs[name].dtype, np.uint8)
            self.assertEqual(obs['task_description'], 'Reach the yellow lemon')
            return dict(states=torch.as_tensor(obs['qpos']))
        ready = queue.Queue()
        def serve():
            server = create_server(Policy(), dataset=preprocess,
                host='127.0.0.1', port=0, device='cpu', mixed_precision_dtype=torch.float32)
            ready.put(server)
            server.run()
        thread = threading.Thread(target=serve)
        thread.start()
        server = ready.get(timeout=5)
        self.addCleanup(thread.join, 2)
        self.addCleanup(server.close)
        endpoint = server.socket.getsockopt_string(zmq.LAST_ENDPOINT)
        runner = build_runner_from_cfg(dict(type='A10InferenceRunner',
            operator=dict(type='A10Operator', host='127.0.0.1',
                ws_port=self.controller.server.socket.getsockname()[1],
                udp_port=self.controller.udp.getsockname()[1],
                joint_udp_port=self.controller.joint_udp.getsockname()[1],
                command_mode='joint', allow_motion=not dry_run,
                gripper_transport='usb', gripper=dict(port='/fake/gripper', timeout=.015),
                cameras=dict(base_0_rgb=dict(device='/fake/base'),
                             right_wrist_0_rgb=dict(device='/fake/wrist'))),
            remote_inference=dict(server_host='127.0.0.1', server_port=int(endpoint.rsplit(':', 1)[1]),
                                  timeout_s=2., serializer='msgpack', compress=True),
            action_chunk=4, execute_horizon=2, action_dt=.04,
            max_publish_step=5, dry_run=dry_run, max_joint_step_rad=.1,
            instruction='Reach the yellow lemon', enable_mixed_precision=False))
        self.addCleanup(runner.cleanup)
        runner._poll_keyboard_pause = lambda: False
        return runner

    def test_real_runner_operator_and_zmq_complete_loop(self):
        runner = self.make_runner()
        runner.run_setup()  # Actual scripts/inference.py entry sequence.
        runner.run()
        self.assertEqual(len(self.predictions), 3)
        np.testing.assert_allclose([float(p[6]) for p in self.predictions], [40., 40.5, 41.], atol=.1)
        self.assertEqual(runner.last_cycle['steps'], 5)
        self.assertEqual(runner.last_cycle['executed'], 1)
        self.assertEqual(len([p for p in self.serial.writes if p[4] == 0x83]), 5)
        for _ in range(5):
            packet = self.controller.joint_udp.recv(1024)
            self.assertTrue(packet[40:].startswith(b'jp --motor_pos=['))
        self.assertEqual(self.controller.commands.count('follower_aj'), 1)
        self.assertEqual(self.controller.commands.count('stop_follower'), 1)
        self.assertFalse(self.controller.active)
        self.assertFalse(runner.ros_operator.is_connected)
        self.assertFalse(self.serial.is_open)
        self.capture.release.assert_called()
        self.assertFalse(any(c == 'manual_en' for c in self.controller.commands))

    def test_dry_run_is_read_only_with_real_interfaces(self):
        runner = self.make_runner(dry_run=True)
        runner.run()
        self.assertEqual(runner.last_cycle['executed'], 0)
        self.assertEqual(len(self.predictions), 3)
        self.assertTrue(all(p[4] in (1, 2) for p in self.serial.writes))
        self.assertTrue(all(c == 'get' for c in self.controller.commands))
        with self.assertRaises(TimeoutError): self.controller.joint_udp.recv(1024)

    def test_policy_failure_stops_owned_follower_and_closes(self):
        runner = self.make_runner(policy_failure=True)
        with self.assertRaisesRegex(RuntimeError, 'inference failure'):
            runner.run()
        self.assertEqual(self.controller.commands.count('stop_follower'), 1)
        self.assertFalse(self.controller.active)
        self.assertFalse(runner.ros_operator.is_connected)
        self.assertFalse(self.serial.is_open)
        self.assertEqual(len([p for p in self.serial.writes if p[4] == 0x83]), 2)

    def test_sensor_failure_keeps_control_connection_for_explicit_stop(self):
        runner = self.make_runner()
        original = runner.ros_operator.execute_trajectory
        def fail_after_chunk(**kwargs):
            original(**kwargs)
            self.capture.read.return_value = (False, None)
        with patch.object(runner.ros_operator, 'execute_trajectory', side_effect=fail_after_chunk):
            with self.assertRaisesRegex(RuntimeError, 'camera read failed'):
                runner.run()
        self.assertEqual(self.controller.commands.count('stop_follower'), 1)
        self.assertFalse(self.controller.active)
        self.assertFalse(runner.ros_operator.is_connected)
        self.assertFalse(self.serial.is_open)


if __name__ == '__main__':
    unittest.main()
