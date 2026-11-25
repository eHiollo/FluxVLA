"""Offline bridge tests: no hardware imports or device connections."""

import importlib.util
from pathlib import Path
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("bridge_test_module", ROOT / "fluxvla/projects/a10/inference.py")
b = importlib.util.module_from_spec(spec)
spec.loader.exec_module(b)
SETTINGS = dict(
    max_robot_age_s=0.15,
    max_gripper_age_s=0.2,
    max_camera_age_s=0.15,
    max_skew_s=0.15,
    max_joint_step_rad=0.2,
    execute_horizon=10,
    action_dt=1 / 30,
    max_send_lag_s=0.1,
    prompt=None,
)


class Clock:
    def __init__(self):
        self.now = 0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Policy:
    def __init__(self, action=None, error=None):
        self.action = action
        self.error = error
        self.closed = False

    def infer(self, obs):
        if self.error:
            raise self.error
        return {"actions": np.tile(obs["observation/state"], (30, 1)) if self.action is None else self.action}

    def close(self):
        self.closed = True


class Device(b.SimDevices):
    def __init__(self):
        super().__init__(lambda *a, **k: None)
        self.stops = 0
        self.closed = False
        self.fail = None

    def send(self, row):
        if self.fail and self.sent == 2:
            raise self.fail
        super().send(row)

    def stop(self):
        self.stops += 1
        return super().stop()

    def close(self):
        self.closed = True


def live_device_fixture(clock=None, query_s=0., send_s=0.):
    """Real LiveDevices logic with in-memory hardware and a controllable clock."""
    import threading
    from types import SimpleNamespace
    from unittest.mock import Mock

    d = object.__new__(b.LiveDevices)
    sample = Device().sample()
    state = SimpleNamespace(
        has_error=False, follower_active=True, follower_mode="Axis",
        raw={"ret_context": {"robot_msg": {"activate": "Enabled", "motion": "Stopped"}}},
        received_monotonic_s=b.time.monotonic(),
        arms={
            "left": SimpleNamespace(ee_id=0, actual_joints_rad=sample["joints"][0]),
            "right": SimpleNamespace(ee_id=1, actual_joints_rad=sample["joints"][1]),
        },
    )
    d.lock = threading.Lock()
    d.write_locks = {s: threading.Lock() for s in ("left", "right")}
    d.cancel, d.quit = threading.Event(), threading.Event()
    d.feedback, d.gripper_faults, d.grips = {}, {}, {}
    d.error, d.target, d._checked_state = None, None, None
    d._speed_restore = None
    d.gripper_pause_required, d.attempted, d.motion = False, False, True
    d.threads = []
    d.cfg, d.log = {"bridge": SETTINGS}, Mock()

    def get_state():
        if clock:
            clock.sleep(query_s)
        now = b.time.monotonic()
        state.received_monotonic_s = now
        d.feedback = {s: (v, now, True) for s, v in sample["grippers"].items()}
        return state

    def snapshot(_):
        return {n: dict(image=img, received_monotonic_s=b.time.monotonic())
                for n, img in sample["images"].items()}

    def start(_):
        state.follower_active = True

    def stop():
        state.follower_active = False
        return state

    d.robot = Mock()
    d.robot.get_state.side_effect = get_state
    d.robot.start_follower.side_effect = start
    d.robot.stop_follower.side_effect = stop
    if clock:
        d.robot.send_joints.side_effect = lambda _: clock.sleep(send_s)
    d.cameras = Mock()
    d.cameras.snapshot.side_effect = snapshot
    return d, state


class BridgeTests(unittest.TestCase):
    def run_bridge(self, device, policy, clock=None, **kw):
        return b.run(device, policy, SETTINGS, lambda *a, **k: None, clock=clock or Clock(), **kw)

    def test_physical_mapping_camera_and_closure(self):
        obs = b.observation(Device().sample(), SETTINGS)
        np.testing.assert_allclose(obs["observation/state"], [0.1] * 6 + [0.25] + [-0.2] * 6 + [0.75])
        for key, value in [("top", 60), ("left", 120), ("right", 180)]:
            self.assertTrue((obs["observation/images/" + key] == value).all())
        self.assertNotIn("prompt", obs)

    def test_two_cycles_and_clock(self):
        device = Device()
        policy = Policy()
        clock = Clock()
        self.run_bridge(device, policy, clock=clock)
        self.assertEqual(device.sent, 20)
        self.assertEqual(device.stops, 1)
        self.assertAlmostEqual(clock.now, 20 / 30)
        self.assertTrue(device.closed and policy.closed)

    def test_readonly_sends_nothing(self):
        d = Device()
        self.run_bridge(d, Policy(), read_only=True)
        self.assertEqual(d.sent, 0)
        self.assertEqual(d.stops, 0)

    def test_bad_response(self):
        for action in [
            np.zeros((30, 7)),
            np.full((30, 14), np.nan),
            np.full((30, 14), np.inf),
        ]:
            with self.subTest(shape=action.shape):
                d = Device()
                p = Policy(action)
                with self.assertRaises(ValueError):
                    self.run_bridge(d, p)
                self.assertEqual(d.sent, 0)
                self.assertTrue(d.closed and p.closed)

    def test_jump_first_and_later_row(self):
        for index in [0, 29]:
            d = Device()
            obs = b.observation(d.sample(), SETTINGS)
            action = np.tile(obs["observation/state"], (30, 1))
            action[index, 0] += 1
            with self.assertRaises(ValueError):
                self.run_bridge(d, Policy(action))
            self.assertEqual(d.sent, 0)

    def test_timeout_disconnect_and_server_errors(self):
        for error in [
            TimeoutError("timeout"),
            ConnectionError("closed"),
            RuntimeError("server"),
        ]:
            d = Device()
            p = Policy(error=error)
            with self.assertRaises(type(error)):
                self.run_bridge(d, p)
            self.assertEqual(d.sent, 0)
            self.assertTrue(d.closed and p.closed)

    def test_midchunk_error_or_interrupt_stops(self):
        for error in [ConnectionError("lost ownership"), KeyboardInterrupt()]:
            d = Device()
            d.fail = error
            with self.assertRaises(type(error)):
                self.run_bridge(d, Policy())
            self.assertEqual(d.sent, 2)
            self.assertEqual(d.stops, 1)
            self.assertFalse(d.active)
            self.assertTrue(d.closed)

    def test_stale_and_skew_rejected(self):
        for stamp in [-100, float("nan")]:
            sample = Device().sample()
            sample["robot_time"] = stamp
            with self.assertRaises(RuntimeError):
                b.observation(sample, SETTINGS)
        sample = Device().sample()
        sample["gripper_times"]["left"] -= 0.16
        with self.assertRaises(RuntimeError):
            b.observation(sample, SETTINGS)

    def test_clipping_only_gripper_no_second_delta(self):
        raw = np.ones((30, 14))
        raw[:, 6] = -2
        raw[:, 13] = 2
        actions, count = b.actions_from_response({"actions": raw})
        self.assertEqual(count, 60)
        self.assertTrue((actions[:, 6] == 0).all())
        self.assertTrue((actions[:, 13] == 1).all())
        self.assertTrue((actions[:, :6] == 1).all())
        self.assertTrue((raw[:, 6] == -2).all())

    def test_unconfirmed_exit_stop_is_reported(self):
        d = Device()
        d.stop = lambda: False
        with self.assertRaisesRegex(RuntimeError, "unconfirmed"):
            self.run_bridge(d, Policy())
        self.assertEqual(d.sent, 20)

    def test_30hz_retains_follower_across_predictions_without_replaying_targets(self):
        from unittest.mock import Mock

        d, policy, clock = Device(), Policy(), Clock()
        d.start = Mock(wraps=d.start)
        infer = policy.infer
        observations = []

        def predict(obs):
            observations.append((d.active, d.sent))
            clock.sleep(.5)
            return infer(obs)

        policy.infer = predict
        self.run_bridge(d, policy, clock=clock, cycles=3)
        self.assertEqual(observations, [(False, 0), (True, 10), (True, 20)])
        d.start.assert_called_once()
        self.assertEqual(d.stops, 1)
        self.assertFalse(d.active)
        self.assertEqual(d.sent, 30)

    def test_prediction_failure_after_first_chunk_stops_retained_follower(self):
        from unittest.mock import Mock

        d, policy = Device(), Policy()
        first = policy.infer(b.observation(d.sample(), SETTINGS))
        policy.infer = Mock(side_effect=[first, TimeoutError("model timeout")])
        with self.assertRaisesRegex(TimeoutError, "model timeout"):
            self.run_bridge(d, policy)
        self.assertEqual(d.sent, 10)
        self.assertEqual(d.stops, 1)
        self.assertFalse(d.active)

    def test_failed_start_still_stops(self):
        d = Device()

        def fail():
            raise TimeoutError("start unknown")

        d.start = fail
        with self.assertRaises(TimeoutError):
            self.run_bridge(d, Policy())
        self.assertEqual(d.stops, 1)
        self.assertEqual(d.sent, 0)

    def test_real_adapter_joint_mapping_and_gripper_raw_target(self):
        d, _ = live_device_fixture()
        row = np.array([1] * 6 + [0.25] + [2] * 6 + [0.75], dtype=float)
        d.check()
        d.send(row)
        received = d.robot.send_joints.call_args.args[0]
        np.testing.assert_array_equal(received["right"], [1] * 6)
        np.testing.assert_array_equal(received["left"], [2] * 6)
        np.testing.assert_array_equal(d.target[0], row)
        d.robot.get_state.assert_called_once()
        self.assertIsNone(d._checked_state)
        # A second send cannot silently reuse the previous frame's feedback.
        d.send(row)
        self.assertEqual(d.robot.get_state.call_count, 2)

    def test_delayed_or_faulted_send_rejects_checked_feedback(self):
        from unittest.mock import patch

        for failure in ("robot_stale", "gripper_stale", "camera_stale", "gripper_fault"):
            with self.subTest(failure=failure):
                clock = Clock()
                with patch.object(b.time, "monotonic", side_effect=clock.monotonic):
                    d, _ = live_device_fixture(clock)
                    d.check()
                    if failure == "robot_stale":
                        clock.sleep(SETTINGS["max_robot_age_s"] + .01)
                    elif failure == "gripper_stale":
                        d.feedback["left"] = (1000, -1., True)
                    elif failure == "camera_stale":
                        d.cameras.snapshot.side_effect = lambda _: {
                            n: dict(image=np.zeros((2, 2, 3), np.uint8), received_monotonic_s=-1.)
                            for n in b.CAMERAS
                        }
                    else:
                        d.gripper_pause_required = True
                    with self.assertRaises(RuntimeError):
                        d.send(np.zeros(14))
                    d.robot.send_joints.assert_not_called()
                    d.robot.get_state.assert_called_once()
                    self.assertIsNone(d._checked_state)

    def test_follower_faults_and_missing_mapping_send_nothing(self):
        for failure in ("alarm", "inactive", "wrong_mode", "missing_arm"):
            with self.subTest(failure=failure):
                d, state = live_device_fixture()
                if failure == "alarm":
                    state.has_error = True
                elif failure == "inactive":
                    state.follower_active = False
                elif failure == "wrong_mode":
                    state.follower_mode = "Cart"
                else:
                    state.arms["left"].ee_id = 1
                with self.assertRaises((RuntimeError, ValueError)):
                    d.send(np.zeros(14))
                d.robot.send_joints.assert_not_called()

    def test_backend_control_rejection_does_not_publish_gripper_target(self):
        d, _ = live_device_fixture()
        d.robot.send_joints.side_effect = PermissionError("control revoked")
        d.check()
        with self.assertRaisesRegex(PermissionError, "control revoked"):
            d.send(np.zeros(14))
        d.robot.get_state.assert_called_once()
        self.assertIsNone(d._checked_state)
        self.assertIsNone(d.target)

    def test_stop_invalidates_checked_feedback(self):
        d, _ = live_device_fixture()
        d.check()
        self.assertIsNotNone(d._checked_state)
        self.assertTrue(d.stop())
        self.assertIsNone(d._checked_state)

    def test_one_query_per_frame_avoids_cumulative_30hz_overrun(self):
        from unittest.mock import patch

        clock = Clock()
        with patch.object(b.time, "monotonic", side_effect=clock.monotonic):
            d, _ = live_device_fixture(clock, query_s=.014, send_s=.003)
            events = []
            settings = dict(SETTINGS, execute_horizon=30)
            b.run(d, Policy(), settings, lambda event, **data: events.append((event, data)),
                  cycles=1, clock=clock)
            # Two observations per prediction, then one query per frame.
            self.assertEqual(d.robot.get_state.call_count, 32)
            self.assertEqual(d.robot.send_joints.call_count, 30)
            sent = [data for event, data in events if event == "sent"]
            self.assertTrue(all(abs(row["check_s"] - .014) < 1e-8 for row in sent))
            self.assertTrue(all(abs(row["send_s"] - .003) < 1e-8 for row in sent))
            self.assertLess(clock.now, 1.04)

    def test_slow_feedback_still_stops_before_sending(self):
        from unittest.mock import patch

        clock = Clock()
        with patch.object(b.time, "monotonic", side_effect=clock.monotonic):
            d, _ = live_device_fixture(clock, query_s=.11)
            events = []
            with self.assertRaisesRegex(RuntimeError, "Feedback check exceeded"):
                b.run(d, Policy(), SETTINGS, lambda event, **data: events.append((event, data)),
                      cycles=1, clock=clock)
            d.robot.send_joints.assert_not_called()
            d.robot.stop_follower.assert_called_once()
            missed = next(data for event, data in events if event == "send_deadline_missed")
            self.assertAlmostEqual(missed["check_s"], .11)

    def test_gripper_worker_rate_raw_and_cancel(self):
        from types import SimpleNamespace
        import threading

        d = object.__new__(b.LiveDevices)
        writes = []
        waits = []
        d.lock = threading.Lock()
        d.write_locks = {"left": threading.Lock()}
        d.cancel = threading.Event()
        d.feedback = {}
        d.error = None
        d.gripper_faults, d.gripper_pause_required = {}, False
        row = np.zeros(14)
        row[6] = 0.25
        d.target = (row, b.time.monotonic() + 1)
        state = SimpleNamespace(position_raw=1000, has_error=False, enabled=True)
        d.grips = {
            "left": SimpleNamespace(
                connect=lambda: None,
                read_state=lambda _: state,
                set_position_raw=lambda *args, **kw: writes.append((args, kw)),
            )
        }

        class Quit:
            done = False

            def is_set(self):
                return self.done

            def wait(self, seconds):
                waits.append(seconds)
                self.done = True

        d.quit = Quit()
        d._gripper_worker("left", dict(device_id=1, speed=5, torque=60))
        self.assertIsNone(d.error)
        self.assertEqual(writes[0][0], (1, 2250))
        self.assertAlmostEqual(waits[0], 0.05, delta=0.01)
        self.assertIs(writes[0][1]["motion_cancel_event"], d.cancel)
        writes.clear()
        d.cancel.set()
        d.quit = Quit()
        d._gripper_worker("left", dict(device_id=1, speed=5, torque=60))
        self.assertEqual(writes, [])

    def test_gripper_expiry_and_stop_cancel_keep_worker_alive(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        import threading
        from fluxvla.engines.operators.zx_gripper import ZXGripperInputExpired

        for cancelled in (False, True):
            with self.subTest(cancelled=cancelled):
                d = object.__new__(b.LiveDevices)
                d.lock = threading.Lock()
                d.write_locks = {"left": threading.Lock()}
                d.cancel = threading.Event()
                d.feedback, d.error = {}, None
                d.gripper_faults, d.gripper_pause_required = {}, False
                d.log = Mock()
                row = np.zeros(14)
                d.target = (row, b.time.monotonic() + 10)
                state = SimpleNamespace(position_raw=1000, has_error=False, enabled=True)
                attempts = []

                def write(*args, **kwargs):
                    attempts.append(args[1])
                    if len(attempts) == 1:
                        if cancelled:
                            d.cancel.set()
                            d.target = None
                        raise ZXGripperInputExpired("expired during serial I/O")

                grip = SimpleNamespace(connect=Mock(), read_state=Mock(return_value=state), set_position_raw=write)
                d.grips = {"left": grip}

                class Quit:
                    ticks = 0

                    def is_set(self):
                        return self.ticks == 3

                    def wait(self, seconds):
                        self.ticks += 1
                        if self.ticks == 2:
                            # Next action chunk supplies a fresh target.
                            d.cancel.clear()
                            fresh = row.copy()
                            fresh[6] = 0.5
                            d.target = (fresh, b.time.monotonic() + 10)
                        else:
                            d.target = None

                d.quit = Quit()
                d._gripper_worker("left", dict(device_id=1, speed=5, torque=60))
                self.assertIsNone(d.error)
                self.assertEqual(grip.read_state.call_count, 3)
                self.assertEqual(attempts, [0, 4500])
                d.log.assert_called_once()
                self.assertEqual(d.log.call_args.kwargs["cancelled"], cancelled)

    def test_gripper_faults_keep_polling_and_latch_pause(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        import threading
        from fluxvla.engines.operators.zx_gripper import ZXGripperCommunicationError

        healthy = SimpleNamespace(position_raw=1000, has_error=False, enabled=True)
        alarm = SimpleNamespace(has_error=True, alarm_a=2, alarm_b=0)
        for failure in (ZXGripperCommunicationError("timeout"), alarm):
            with self.subTest(failure=failure):
                d = object.__new__(b.LiveDevices)
                d.lock = threading.Lock()
                d.write_locks = {"left": threading.Lock()}
                d.cancel = threading.Event()
                d.feedback, d.error = {}, None
                d.gripper_faults = {"right": "other fault"}
                d.gripper_pause_required = False
                d.target = (np.zeros(14), b.time.monotonic() + 10)
                d.log = Mock()
                grip = Mock()
                grip.read_state.side_effect = [failure, healthy]
                d.grips = {"left": grip}

                class Quit:
                    ticks = 0

                    def is_set(self):
                        return self.ticks == 2

                    def wait(self, seconds):
                        self.ticks += 1

                d.quit = Quit()
                d._gripper_worker("left", dict(device_id=1, speed=5, torque=60))
                self.assertIsNone(d.error)
                self.assertEqual(grip.read_state.call_count, 2)
                grip.disconnect.assert_called_once()
                grip.set_position_raw.assert_not_called()
                self.assertTrue(d.cancel.is_set())
                self.assertTrue(d.gripper_pause_required)
                self.assertIsNone(d.target)
                self.assertEqual(d.gripper_faults, {"right": "other fault"})
                with self.assertRaises(b.GripperFault):
                    d.sample()

    def test_drop_alarm_warn_continues_but_mixed_alarm_pauses(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        import threading

        for alarm_a, alarm_b, paused in [(0, 0x1000, False), (1, 0x1000, True), (0, 0x1001, True)]:
            with self.subTest(alarm_a=alarm_a, alarm_b=alarm_b):
                d = object.__new__(b.LiveDevices)
                d.lock = threading.Lock()
                d.write_locks = {"left": threading.Lock()}
                d.cancel = threading.Event()
                d.feedback, d.error, d.gripper_faults = {}, None, {}
                d.gripper_pause_required = False
                d.target = (np.zeros(14), b.time.monotonic() + 10)
                d.log = Mock()
                grip = Mock()
                grip.read_state.return_value = SimpleNamespace(
                    has_error=True, alarm_a=alarm_a, alarm_b=alarm_b, enabled=True, position_raw=1000,
                )
                d.grips = {"left": grip}

                class Quit:
                    done = False

                    def is_set(self):
                        return self.done

                    def wait(self, seconds):
                        self.done = True

                d.quit = Quit()
                d._gripper_worker("left", dict(device_id=1, speed=5, torque=60, drop_alarm_policy="warn"))
                self.assertIsNone(d.error)
                self.assertEqual(d.gripper_pause_required, paused)
                self.assertEqual(grip.set_position_raw.call_count, 0 if paused else 1)

    def test_gripper_pause_requires_fresh_feedback_and_acknowledgement(self):
        import threading
        from unittest.mock import Mock, patch

        d = object.__new__(b.LiveDevices)
        d.lock = threading.Lock()
        d.stop = Mock(return_value=True)
        d.log = Mock()
        d.error, d.gripper_faults = None, {"left": "timeout"}
        d.grips = {"left": object(), "right": object()}
        d.feedback = {}
        d.cfg = {"bridge": SETTINGS}
        d.motion, d.gripper_pause_required = True, True
        events = []

        def recover(_):
            events.append("poll")
            d.gripper_faults.clear()
            d.feedback = {s: (1000, b.time.monotonic(), True) for s in d.grips}

        def acknowledge(_):
            d.stop.assert_called_once()
            self.assertTrue(d.gripper_pause_required)
            events.append("ack")

        with patch.object(b.time, "sleep", side_effect=recover), patch("builtins.input", side_effect=acknowledge):
            d.wait_for_grippers()
        self.assertEqual(events, ["poll", "ack"])
        self.assertFalse(d.gripper_pause_required)

    def test_fault_midchunk_stops_then_repredicts_without_closing(self):
        from unittest.mock import Mock

        d = Device()
        d.fail = b.GripperFault("communication lost")

        def recover():
            self.assertGreater(d.stops, 0)
            self.assertFalse(d.closed)
            d.fail = None

        d.wait_for_grippers = Mock(side_effect=recover)
        policy = Policy()
        policy.infer = Mock(wraps=policy.infer)
        self.run_bridge(d, policy, cycles=1)
        d.wait_for_grippers.assert_called_once()
        self.assertEqual(policy.infer.call_count, 2)
        self.assertEqual(d.sent, 12)
        self.assertTrue(d.closed and policy.closed)

    def test_websocket_native_codec_and_timeout(self):
        from unittest.mock import Mock, patch

        codec = b.load_file(
            "_test_openpi_codec", ROOT.parent / "openpi/packages/openpi-client/src/openpi_client/msgpack_numpy.py"
        )
        obs = b.observation(Device().sample(), SETTINGS)
        socket = Mock()
        socket.recv.side_effect = [
            codec.packb({}),
            codec.packb({"actions": np.tile(obs["observation/state"], (30, 1))}),
            TimeoutError("recv timeout"),
        ]
        with patch("websockets.sync.client.connect", return_value=socket):
            client = b.RemotePolicy("127.0.0.1", 8001, 0.1, ROOT.parent / "openpi/packages/openpi-client/src")
            try:
                actions, _ = b.actions_from_response(client.infer(obs))
                np.testing.assert_allclose(actions[0], obs["observation/state"])
                encoded = socket.send.call_args.args[0]
                np.testing.assert_array_equal(codec.unpackb(encoded)["observation/state"], obs["observation/state"])
                with self.assertRaises(TimeoutError):
                    client.infer(obs)
            finally:
                client.close()
        socket.close.assert_called_once()


class ManualSpeedTests(unittest.TestCase):
    def device(self, original=30):
        d, state = live_device_fixture()
        state.follower_active = False
        d.cfg = {"bridge": dict(SETTINGS, manual_speed_percent=70)}
        speed = dict(mode="Manual", percent=original)
        d.robot.get_speed_override.side_effect = lambda: dict(speed)

        def set_speed(percent, *, expected_mode, expected_percent, **kwargs):
            self.assertEqual(speed["mode"], expected_mode)
            self.assertEqual(speed["percent"], expected_percent)
            speed["percent"] = percent

        d.robot.set_speed_override.side_effect = set_speed
        return d, speed

    def test_motion_sets_once_and_restores_after_all_chunks(self):
        d, speed = self.device()
        b.run(d, Policy(), SETTINGS, lambda *a, **kw: None, cycles=2, clock=Clock())
        self.assertEqual(speed["percent"], 30)
        calls = d.robot.set_speed_override.call_args_list
        self.assertEqual([call.args[0] for call in calls], [70, 30])
        self.assertTrue(calls[-1].kwargs["allow_cancelled"])
        self.assertIsNone(d._speed_restore)
        d.robot.enable.assert_not_called()

    def test_readonly_never_changes_speed(self):
        d, speed = self.device()
        d.motion = False
        b.run(d, Policy(), SETTINGS, lambda *a, **kw: None, cycles=1, read_only=True, clock=Clock())
        self.assertEqual(speed["percent"], 30)
        d.robot.get_speed_override.assert_not_called()
        d.robot.set_speed_override.assert_not_called()
        d.robot.start_follower.assert_not_called()

    def test_ctrl_c_stops_and_restores(self):
        d, speed = self.device()
        d.robot.send_joints.side_effect = KeyboardInterrupt()
        with self.assertRaises(KeyboardInterrupt):
            b.run(d, Policy(), SETTINGS, lambda *a, **kw: None, cycles=1, clock=Clock())
        d.robot.stop_follower.assert_called_once()
        self.assertEqual(speed["percent"], 30)

    def test_lost_setting_ack_restores_without_starting_motion(self):
        d, speed = self.device()
        normal_set = d.robot.set_speed_override.side_effect

        def lost_ack(percent, **kwargs):
            normal_set(percent, **kwargs)
            if percent == 70:
                raise TimeoutError("ACK lost after setting took effect")

        d.robot.set_speed_override.side_effect = lost_ack
        with self.assertRaises(TimeoutError):
            d.start()
        d.close()
        self.assertEqual(speed["percent"], 30)
        d.robot.start_follower.assert_not_called()
        d.robot.send_joints.assert_not_called()

    def test_external_change_is_preserved(self):
        for mode, percent in (("Manual", 40), ("Auto", 70)):
            with self.subTest(mode=mode, percent=percent):
                d, speed = self.device()
                d._apply_manual_speed()
                speed.update(mode=mode, percent=percent)
                with self.assertRaisesRegex(RuntimeError, "preserving site settings"):
                    d.close()
                self.assertEqual(speed, dict(mode=mode, percent=percent))
                d.robot.set_speed_override.assert_called_once()
                d.robot.close.assert_called_once()

    def test_auto_mode_does_not_change_speed_or_start(self):
        d, speed = self.device()
        speed["mode"] = "Auto"
        with self.assertRaisesRegex(RuntimeError, "requires Manual"):
            d.start()
        d.robot.set_speed_override.assert_not_called()
        d.robot.start_follower.assert_not_called()
        self.assertIsNone(d._speed_restore)

    def test_disabled_robot_does_not_change_speed_or_enable(self):
        d, speed = self.device()
        state = d.robot.get_state()
        state.raw["ret_context"]["robot_msg"]["activate"] = "Disabled"
        with self.assertRaisesRegex(RuntimeError, "Enabled/Stopped"):
            d.start()
        self.assertEqual(speed["percent"], 30)
        d.robot.set_speed_override.assert_not_called()
        d.robot.enable.assert_not_called()
        d.robot.start_follower.assert_not_called()

    def test_unconfirmed_stop_does_not_restore_speed(self):
        d, speed = self.device()
        d._apply_manual_speed()
        d.stop = lambda: False
        with self.assertRaisesRegex(RuntimeError, "stop unconfirmed"):
            d.close()
        self.assertEqual(speed["percent"], 70)
        d.robot.set_speed_override.assert_called_once()
        d.robot.close.assert_called_once()


class ConfiguredInferenceTests(unittest.TestCase):

    def test_manual_speed_percent_validation_and_config_scope(self):
        from mmengine import Config
        from fluxvla.engines import build_runner_from_cfg

        base = Config.fromfile(ROOT / "configs/a10/a10_dual_arm_operator.py")
        config = Config.fromfile(ROOT / "configs/a10/a10_dual_openpi_remote.py")
        self.assertEqual(base.operator.joint_follower_velocity_percent, 100)
        self.assertEqual(config.operator.joint_follower_velocity_percent, 12)
        self.assertEqual(config.inference.manual_speed_percent, 100)
        self.assertTrue(config.inference.follower_tail.enabled)
        config.inference.cfg = config
        runner = build_runner_from_cfg(config.inference)
        for percent in (None, 1, 70, 100):
            runner.settings["manual_speed_percent"] = percent
            runner.run_setup()
        for percent in (True, 0, -1, 101, "70", float("nan")):
            with self.subTest(percent=percent):
                runner.settings["manual_speed_percent"] = percent
                with self.assertRaisesRegex(ValueError, "manual_speed_percent"):
                    runner.run_setup()

    def test_cli_alarm_policy_validated_before_hardware(self):
        import argparse
        from mmengine import Config, DictAction
        from fluxvla.engines import build_runner_from_cfg
        from fluxvla.engines.operators.zx_gripper import ZXGripper

        parser = argparse.ArgumentParser()
        parser.add_argument('--cfg-options', nargs='+', action=DictAction)
        for right in ('warn', 'war'):
            with self.subTest(right=right):
                args = parser.parse_args([
                    '--cfg-options', 'inference.mode=read_only',
                    'grippers.left.drop_alarm_policy=warn',
                    f'grippers.right.drop_alarm_policy={right}',
                ])
                config = Config.fromfile(ROOT / 'configs/a10/a10_dual_openpi_remote.py')
                config.merge_from_dict(args.cfg_options)
                config.inference.cfg = config
                runner = build_runner_from_cfg(config.inference)
                if right == 'war':
                    with self.assertRaisesRegex(ValueError, r"grippers.right.drop_alarm_policy.*'war'"):
                        runner.run_setup()
                else:
                    runner.run_setup()
                    for device in runner.config['grippers'].values():
                        # Construct the real driver without opening a port.
                        driver = ZXGripper(device['port'], drop_alarm_policy=device['drop_alarm_policy'])
                        self.assertEqual(driver.drop_alarm_policy, 'warn')
                        self.assertIsNone(driver._serial)

    def test_preview_uses_standard_registry_without_devices(self):
        from mmengine import Config
        from fluxvla.engines import build_runner_from_cfg
        from unittest.mock import patch

        config = Config.fromfile(ROOT / "configs/a10/a10_dual_openpi_remote.py")
        config.inference.mode = "preview"
        config.inference.cfg = config
        runner = build_runner_from_cfg(config.inference)
        self.assertEqual(type(runner).__name__, "A10DualInferenceRunner")
        with (
            patch(
                "fluxvla.projects.a10.inference.LiveDevices",
                side_effect=AssertionError("hardware opened"),
            ),
            patch(
                "fluxvla.projects.a10.inference.RemotePolicy",
                side_effect=AssertionError("network opened"),
            ),
        ):
            runner.run_setup()
            runner.run()
        self.assertNotIn("cfg", runner.config["inference"])

    def test_remote_client_config_needs_no_local_model(self):
        import os
        from unittest.mock import patch
        from mmengine import Config
        from fluxvla.engines import build_runner_from_cfg

        with patch.dict(os.environ, {"A10_DUAL_CHECKPOINT": "/tmp/absent-a10-test-bundle"}):
            config = Config.fromfile(ROOT / "configs/a10/a10_dual_fold_inference.py")
        config.inference.cfg = config
        runner = build_runner_from_cfg(config.inference)
        runner.run_setup()
        self.assertEqual(runner.mode, "motion")
        self.assertEqual(runner.remote["protocol"], "fluxvla_zmq")

    def test_native_quantile_roundtrip_preserves_both_grippers(self):
        from fluxvla.transforms.a10_openpi import A10RTCDenormalizeAction

        low = np.arange(14, dtype=np.float32) / 100
        high = low + 0.2
        stats = {"a10_dual": {"action": {"q01": low, "q99": high}}}
        transform = A10RTCDenormalizeAction(stats, state_dim=14, statistic_name="a10_dual", norm_type="quantile")
        state = np.arange(14, dtype=np.float32)
        raw = np.tile((low + high) / 2, (30, 1))
        mask = ([True] * 6 + [False]) * 2
        absolute = raw.copy()
        absolute[:, mask] += state[mask]
        normalized = transform.normalize_absolute(absolute, state)
        result = transform(dict(action=np.pad(normalized, ((0, 0), (0, 18))), state=state))
        np.testing.assert_allclose(result, absolute, rtol=1e-5, atol=1e-6)
        np.testing.assert_allclose(result[:, [6, 13]], raw[:, [6, 13]])
        with self.assertRaises(ValueError):
            transform(dict(action=np.zeros((30, 7)), state=state))

    def test_native_loader_accepts_bundle_relative_weights_and_rejects_extra(self):
        import tempfile
        from unittest.mock import patch
        import torch
        from safetensors.torch import save_file
        from fluxvla.models.vlas.openpi_a10 import load_native_a10_policy

        with tempfile.TemporaryDirectory() as name:
            directory = Path(name)
            (directory / "checkpoints").mkdir()
            weights = directory / "checkpoints/model.safetensors"
            save_file({"weight": torch.ones(2, 3)}, str(weights))
            with patch(
                "fluxvla.engines.build_vla_from_cfg",
                return_value=torch.nn.Linear(3, 2, bias=False),
            ):
                model = load_native_a10_policy(directory, {}, weight_file="checkpoints/model.safetensors")
                self.assertTrue(torch.equal(model.weight, torch.ones(2, 3)))
            save_file({"weight": torch.ones(2, 3), "unexpected": torch.ones(1)}, str(weights))
            with patch(
                "fluxvla.engines.build_vla_from_cfg",
                return_value=torch.nn.Linear(3, 2, bias=False),
            ):
                with self.assertRaises(RuntimeError):
                    load_native_a10_policy(directory, {}, weight_file="checkpoints/model.safetensors")

    def test_zmq_adapter_roundtrip_and_timeout(self):
        import io
        import zmq
        import msgpack
        from unittest.mock import Mock, patch
        from fluxvla.engines.runners.serving.serializers import decode_predict_request

        obs = b.observation(Device().sample(), SETTINGS)
        data = io.BytesIO()
        np.save(data, np.tile(obs["observation/state"], (1, 30, 1)), allow_pickle=False)
        socket = Mock()
        socket.recv.side_effect = [msgpack.packb({"status": "ready"}),
                                   msgpack.packb({"action_data": data.getvalue()}), zmq.Again()]
        context = Mock()
        context.socket.return_value = socket
        with patch("zmq.Context", return_value=context):
            policy = b.ZmqPolicy("127.0.0.1", 5556, 0.2)
            try:
                self.assertNotIn("model", policy.metadata)
                result = policy.infer(obs)
                np.testing.assert_allclose(result["actions"][0], obs["observation/state"])
                _, encoded, _ = decode_predict_request(socket.send.call_args.args[0])
                np.testing.assert_array_equal(encoded["left_wrist_0_rgb"], obs["observation/images/left"])
                with self.assertRaises(zmq.Again):
                    policy.infer(obs)
            finally:
                policy.close()
        context.term.assert_called_once()

    def test_native_inputs_match_training_pipeline(self):
        import json
        import os
        from unittest.mock import patch
        from mmengine import Config
        import torch
        from fluxvla.engines import build_dataset_from_cfg, build_transform_from_cfg

        bundle = ROOT / "checkpoints/FluxVLA-A10-Dual-Fold-10steps"
        if not bundle.exists():
            self.skipTest("Model bundle not downloaded")
        with patch.dict(os.environ, {"A10_DUAL_CHECKPOINT": str(bundle)}):
            config = Config.fromfile(ROOT / "configs/a10/a10_dual_fold_inference.py")
        dataset = build_dataset_from_cfg(config.inference.dataset)
        sample = Device().sample()
        obs = dict(
            qpos=b.observation(sample, SETTINGS)["observation/state"],
            task_description=config.inference.prompt,
            **sample["images"],
        )
        actual = dataset(obs)
        stats = json.loads((bundle / "dataset_statistics.json").read_text())["a10_dual"]
        # Load the actual server training configuration with local statistics.
        stats_path = (
            ROOT.parent
            / "openpi/checkpoints/pi05_a10_dual_finetune/a10_dua_1113_fold_h30_smoke_retry/9/assets/dataset/a10_dua_1113_v21/norm_stats.json"
        )
        with patch.dict(os.environ, {"A10_DUAL_OPENPI_STATS": str(stats_path)}):
            training = Config.fromfile(ROOT / "configs/pi05/pi05_paligemma_a10_dual_fold_10steps.py")
        pipeline = training.train_dataloader.dataset.datasets.transforms[1:]
        data = dict(
            states=obs["qpos"].copy(),
            task_description=obs["task_description"],
            actions=np.tile(obs["qpos"], (30, 1)),
            images=[obs[key].transpose(2, 0, 1) for key in b.CAMERAS],
            img_masks=np.ones(3, dtype=bool),
            stats=stats,
        )
        for item in pipeline:
            item = item.to_dict() if hasattr(item, "to_dict") else dict(item)
            if item["type"] == "PrepareA10OpenPIInputs":
                item["task_description"] = obs["task_description"]
            if item["type"] == "ProcessPrompts":
                item["tokenizer"]["model_path"] = str(bundle / "tokenizer")
            data = build_transform_from_cfg(item)(data)
        for key in ("states", "images", "lang_tokens", "lang_masks"):
            np.testing.assert_allclose(actual[key].numpy()[0], np.asarray(data[key]), rtol=1e-6, atol=1e-6)
        self.assertEqual(tuple(actual["images"].shape), (1, 9, 224, 224))
        self.assertTrue(torch.equal(actual["img_masks"], torch.ones((1, 3), dtype=torch.bool)))




class MotorAccelerationPreflightTests(unittest.TestCase):
    def test_motion_preflight_runs_before_cameras_or_grippers(self):
        from unittest.mock import Mock, patch
        config = dict(operator=dict(shared_control=dict(mode='managed'),
                                    motor_acceleration_deg_s2=375), cameras={}, grippers={}, bridge={})
        events = []
        robot, cameras = Mock(), Mock()
        robot.connect.side_effect = lambda: events.append('connect')
        robot.ensure_motor_acceleration.side_effect = lambda: events.append('acceleration')
        cameras.start.side_effect = lambda: events.append('cameras')
        with patch('fluxvla.engines.operators.a10_operator.A10Operator', return_value=robot), \
             patch('fluxvla.engines.operators.camera_operator.BufferedCameraOperator', return_value=cameras), \
             patch.object(b.LiveDevices, 'sample', return_value={}), \
             patch.object(b, 'observation', return_value={}):
            b.LiveDevices(config, Mock(), motion=True)
        self.assertEqual(events, ['connect', 'acceleration', 'cameras'])

    def test_preflight_failure_blocks_camera_start_and_cleans_up(self):
        from unittest.mock import Mock, patch
        config = dict(operator=dict(shared_control=dict(mode='managed')), cameras={}, grippers={}, bridge={})
        robot, cameras = Mock(), Mock()
        robot.ensure_motor_acceleration.side_effect = RuntimeError('acceleration mismatch')
        with patch('fluxvla.engines.operators.a10_operator.A10Operator', return_value=robot), \
             patch('fluxvla.engines.operators.camera_operator.BufferedCameraOperator', return_value=cameras), \
             patch.object(b.LiveDevices, 'close') as close:
            with self.assertRaisesRegex(RuntimeError, 'acceleration mismatch'):
                b.LiveDevices(config, Mock(), motion=True)
        cameras.start.assert_not_called()
        robot.start_follower.assert_not_called()
        close.assert_called_once()

    def test_read_only_does_not_request_acceleration_changes(self):
        from unittest.mock import Mock, patch
        config = dict(operator=dict(shared_control=dict(mode='managed')), cameras={}, grippers={}, bridge={})
        robot = Mock()
        with patch('fluxvla.engines.operators.a10_operator.A10Operator', return_value=robot), \
             patch('fluxvla.engines.operators.camera_operator.BufferedCameraOperator'), \
             patch.object(b.LiveDevices, 'sample', return_value={}), \
             patch.object(b, 'observation', return_value={}):
            b.LiveDevices(config, Mock(), motion=False)
        robot.ensure_motor_acceleration.assert_not_called()


if __name__ == "__main__":
    unittest.main()
