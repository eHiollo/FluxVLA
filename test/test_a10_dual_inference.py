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
        self.assertEqual(device.stops, 2)
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

    def test_unconfirmed_stop_never_next_cycle(self):
        d = Device()
        d.stop = lambda: False
        with self.assertRaisesRegex(RuntimeError, "unconfirmed"):
            self.run_bridge(d, Policy())
        self.assertEqual(d.sent, 10)

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
        from types import SimpleNamespace
        import threading

        d = object.__new__(b.LiveDevices)
        received = []
        d.robot = SimpleNamespace(
            get_state=lambda: SimpleNamespace(
                arms={
                    "left": SimpleNamespace(ee_id=0),
                    "right": SimpleNamespace(ee_id=1),
                }
            ),
            send_joints=lambda x: received.append(x),
        )
        d.lock = threading.Lock()
        row = np.array([1] * 6 + [0.25] + [2] * 6 + [0.75], dtype=float)
        d.send(row)
        np.testing.assert_array_equal(received[0]["right"], [1] * 6)
        np.testing.assert_array_equal(received[0]["left"], [2] * 6)
        np.testing.assert_array_equal(d.target[0], row)

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
        row = np.zeros(14)
        row[6] = 0.25
        d.target = (row, b.time.monotonic() + 1)
        state = SimpleNamespace(position_raw=1000, has_error=False, enabled=True)
        d.grips = {
            "left": SimpleNamespace(
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


class ConfiguredInferenceTests(unittest.TestCase):
    def test_preview_uses_standard_registry_without_devices(self):
        from mmengine import Config
        from fluxvla.engines import build_runner_from_cfg
        from unittest.mock import patch

        config = Config.fromfile(ROOT / "configs/a10/a10_dual_openpi_remote.py")
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
        self.assertEqual(runner.mode, "preview")
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
        socket.recv.side_effect = [msgpack.packb({"action_data": data.getvalue()}), zmq.Again()]
        context = Mock()
        context.socket.return_value = socket
        with patch("zmq.Context", return_value=context):
            policy = b.ZmqPolicy("127.0.0.1", 5556, 0.2)
            try:
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
            actions=np.tile(obs["qpos"], (30, 1)),
            images=[obs[key].transpose(2, 0, 1) for key in b.CAMERAS],
            img_masks=np.ones(3, dtype=bool),
            stats=stats,
        )
        for item in pipeline:
            item = item.to_dict() if hasattr(item, "to_dict") else dict(item)
            if item["type"] == "ProcessPrompts":
                item["tokenizer"]["model_path"] = str(bundle / "tokenizer")
            data = build_transform_from_cfg(item)(data)
        for key in ("states", "images", "lang_tokens", "lang_masks"):
            np.testing.assert_allclose(actual[key].numpy()[0], np.asarray(data[key]), rtol=1e-6, atol=1e-6)
        self.assertEqual(tuple(actual["images"].shape), (1, 9, 224, 224))
        self.assertTrue(torch.equal(actual["img_masks"], torch.ones((1, 3), dtype=torch.bool)))


if __name__ == "__main__":
    unittest.main()
