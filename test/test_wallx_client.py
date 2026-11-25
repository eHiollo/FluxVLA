"""Protocol checks without credentials, network access, or hardware drivers."""
import copy
import importlib.util
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import msgpack
import msgpack_numpy
import numpy as np
from PIL import Image

spec = importlib.util.spec_from_file_location(
    "wallx_client_test_module", Path(__file__).resolve().parents[1] / "scripts/wallx_client.py")
client = importlib.util.module_from_spec(spec)
spec.loader.exec_module(client)

METADATA = {"cb_history": {"public_embodiments": {"yam": "desktop_yam/gripper"}},
            "action_horizon": 32, "row_dt_s": 0.0337, "valid_rows": 31}


def request():
    buf = io.BytesIO()
    Image.new("RGB", (16, 16)).save(buf, format="JPEG")
    state = {"position": [0.1, 0.2, 0.3], "rotation_rpy": [0., 0., 0.], "gripper": 0.5}
    return {"embodiment": "yam", "normalization": "provided", "instruction": "folding clothes",
            "observation": {"images": {name: buf.getvalue() for name in
                                       ("front", "left_wrist", "right_wrist")},
                            "state": {arm: copy.deepcopy(state) for arm in client.ARMS}}}


def response():
    return {"embodiment": "yam", "actions": {arm: {
        "position": np.zeros((32, 3)), "rotation_rpy": np.zeros((32, 3)),
        "gripper": np.zeros(32)} for arm in client.ARMS}}


def packed(value):
    return msgpack.packb(value, default=msgpack_numpy.encode, use_bin_type=True)


class WallXTests(unittest.TestCase):
    def connection(self, frames):
        ws = Mock()
        ws.response.status_code = 101
        ws.recv.side_effect = frames
        return ws

    def test_handshake_precedes_binary_request_and_numpy_response(self):
        ws = self.connection([packed(METADATA), packed(response())])
        with patch("websockets.sync.client.connect", return_value=ws) as connect:
            with client.WallXClient("unit-test-secret") as policy:
                ws.send.assert_not_called()
                ws.recv.assert_called_once_with(timeout=120)
                result = policy.infer(request())
                self.assertEqual(result["actions"]["left_ee"]["position"].shape, (32, 3))
            self.assertEqual(connect.call_args.kwargs["additional_headers"],
                             {"Authorization": "Bearer unit-test-secret"})
            self.assertIsNone(connect.call_args.kwargs["proxy"])
        sent = ws.send.call_args.args[0]
        self.assertIsInstance(sent, bytes)
        self.assertIsInstance(msgpack.unpackb(sent, raw=False)["observation"]["images"]["front"], bytes)
        ws.close.assert_called_once()

    def test_a10_rejected_before_send(self):
        ws = self.connection([packed(METADATA)])
        with patch("websockets.sync.client.connect", return_value=ws):
            with client.WallXClient("unit-test-secret") as policy:
                payload = request()
                payload["embodiment"] = "a10"
                with self.assertRaisesRegex(ValueError, "Unsupported embodiment"):
                    policy.infer(payload)
                ws.send.assert_not_called()

    def test_bad_metadata_closes_connection(self):
        for frame in ("text error", packed({"model": "wrong"}), packed({"hello": 1})):
            ws = self.connection([frame])
            with self.subTest(frame=frame), patch("websockets.sync.client.connect", return_value=ws):
                with self.assertRaises(ValueError):
                    client.WallXClient("unit-test-secret")
                ws.close.assert_called_once()

    def test_timeout_or_server_error_closes_connection(self):
        for frame in (TimeoutError("late response"), packed({"error": {"code": "busy", "message": "retry"}})):
            ws = self.connection([packed(METADATA), frame])
            with patch("websockets.sync.client.connect", return_value=ws):
                policy = client.WallXClient("unit-test-secret")
                with self.assertRaises((TimeoutError, RuntimeError)):
                    policy.infer(request())
                ws.close.assert_called_once()

    def test_nonfinite_or_wrong_action_shape_rejected(self):
        for value in (np.full((32, 3), np.nan), np.zeros((31, 3))):
            result = response()
            result["actions"]["left_ee"]["position"] = value
            with self.assertRaises(ValueError):
                client.validate_response(result, request(), METADATA)

    def test_invalid_observation_never_accepted(self):
        for mutate in (lambda p: p["observation"]["images"].pop("front"),
                       lambda p: p["observation"]["state"]["left_ee"].update(position=[float("nan"), 0, 0]),
                       lambda p: p["observation"]["state"]["left_ee"].update(gripper=1.1),
                       lambda p: p.update(norm_key="legacy"),
                       lambda p: p.update(use_rtc=True),
                       lambda p: p["observation"]["images"].update(front=b"invalid JPEG")):
            payload = request()
            mutate(payload)
            with self.assertRaises((ValueError, OSError)):
                client.validate_request(payload, METADATA)

    def test_credentials_environment_precedence_and_official_host(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "config"
            path.write_text("# literal values only\nWALLX_API_KEY=file-secret\n")
            with patch.dict("os.environ", {"WALLX_API_KEY": "env-secret", "WALLX_URL": client.URL}):
                self.assertEqual(client.credentials(path), ("env-secret", client.URL))
            with patch.dict("os.environ", {"WALLX_API_KEY": "secret", "WALLX_URL": "wss://example.com"}):
                with self.assertRaisesRegex(ValueError, "official"):
                    client.credentials(path)


if __name__ == "__main__":
    unittest.main()
