#!/usr/bin/env python3
"""Wall-X public API probe and inference-to-file. Never opens robot hardware."""
from __future__ import annotations

import argparse
import io
import json
import math
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
URL = "wss://maas-api.x2robot.com/v1/vla/wallx"
ARMS = ("left_ee", "right_ee")
# Ranges explicitly documented in the public API. Other advertised names need
# their own confirmed state/gripper contract before this tool accepts them.
GRIPPER_MAX = {
    "quanta_x1_pro": 1.526, "quanta_x1_pro/gripper-g": 1.526,
    "quanta_x1_pro/gripper-h": 4.6, "teletwin_desktop/gripper-g": 1.526,
    "teletwin_desktop/gripper-h": 4.6, "yam": 1.0, "agilex": 1.0, "franka": 1.0,
}


def credentials(path):
    """Read only literal KEY=VALUE entries; environment takes precedence."""
    values = {}
    if path.exists():
        for line in path.read_text().splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            key, sep, value = line.partition("=")
            if not sep or key not in ("WALLX_API_KEY", "WALLX_URL"):
                raise ValueError("Credentials file accepts only WALLX_API_KEY and WALLX_URL")
            values[key] = value.strip().strip("\"'")
    key = os.environ.get("WALLX_API_KEY", values.get("WALLX_API_KEY", "")).strip()
    url = os.environ.get("WALLX_URL", values.get("WALLX_URL", URL)).strip()
    if not key or any(c.isspace() for c in key):
        raise ValueError("Set WALLX_API_KEY in the environment or .env.wallx")
    if url != URL:
        raise ValueError("This personal-key client only connects to the official Wall-X URL")
    return key, url


def public_embodiments(metadata):
    """The service currently nests this mapping under cb_history."""
    if not isinstance(metadata, dict):
        return {}
    found = metadata.get("public_embodiments")
    if isinstance(found, dict):
        return found
    for value in metadata.values():
        if isinstance(value, dict):
            found = public_embodiments(value)
            if found:
                return found
    return {}


def finite_vector(value, size, field):
    import numpy as np
    array = np.asarray(value)
    if array.shape != (size,) or array.dtype.kind not in "fiu" or not np.isfinite(array).all():
        raise ValueError(f"{field}: expected {size} finite numbers")


def validate_request(payload, metadata):
    if not isinstance(payload, dict):
        raise ValueError("Request must be a mapping")
    embodiment = payload.get("embodiment")
    supported = public_embodiments(metadata)
    if not supported:
        raise ValueError("Server did not advertise public_embodiments; cannot verify compatibility")
    if embodiment not in supported:
        raise ValueError(f"Unsupported embodiment: {embodiment!r}; A10 has no advertised adapter")
    if embodiment not in GRIPPER_MAX:
        raise ValueError(f"No documented state/gripper contract in this client for {embodiment!r}")
    if not isinstance(payload.get("instruction"), str) or not payload["instruction"].strip():
        raise ValueError("instruction must be nonempty")
    if payload.get("normalization") != "provided":
        raise ValueError('Use normalization="provided" for this public API client')
    if any(k in payload for k in ("views", "norm_key", "normalizer", "robot_id", "dof_mask")):
        raise ValueError("Legacy/custom normalization fields are not supported by this client")
    # This CLI implements synchronous requests only, without previous trajectories.
    if any(k in payload for k in ("use_rtc", "prev_action_chunk", "inference_delay", "use_history")):
        raise ValueError("This client supports independent synchronous requests only")
    arms = ("right_ee",) if embodiment == "franka" else ARMS
    views = ("front", "right_wrist") if embodiment == "franka" else ("front", "left_wrist", "right_wrist")
    obs = payload.get("observation", {})
    if not isinstance(obs, dict):
        raise ValueError("observation must be a mapping")
    images, states = obs.get("images", {}), obs.get("state", {})
    if set(images) != set(views) or set(states) != set(arms):
        raise ValueError("Expected exactly the documented camera views and arm states")
    from PIL import Image
    for name in views:
        if not isinstance(images[name], bytes):
            raise ValueError(f"{name}: expected JPEG bytes")
        with Image.open(io.BytesIO(images[name])) as image:
            if image.format != "JPEG":
                raise ValueError(f"{name}: expected JPEG")
            image.verify()
    for arm in arms:
        block = states[arm]
        finite_vector(block["position"], 3, f"{arm}.position (metres)")
        finite_vector(block["rotation_rpy"], 3, f"{arm}.rotation_rpy (radians)")
        grip = block["gripper"]
        if isinstance(grip, bool) or not isinstance(grip, (int, float)) or not math.isfinite(grip):
            raise ValueError(f"{arm}.gripper must be finite")
        maximum = GRIPPER_MAX[embodiment]
        if not 0 <= grip <= maximum:
            raise ValueError(f"{arm}.gripper outside documented range [0, {maximum}]")
    return arms


def validate_response(response, payload, metadata):
    import numpy as np
    if response.get("embodiment") != payload["embodiment"]:
        raise ValueError("Response embodiment differs from request")
    horizon = metadata.get("action_horizon", 32)
    arms = ("right_ee",) if payload["embodiment"] == "franka" else ARMS
    actions = response.get("actions", {})
    if set(actions) != set(arms):
        raise ValueError("Unexpected response arm/body fields")
    for arm in arms:
        for field, shape in (("position", (horizon, 3)), ("rotation_rpy", (horizon, 3)),
                             ("gripper", (horizon,))):
            array = np.asarray(actions[arm][field])
            if array.shape != shape or array.dtype.kind not in "fiu" or not np.isfinite(array).all():
                raise ValueError(f"{arm}.{field}: expected finite actions of shape {shape}")


class WallXClient:
    def __init__(self, api_key, url=URL, timeout=120, use_proxy=False):
        import msgpack
        import msgpack_numpy
        from websockets.sync.client import connect
        if url != URL:
            raise ValueError("Only the official Wall-X endpoint is allowed")
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be positive and finite")
        self.msgpack, self.numpy_codec, self.timeout = msgpack, msgpack_numpy, timeout
        self.ws = connect(url, additional_headers={"Authorization": "Bearer " + api_key},
                          open_timeout=30, max_size=16 * 1024 * 1024,
                          compression=None, proxy=True if use_proxy else None)
        try:
            self.http_status = self.ws.response.status_code
            self.metadata = self.decode(self.ws.recv(timeout=timeout))
            # Current production metadata has no `model` field, unlike the docs example.
            if self.metadata.get("model", "wall-x") != "wall-x":
                raise ValueError("Unexpected model in server metadata")
            if not public_embodiments(self.metadata):
                raise ValueError("Missing public embodiment capabilities in metadata")
        except BaseException:
            self.close()
            raise

    def decode(self, message):
        if not isinstance(message, bytes):
            raise ValueError("Expected binary MessagePack frame")
        result = self.msgpack.unpackb(message, raw=False, object_hook=self.numpy_codec.decode)
        if not isinstance(result, dict):
            raise ValueError("Expected MessagePack mapping")
        if "status" in result and result["status"] != "ok":
            raise RuntimeError(f"Wall-X service status: {result['status']}")
        if result.get("error"):
            error = result["error"]
            if isinstance(error, dict):
                raise RuntimeError(f"Wall-X {error.get('code', 'error')}: {error.get('message', '')}")
            raise RuntimeError(f"Wall-X error: {error}")
        return result

    def infer(self, payload):
        validate_request(payload, self.metadata)
        try:
            self.ws.send(self.msgpack.packb(payload, default=self.numpy_codec.encode, use_bin_type=True))
            response = self.decode(self.ws.recv(timeout=self.timeout))
            validate_response(response, payload, self.metadata)
            return response
        except BaseException:
            # A timeout leaves a response in flight; never reuse that connection.
            self.close()
            raise

    def close(self):
        self.ws.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def load_request(path):
    payload = json.loads(path.read_text())
    # In JSON, image values are JPEG file paths relative to the request file.
    for name, image in payload["observation"]["images"].items():
        payload["observation"]["images"][name] = (path.parent / image).read_bytes()
    return payload


def json_default(value):
    if hasattr(value, "tolist"):
        return value.tolist()
    raise TypeError(f"Cannot serialize {type(value).__name__}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--env-file", type=Path, default=ROOT / ".env.wallx")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--probe", action="store_true", help="Handshake only (default)")
    mode.add_argument("--request", type=Path, help="One public-schema JSON request with JPEG paths")
    parser.add_argument("--output", type=Path, help="Write metadata and optional response as JSON")
    parser.add_argument("--timeout", type=float, default=120)
    parser.add_argument("--use-proxy", action="store_true", help="Use environment/system proxy")
    args = parser.parse_args(argv)
    api_key = ""
    try:
        api_key, url = credentials(args.env_file)
        with WallXClient(api_key, url, args.timeout, args.use_proxy) as client:
            summary = dict(http_status=client.http_status,
                           action_horizon=client.metadata.get("action_horizon"),
                           row_dt_s=client.metadata.get("row_dt_s"),
                           public_embodiments=sorted(public_embodiments(client.metadata)),
                           a10_supported="a10" in public_embodiments(client.metadata),
                           robot_commands_sent=0)
            result = dict(summary=summary, metadata=client.metadata)
            if args.request:
                payload = load_request(args.request)
                start = time.monotonic()
                result["response"] = client.infer(payload)
                summary["inference_roundtrip_s"] = time.monotonic() - start
            if args.output:
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(json.dumps(result, default=json_default, ensure_ascii=False,
                                                  indent=2).replace(api_key, "[REDACTED]") + "\n")
            print(json.dumps(summary, ensure_ascii=False, indent=2).replace(api_key, "[REDACTED]"))
        return 0
    except Exception as exc:
        message = f"{type(exc).__name__}: {exc}"
        if api_key:
            message = message.replace(api_key, "[REDACTED]")
        print(message, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
