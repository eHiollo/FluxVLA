#!/usr/bin/env python3
"""Preview calibrated A10 grasp poses; only GET is sent to the controller."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import importlib.util
import json
from pathlib import Path
import socket
import struct
import sys
import uuid

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("a10_wallx_geometry", ROOT / "fluxvla/projects/a10/wallx.py")
geometry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geometry)


def read_feedback(host="192.168.0.150", port=5999, shared_port=5870):
    """Reuse a running relay, otherwise open a GET-only status connection.

    No relay auto-start, motion lease, coordinate/tool change, or UDP traffic.
    An existing relay's failures do not trigger a second controller connection.
    """
    try:
        sock = socket.create_connection(("127.0.0.1", shared_port), 1.)
    except ConnectionRefusedError:
        sock = None
    if sock is not None:
        with sock:
            sock.settimeout(5.)
            with sock.makefile("rb") as reader:
                common = dict(session_id=uuid.uuid4().hex, role="inference",
                              status_channel=True, timeout_s=3.)

                def rpc(**fields):
                    sock.sendall((json.dumps(dict(common, **fields)) + "\n").encode())
                    response = json.loads(reader.readline(1 << 20))
                    if "error" in response:
                        raise RuntimeError(response["error"])
                    return response.get("result")

                rpc(op="hello", robot_host=host, ws_port=port, joint_udp_port=9999, udp_port=9998)
                reply = rpc(op="request", command="get", status=True, raw=False)
        source = "existing_shared_status_channel"
    else:
        from websockets.sync.client import connect
        header = struct.Struct("<IIQqqq")
        with connect(f"ws://{host}:{port}", proxy=None, open_timeout=3., close_timeout=1.,
                     ping_interval=None, max_size=(1 << 20) + header.size) as ws:
            ws.send(header.pack(3, 1, 0x1000, 0xA1B2C3D4, 0, 0) + b"get")
            packet = ws.recv(timeout=3.)
            if not isinstance(packet, bytes) or len(packet) < header.size:
                raise ValueError("Expected binary KAANH GET response")
            size = header.unpack_from(packet)[0]
            if size != len(packet) - header.size:
                raise ValueError("Invalid KAANH GET response length")
            reply = json.loads(packet[header.size:])
        source = "direct_get_only"
    geometry.feedback_context(reply)
    return dict(captured_utc=datetime.now(timezone.utc).isoformat(), source=source, reply=reply)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--geometry", type=Path, default=ROOT / "configs/a10/a10_wallx_geometry.json")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--live-read-only", action="store_true", help="Read one current GET frame")
    source.add_argument("--feedback", type=Path, help="Replay a saved GET JSON; opens no sockets")
    parser.add_argument("--host", default="192.168.0.150")
    parser.add_argument("--port", type=int, default=5999)
    parser.add_argument("--shared-port", type=int, default=5870)
    parser.add_argument("--output", type=Path, default=ROOT / "work_dirs/wallx/a10_geometry_preview.json")
    args = parser.parse_args(argv)
    try:
        adapter = geometry.A10WallXGeometry(args.geometry)
        snapshot = (read_feedback(args.host, args.port, args.shared_port) if args.live_read_only
                    else json.loads(args.feedback.read_text()))
        result = adapter.observation_geometry(snapshot)
        result["feedback_mode"] = "live_read_only" if args.live_read_only else "offline_replay"
        result["snapshot"] = snapshot
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
        summary = {key: value for key, value in result.items() if key != "snapshot"}
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        return 0
    except Exception as exc:
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
