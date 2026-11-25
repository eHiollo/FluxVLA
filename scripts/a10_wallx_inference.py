#!/usr/bin/env python3
"""Run Wall-X dual A10 EE inference; simulate uses no hardware or cloud."""
import argparse
from datetime import datetime
import importlib.util
import json
from pathlib import Path
import sys
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main(argv=None):
    from mmengine import Config
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/a10/a10_wallx_inference.py")
    parser.add_argument("--mode", choices=("preview", "simulate", "capture", "read_only", "motion"))
    parser.add_argument("--cycles", type=int)
    parser.add_argument("--prompt")
    parser.add_argument("--env-file", type=Path, default=ROOT / ".env.wallx")
    parser.add_argument("--contract", type=Path, help="Explicit model frame/tool/gripper mapping JSON")
    args = parser.parse_args(argv)
    cfg = Config.fromfile(str(args.config), lazy_import=False).to_dict()
    settings = cfg["wallx"]
    if args.mode is not None:
        settings["mode"] = args.mode
    if args.cycles is not None:
        settings["cycles"] = args.cycles
    if args.prompt is not None:
        settings["prompt"] = args.prompt
    mode = settings["mode"]
    ex = load("_wallx_ee_execution", ROOT / "fluxvla/projects/a10/wallx_execution.py")
    geometry_path = Path(settings["geometry"])
    if not geometry_path.is_absolute():
        geometry_path = ROOT / geometry_path
    geometry = ex.g.A10WallXGeometry(geometry_path)
    if args.contract is not None:
        geometry.config["model_contract"] = json.loads(args.contract.read_text())
    ex.validate_settings(settings)
    if mode == "preview":
        print(json.dumps(dict(mode=mode, settings=settings, geometry=geometry.config,
                              hardware_opened=False, cloud_connected=False), ensure_ascii=False, indent=2))
        return 0
    # Validate the contract before opening robot devices or authenticating.
    contract = ex.simulation_contract() if mode == "simulate" else ex.ModelContract(geometry.config["model_contract"])
    if mode == "motion" and not contract.verified:
        raise ValueError("Confirm model frame/tool/gripper mapping before mode=motion")
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:8]
    folder = ROOT / settings["log_root"] / stamp
    folder.mkdir(parents=True, exist_ok=True)
    print(f"Wall-X EE log: {folder}", flush=True)
    devices = policy = None
    api_key = ""
    with (folder / "events.jsonl").open("w") as stream:
        def log(event, **fields):
            line = json.dumps(ex.bridge.plain(dict(event=event, **fields)), ensure_ascii=False)
            if api_key:
                line = line.replace(api_key, "[REDACTED]")
            stream.write(line + "\n")
            stream.flush()

        log("config", settings=settings, geometry=geometry.config, mode=mode)
        try:
            if mode == "simulate":
                devices, policy = ex.SimEEDevices(geometry), ex.SimPolicy()
            elif mode == "capture":
                devices = ex.LiveEEDevices(dict(cfg, bridge=settings), log, geometry, motion=False)
                sample = devices.sample()
                payload, geom = ex.observation(sample, geometry, contract, settings)
                capture_dir = folder / "capture"
                capture_dir.mkdir()
                for view, data in payload["observation"]["images"].items():
                    image_path = capture_dir / (view + ".jpg")
                    image_path.write_bytes(data)
                    payload["observation"]["images"][view] = image_path.name
                (capture_dir / "request.json").write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n")
                log("capture", geometry=geom, model_state=payload["observation"]["state"],
                    robot_time=sample["robot_time"], camera_times=sample["camera_times"],
                    gripper_times=sample["gripper_times"], cloud_requests_sent=0)
                devices.close()
                devices = None
                print(f"A10_WALLX_CAPTURE_OK {capture_dir} (cloud requests=0, robot targets=0)", flush=True)
                return 0
            else:
                client = load("_a10_wallx_client", ROOT / "scripts/wallx_client.py")
                api_key, url = client.credentials(args.env_file)
                policy = client.WallXClient(api_key, url, settings["timeout_s"])
                if contract.embodiment not in client.public_embodiments(policy.metadata):
                    raise ValueError("Configured embodiment is absent from cloud metadata")
                devices = ex.LiveEEDevices(dict(cfg, bridge=settings), log, geometry, motion=mode == "motion")
            # run() owns cleanup after both resources have been constructed.
            owned_devices, owned_policy = devices, policy
            devices = policy = None
            result = ex.run(owned_devices, owned_policy, geometry, contract, settings, log,
                            cycles=settings["cycles"], read_only=mode == "read_only")
            log("complete", **result)
            print("A10_WALLX_EE_OK " + json.dumps(result), flush=True)
            return 0
        except Exception as exc:
            message = f"{type(exc).__name__}: {exc}"
            if api_key:
                message = message.replace(api_key, "[REDACTED]")
            log("error", error=message)
            print(message, file=sys.stderr)
            return 1
        except BaseException as exc:
            log("error", error=type(exc).__name__)
            raise
        finally:
            try:
                if devices is not None:
                    devices.close()
            finally:
                if policy is not None:
                    policy.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("Wall-X EE interrupted; cleanup requested", file=sys.stderr)
        raise SystemExit(130)
