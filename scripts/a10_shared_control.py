#!/usr/bin/env python3
"""Keep one robot connection pair for concurrent console and inference clients."""
import argparse
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fluxvla.engines.communication.a10_shared_server import SharedControl, SharedControlServer
from fluxvla.engines.utils.a10_debug_log import create_a10_debug_log


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.0.125', help='Robot IP')
    parser.add_argument('--ws-port', type=int, default=5999)
    parser.add_argument('--status-ws-port', type=int, default=5888)
    parser.add_argument('--port', type=int, default=5870, help='Local sharing port (loopback only)')
    args = parser.parse_args()
    robot = SharedControl(args.host, args.ws_port, args.status_ws_port)
    server = None
    try:
        robot.connect()
        robot.recorder = create_a10_debug_log(dict(enabled=True), role='shared_control', metadata=vars(args))
        server = SharedControlServer(('127.0.0.1', args.port), robot)
        print(f'A10 shared control ready: 127.0.0.1:{args.port} -> {args.host}:{args.ws_port}', flush=True)
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        print('Stopping shared control service...', flush=True)
    except Exception as exc:
        print(f'Shared control service failed: {exc}. Exit old direct console/inference clients before starting this service.', file=sys.stderr)
        return 1
    finally:
        if server is not None:
            server.server_close()
        robot.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
