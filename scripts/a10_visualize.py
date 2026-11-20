"""Serve controller state to a local read-only dual A10 viewer."""

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import runpy
import sys
import threading
import time


HERE = Path(__file__).resolve().parent
DEFAULT_CONFIG = HERE.parent / 'configs/a10/a10_visualizer.py'

# Use the transport from this checkout even when another checkout is installed.
sys.path.insert(0, str(HERE.parent))


def _vectors(values, rows, width, field):
    if not isinstance(values, list) or len(values) != rows:
        raise ValueError(f'{field} 应有 {rows} 组')
    for vector in values:
        if (not isinstance(vector, list) or len(vector) != width
                or any(type(x) not in (int, float) or not math.isfinite(x) for x in vector)):
            raise ValueError(f'{field} 每组应有 {width} 个有限数值')
    return values


def _pose(values, position_scale):
    norm = math.hypot(*values[3:])
    if norm < 1e-8 or not math.isfinite(norm):
        raise ValueError('回包中的姿态四元数无效')
    return [x * position_scale for x in values[:3]] + [x / norm for x in values[3:]]


def _rotate(xyz, quat):
    x, y, z = xyz
    qx, qy, qz, qw = quat
    tx = 2 * (qy * z - qz * y)
    ty = 2 * (qz * x - qx * z)
    tz = 2 * (qx * y - qy * x)
    return (x + qw * tx + qy * tz - qz * ty,
            y + qw * ty + qz * tx - qx * tz,
            z + qw * tz + qx * ty - qy * tx)


def _world_point(pose, offset):
    return [p + d for p, d in zip(pose[:3], _rotate(offset, pose[3:]))]


def decode_frame(reply, settings):
    context = reply.get('ret_context', reply)
    if isinstance(context, str):
        context = json.loads(context)
    motion = context['motion_msg']
    robot = context.get('robot_msg', {})
    log = context.get('log_msg', {})
    follower = context.get('follower_msg', {})
    driver_codes = context.get('driver_msg', {}).get('driver_err_code', [])
    pos_scale = {'mm': 0.001, 'm': 1.}[settings['position_unit']]
    to_deg = {'deg': 1., 'rad': 180 / math.pi}[settings['joint_unit']]
    actual = _vectors(motion.get('actual_pos'), 2, 6, 'actual_pos')
    target = _vectors(motion.get('motor_pos'), 2, 6, 'motor_pos')
    tcp = _vectors(motion.get('pq'), 2, 7, 'pq')
    parts = motion.get('part_pq')
    if not isinstance(parts, list) or len(parts) != 2:
        raise ValueError('part_pq 应有两组连杆位姿')
    arms = []
    for side, group in enumerate(parts):
        poses = [_pose(p, pos_scale) for p in _vectors(group, 7, 7, f'part_pq[{side}]')]
        joints = [_world_point(poses[parent], marker)
                  for parent, marker in settings['joint_markers']]
        arms.append(dict(id=side, base_m=poses[0][:3], joints_m=joints,
                         tcp_pq_m=_pose(tcp[side], pos_scale),
                         offset_m=settings['display_offsets_m'][side],
                         actual_deg=[x * to_deg for x in actual[side]],
                         target_deg=[x * to_deg for x in target[side]]))
    return dict(arms=arms, mode=robot.get('op_mode'), motor=robot.get('activate'),
                motion=robot.get('motion'), controller_status=robot.get('status'),
                follower_active=follower.get('running_state'),
                follower_mode=follower.get('follower_mode'),
                fault=bool(log.get('has_error') or robot.get('cs_error_code') or any(driver_codes)),
                fault_code=robot.get('cs_error_code'), fault_message=robot.get('cs_error_msg'),
                driver_codes=driver_codes)


class ViewerState:
    def __init__(self, source, mode, stale_after):
        self.source = source
        self.mode = mode
        self.stale_after = stale_after
        self.lock = threading.Lock()
        self.frame = self.updated = self.error = None

    def publish(self, frame):
        with self.lock:
            self.frame, self.updated, self.error = frame, time.monotonic(), None

    def failed(self, error):
        with self.lock:
            self.error = str(error)

    def snapshot(self):
        with self.lock:
            age = None
            if self.updated is not None and self.mode == 'live':
                age = time.monotonic() - self.updated
            stale = self.mode == 'live' and (
                self.error is not None or age is None or age > self.stale_after)
            return dict(source=self.source, mode=self.mode, frame=self.frame, error=self.error,
                        stale=stale, age_ms=None if age is None else round(age * 1000))


def read_live(connection_factory, state, settings, stopped):
    interval = 1 / settings['poll_hz']
    while not stopped.is_set():
        connection = None
        try:
            connection = connection_factory()
            connection.connect()
            while not stopped.is_set():
                started = time.monotonic()
                reply = connection.request('get', status=True)
                try:
                    state.publish(decode_frame(reply, settings))
                except (KeyError, TypeError, ValueError, AttributeError) as exc:
                    state.failed(f'反馈格式异常：{exc}')
                stopped.wait(max(0., interval - (time.monotonic() - started)))
        except Exception as exc:
            state.failed(f'{type(exc).__name__}: {exc}')
        finally:
            if connection is not None:
                connection.close()
        stopped.wait(1.)


def connection_for(settings, shared_port):
    host = settings['host']
    status = settings['status_port']
    cartesian = settings['cartesian_udp_port']
    joint = settings['joint_udp_port']
    timeout = settings['timeout_s']
    if shared_port is not None:
        from fluxvla.engines.communication.a10_shared_connection import SharedFollowerConnection
        return SharedFollowerConnection(
            host, status, cartesian, timeout, joint,
            shared=dict(host='127.0.0.1', port=shared_port, role='console'),
            status_channel=True,
        )
    from fluxvla.engines.operators.a10_operator import _FollowerConnection
    return _FollowerConnection(host, status, cartesian, timeout, joint)


def replay_frames(path, settings):
    data = json.loads(path.read_text(encoding='utf-8'))
    entries = data if isinstance(data, list) else [data]
    if not entries:
        raise ValueError('回放文件没有帧')
    return [decode_frame(entry.get('state', entry), settings) for entry in entries]


def serve_page(page, state):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/':
                payload, media = page, 'text/html; charset=utf-8'
            elif self.path == '/state':
                payload = json.dumps(state.snapshot(), ensure_ascii=False).encode()
                media = 'application/json; charset=utf-8'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', media)
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            try:
                self.wfile.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, _format, *args):
            pass

    return Handler


def main(argv=None):
    parser = argparse.ArgumentParser(description='A10 双臂只读可视化')
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--host', help='控制器 IP 或主机名')
    parser.add_argument('--status-port', type=int, help='GET WebSocket 端口')
    parser.add_argument('--listen-port', type=int, help='网页的本机端口')
    parser.add_argument('--poll-hz', type=float, help='GET 轮询或回放频率')
    source = parser.add_mutually_exclusive_group()
    source.add_argument('--replay', type=Path, help='GET 回包 JSON，支持单帧或帧数组')
    source.add_argument('--shared-port', type=int, help='复用正在运行的本机 A10 状态服务')
    args = parser.parse_args(argv)
    try:
        settings = dict(runpy.run_path(str(args.config))['viewer'])
        for key in ('host', 'status_port', 'listen_port', 'poll_hz'):
            if getattr(args, key) is not None:
                settings[key] = getattr(args, key)
        if not math.isfinite(settings['poll_hz']) or not 0 < settings['poll_hz'] <= 30:
            raise ValueError('poll_hz 应在 (0, 30] 内')
        if not math.isfinite(settings['stale_after_s']) or settings['stale_after_s'] <= 0:
            raise ValueError('stale_after_s 应为正数')
        for port in (settings['listen_port'], settings['status_port'], args.shared_port):
            if port is not None and not 1 <= port <= 65535:
                raise ValueError('端口应在 1–65535 内')
        mode = 'replay' if args.replay else 'live'
        label = args.replay.name if args.replay else f"{settings['host']}:{settings['status_port']}"
        state = ViewerState(label, mode, settings['stale_after_s'])
        frames = replay_frames(args.replay, settings) if args.replay else None
        page = (HERE / 'a10_visualizer/index.html').read_bytes()
        server = ThreadingHTTPServer(('127.0.0.1', settings['listen_port']), serve_page(page, state))
    except (OSError, ValueError, TypeError, KeyError, AttributeError) as exc:
        parser.exit(2, f'启动失败：{exc}\n')

    stopped = threading.Event()

    def replay():
        for frame in frames[1:]:
            if stopped.wait(1 / settings['poll_hz']):
                return
            state.publish(frame)

    if frames:
        state.publish(frames[0])
        worker = threading.Thread(target=replay, daemon=True)
    else:
        worker = threading.Thread(
            target=read_live,
            args=(lambda: connection_for(settings, args.shared_port), state, settings, stopped),
            daemon=True,
        )
    worker.start()
    print(f'浏览器打开 http://127.0.0.1:{settings["listen_port"]} · {label}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stopped.set()
        server.server_close()
        worker.join(settings['timeout_s'] + 0.5)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
