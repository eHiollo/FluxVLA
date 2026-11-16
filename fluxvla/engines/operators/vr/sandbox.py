"""VR-only comparison of mapping targets. No robot runtime or hardware IO.

Synthetic feedback is the previous virtual command, not simulated robot dynamics.
"""
import copy
from collections import deque
from dataclasses import asdict
import math
import ipaddress
import json
import subprocess
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

from .control import ReferenceConfig, VRPoseController, pose
from .mapper import XLeVRDeltaEEMapper

MODES = ('anchor_relative', 'world_translation_local_rotation')
# WebXR world: +X right, +Y up, -Z forward. Table: +X forward, +Y left, +Z up.
TABLE_FROM_VR_WORLD = [[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]]
# Shared table frame: +X forward, +Y left, +Z up (right-handed).
# Positions are illustrative virtual base spacing, not measured mounting geometry.
BASE_FRAMES = {
    'left': dict(x='forward', y='left', z='up', handedness='right',
                 rotation_table_from_base=[[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
                 origin_table_m=[0., .25, 0.]),
    'right': dict(x='backward', y='right', z='up', handedness='right',
                  rotation_table_from_base=[[-1., 0., 0.], [0., -1., 0.], [0., 0., 1.]],
                  origin_table_m=[0., -.25, 0.]),
}
# Columns are tool +X (up), +Y (right), +Z (forward) in table coordinates.
PRESETS = {'horizontal': [[0., 0., 1.], [0., -1., 0.], [1., 0., 0.]],
           'down': [[1., 0., 0.], [0., -1., 0.], [0., 0., -1.]]}


def sandbox_access_urls(network):
    """Prefer loopback for this PC; list real LAN interfaces instead of proxy routes."""
    host, port = network['host_ip'], network['https_port']
    if host != '0.0.0.0':
        base = f'https://{host}:{port}'
        return base + '/vr-sandbox.html', [base + '/']
    addresses = []
    private = tuple(ipaddress.ip_network(net) for net in ('10.0.0.0/8', '172.16.0.0/12', '192.168.0.0/16'))
    try:
        result = subprocess.run(['ip', '-j', '-4', 'address', 'show', 'up'],
                                capture_output=True, text=True, check=True, timeout=2)
        for interface in json.loads(result.stdout):
            if interface.get('operstate') != 'UP' or interface.get('ifname', '').startswith(
                    ('lo', 'docker', 'veth', 'br-', 'virbr', 'tailscale', 'tun', 'tap', 'wg')):
                continue
            for info in interface.get('addr_info', []):
                value = info.get('local')
                if info.get('family') == 'inet' and value and any(
                        ipaddress.ip_address(value) in subnet for subnet in private):
                    url = f'https://{value}:{port}/'
                    if url not in addresses:
                        addresses.append(url)
    except (OSError, subprocess.SubprocessError, ValueError, TypeError):
        pass
    return f'https://127.0.0.1:{port}/vr-sandbox.html', addresses


class VRSandbox:
    """One VR snapshot feeds independent old/new pipelines, without transports."""

    def __init__(self, vr_input, mapping_by_hand, bindings, reference_options=None, *, fps=90):
        if set(mapping_by_hand) != {'left', 'right'} or set(bindings) != {'left', 'right'}:
            raise ValueError('Sandbox requires both physical hands and explicit bindings')
        self.input = vr_input
        self.bindings = dict(bindings)
        self.fps = fps
        self.config = ReferenceConfig(**dict(reference_options or {}))
        if not self.config.limit_acceleration:
            raise ValueError('Sandbox release requires acceleration limiting')
        self._lock = threading.Lock()
        self._pending_preset = None
        self._generation = 0
        self._preset = 'horizontal'
        self._previous_tick = None
        self._ticks = deque(maxlen=180)
        self._input_ticks = {hand: deque(maxlen=180) for hand in bindings}
        self._sequences = {hand: -1 for hand in bindings}
        self._release_required = {hand: False for hand in bindings}
        self._pipelines = {}
        for mode in MODES:
            self._pipelines[mode] = {}
            for hand, options in mapping_by_hand.items():
                options = dict(options, anchor_translation_frame=(
                    'robot_base' if mode == MODES[1] else 'tool_anchor'))
                if mode == MODES[1]:
                    # World axes differ from the old tool-local axis remap. Convert
                    # WebXR world into the table first, then into this arm's base.
                    table_from_base = np.asarray(BASE_FRAMES[hand]['rotation_table_from_base'])
                    table_from_vr = np.asarray(TABLE_FROM_VR_WORLD)
                    options['position_axis_remap'] = (table_from_base.T @ table_from_vr).tolist()
                mapper = XLeVRDeltaEEMapper(**options)
                if (mapper.position_control_mode != 'safe_frame_delta'
                        or mapper.motion_shaping_mode != 'robot_controller' or not mapper.compute_anchor_shadow):
                    raise ValueError('Sandbox requires safe_frame_delta, anchor output and shaping bypass')
                controller = VRPoseController(self.config, pose_mapping=mode)
                self._pipelines[mode][hand] = dict(mapper=mapper, controller=controller, state='holding')
        self._reset('horizontal')
        self._latest = dict(running=False, hands={}, modes={}, notice='等待 VR 输入')
        self.input.sandbox_view = self

    def _reset(self, preset):
        self._preset = preset
        table_from_tool = np.asarray(PRESETS[preset])
        for contexts in self._pipelines.values():
            for hand, ctx in contexts.items():
                table_from_base = np.asarray(BASE_FRAMES[hand]['rotation_table_from_base'])
                quaternion = Rotation.from_matrix(table_from_base.T @ table_from_tool).as_quat()
                initial = np.r_[[0., 0., .4], quaternion]
                ctx['mapper'].reset()
                ctx['controller'].start(initial)
                ctx.update(state='holding', trail=deque(maxlen=300), last_trail=-math.inf)
        self._release_required = {hand: False for hand in self.bindings}
        self._generation += 1

    @staticmethod
    def _rate(times):
        return (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 and times[-1] > times[0] else 0.

    def request_reset(self, preset):
        if preset not in PRESETS:
            raise ValueError('preset must be down or horizontal')
        with self._lock:
            if any(hand.get('active') for hand in self._latest['hands'].values()):
                raise RuntimeError('请先松开两侧侧键和前扳机，再重置虚拟姿态')
            self._pending_preset = preset
        return dict(queued=True, preset=preset)

    def snapshot(self):
        with self._lock:
            result = copy.deepcopy(self._latest)
        now = time.monotonic()
        result.update(sandbox_only=True, transmitted=False,
                      feedback='previous_virtual_command', preset=self._preset,
                      generation=self._generation, configured_fps=self.fps,
                      reference_options=asdict(self.config),
                      world_frame=dict(handedness='right', x='forward', y='left', z='up'),
                      base_frames=copy.deepcopy(BASE_FRAMES),
                      tool_frame=dict(x='up', y='right', z='forward'),
                      tool_forward_axis='+Z',
                      table_from_vr_world=copy.deepcopy(TABLE_FROM_VR_WORLD))
        result['snapshot_age_s'] = max(0., now - result.get('tick_time_s', now))
        for hand in result['hands'].values():
            stamp = hand.get('received_s')
            hand['age_ms'] = max(0., now - stamp) * 1000 if stamp is not None else None
            if (stamp is None or now - stamp > hand['age_limit_s']
                    or result['snapshot_age_s'] > self.config.max_dt_s or not result['running']):
                hand.update(fresh=False, state='stale')
        return result

    def _hand(self, raw, hand, now):
        stamp = raw.get('xlevr.sample_receive_time_s')
        sequence = raw.get('xlevr.sample_sequence')
        pressed = bool(raw.get('xlevr.grip_active') or raw.get('xlevr.buttons', {}).get('squeeze'))
        mapper = self._pipelines[MODES[0]][hand]['mapper']
        trigger = self._number(raw.get('xlevr.trigger', 0.))
        fine_active = trigger is not None and trigger >= mapper.fine_trigger_threshold
        active = pressed or fine_active
        rotation_enabled = not (mapper.squeeze_translation_only and pressed and not fine_active)
        limit = min(mapper.stale_timeout_s, self.config.max_input_age_s)
        hand_pose = None
        try:
            hand_pose = pose(np.r_[raw.get('xlevr.target_position'), raw.get('xlevr.orientation_quat')]).tolist()
        except (ValueError, TypeError):
            pass
        stamp_valid = (not isinstance(stamp, bool) and isinstance(stamp, (int, float))
                       and math.isfinite(stamp) and 0 <= stamp <= now)
        fresh = (stamp_valid and now - stamp <= limit and hand_pose is not None
                 and raw.get('xlevr.verification_tracking') is True
                 and type(sequence) is int and sequence >= 0)
        if fresh and sequence != self._sequences[hand]:
            self._input_ticks[hand].append(stamp)
            if sequence < self._sequences[hand]:
                self._input_ticks[hand].clear()
                self._release_required[hand] = True
            self._sequences[hand] = sequence
        return dict(grip=pressed, active=active, rotation_enabled=rotation_enabled, fresh=bool(fresh), received_s=float(stamp) if stamp_valid else None,
                    age_limit_s=limit, sequence=sequence if type(sequence) is int else None,
                    input_hz=self._rate(self._input_ticks[hand]), pose=hand_pose,
                    logical_arm=self.bindings[hand], trigger=trigger,
                    buttons={k: bool(v) for k, v in raw.get('xlevr.buttons', {}).items()
                             if isinstance(v, (bool, int, float))},
                    stick={k: self._number(v) for k, v in raw.get('xlevr.thumbstick', {}).items()
                           if k in ('x', 'y')})

    @staticmethod
    def _number(value):
        return float(value) if isinstance(value, (int, float)) and math.isfinite(value) else None

    def _pause(self, hand):
        for contexts in self._pipelines.values():
            ctx = contexts[hand]
            ctx['mapper'].reset()
            ctx['controller'].start(ctx['controller'].command_pose.copy())
            ctx['state'] = 'paused'

    def step(self, raw=None, *, now=None):
        """Read both hands once; injectable snapshots/time support offline tests."""
        raw = self.input.get_hand_actions() if raw is None else copy.deepcopy(raw)
        # The receive worker may publish a sample during get_hand_actions().
        # Take the tick clock after reading so that fresh data cannot look future-dated.
        now = time.monotonic() if now is None else now
        dt = 1. / self.fps if self._previous_tick is None else now - self._previous_tick
        self._previous_tick = now
        self._ticks.append(now)
        hands = {hand: self._hand(raw.get(hand, {}), hand, now) for hand in self.bindings}
        notice = ''
        with self._lock:
            pending, self._pending_preset = self._pending_preset, None
        if pending:
            if any(h['active'] for h in hands.values()):
                notice = '重置已取消：请先松开两侧侧键和前扳机'
            else:
                self._reset(pending)
        for hand, info in hands.items():
            if not info['fresh'] or not 0 < dt <= self.config.max_dt_s:
                self._pause(hand)
                self._release_required[hand] = True
                info['state'] = 'waiting' if info['received_s'] is None else 'stale'
                continue
            if self._release_required[hand]:
                if not info['active']:
                    self._release_required[hand] = False
                else:
                    info['state'] = 'release_required'
                    continue
            info['state'] = 'tracking' if info['active'] else 'holding'
            try:
                for contexts in self._pipelines.values():
                    ctx = contexts[hand]
                    sample = copy.deepcopy(raw[hand])
                    sample['xlevr.enabled'] = info['active']
                    sample['xlevr.sample_age_s'] = now - info['received_s']
                    action = ctx['mapper'](sample)
                    diagnostics = ctx['mapper'].get_last_diagnostics() or {}
                    if info['active'] and diagnostics.get('guard_reason') not in (
                            'active', 'arming', 'duplicate_sample'):
                        raise ValueError(diagnostics.get('guard_reason', 'input_rejected'))
                    controller = ctx['controller']
                    actual = controller.command_pose.copy()
                    if info['active']:
                        result = controller.step(action['_xlevr.anchor_command'], actual, dt,
                                                 sample_age_s=sample['xlevr.sample_age_s'])
                    elif ctx['state'] in ('tracking', 'frozen', 'releasing'):
                        result = controller.step_released(actual, dt)
                    else:
                        result = dict(state='holding', stop_requested=False)
                    if result.get('stop_requested') and result['state'] == 'fault':
                        raise ValueError(result.get('fault_reason') or 'reference_fault')
                    ctx['state'] = result['state']
            except (ValueError, RuntimeError) as error:
                self._pause(hand)
                self._release_required[hand] = True
                info['state'] = 'release_required'
                notice = f'{hand}: {error}；松开侧键和前扳机后重新接管'
        modes = {}
        for mode, contexts in self._pipelines.items():
            modes[mode] = {}
            for hand, ctx in contexts.items():
                target = ctx['controller'].command_pose.tolist()
                trail = ctx['trail']
                while trail and now - trail[0][0] > 10.:
                    trail.popleft()
                if hands[hand]['fresh'] and now - ctx['last_trail'] >= 1. / 30.:
                    trail.append([now, *target[:3]])
                    ctx['last_trail'] = now
                modes[mode][hand] = dict(pose=target, state=ctx['state'], trail=list(trail),
                    position_axis_remap=np.asarray(ctx['mapper']._position_axes).tolist(),
                    translation_frame=ctx['mapper'].anchor_translation_frame,
                    rotation_enabled=ctx['controller']._rotation_enabled,
                    logical_arm=self.bindings[hand])
        with self._lock:
            self._latest = dict(running=True, tick_time_s=now, control_hz=self._rate(self._ticks),
                                hands=hands, modes=modes, notice=notice)
        return self.snapshot()

    def run(self, *, max_steps=None):
        try:
            self.input.connect()
            desktop, headset_urls = sandbox_access_urls(self.input.config.server_settings)
            print(f'电脑本机测试台：{desktop}', flush=True)
            for url in headset_urls:
                print(f'头显输入页（选择与头显同网段的地址）：{url}', flush=True)
            if not headset_urls:
                print('未自动找到局域网地址；请用 ip -brief -4 address 查看本机网卡地址。', flush=True)
            print('本地证书提示时选择“高级 → 继续访问”。其他电脑用局域网地址加 /vr-sandbox.html。', flush=True)
            print('仅虚拟目标；不连接机械臂、夹爪或相机。Ctrl+C 退出。', flush=True)
            tick, count = time.monotonic(), 0
            while max_steps is None or count < max_steps:
                self.step()
                count += 1
                tick += 1. / self.fps
                now = time.monotonic()
                if tick < now:
                    tick = now
                time.sleep(max(0., tick - now))
        except KeyboardInterrupt:
            pass
        finally:
            with self._lock:
                self._latest['running'] = False
            self.input.disconnect()


def build_sandbox(teleop):
    """Whitelist VR input fields only, even if the supplied profile is live."""
    from ..vr_operator import VROperator
    settings = copy.deepcopy(dict(teleop))
    input_cfg = dict(settings['input'])
    input_cfg.pop('type', None)
    input_cfg['enable_left_events'] = False
    if input_cfg.get('arm') != 'both':
        raise ValueError('Choose a both-hands VR configuration')
    vr = VROperator(**input_cfg)
    mappings = {hand: {field: getattr(mapper, field) for field in mapper.__dataclass_fields__
                      if mapper.__dataclass_fields__[field].init}
                for hand, mapper in vr.mappers.items()}
    return VRSandbox(vr.input, mappings, vr.hand_to_arm,
                     settings.get('reference_options'), fps=90)
