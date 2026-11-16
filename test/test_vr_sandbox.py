"""Synthetic VR and loopback HTTP only: sandbox cannot construct hardware."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from urllib.request import Request, urlopen
from urllib.error import HTTPError

import numpy as np
from mmengine import Config
from scipy.spatial.transform import Rotation

from fluxvla.engines.operators.vr.sandbox import VRSandbox, build_sandbox, sandbox_access_urls, MODES
from fluxvla.engines.operators.vr.monitor import ReuseHTTPServer, SimpleAPIHandler

ROOT = Path(__file__).resolve().parents[1]


class FakeInput:
    def __init__(self):
        self.calls = 0
        self.rows = {}
        self.connected = False
        self.config = SimpleNamespace(server_settings=dict(host_ip='127.0.0.1', https_port=8443))

    def get_hand_actions(self):
        self.calls += 1
        return copy.deepcopy(self.rows)

    def connect(self):
        self.connected = True

    def disconnect(self):
        self.connected = False


def hand(seq, stamp, *, grip=False, x=0., age=0., quaternion=None, tracked=True):
    return {'xlevr.target_position': [x, .3, -.1],
            'xlevr.orientation_quat': quaternion if quaternion is not None else [0., 0., 0., 1.],
            'xlevr.grip_active': grip, 'xlevr.buttons': {'squeeze': grip},
            'xlevr.sample_sequence': seq, 'xlevr.sample_receive_time_s': stamp - age,
            'xlevr.verification_tracking': tracked, 'xlevr.trigger': 0.,
            'xlevr.thumbstick': {'x': 0., 'y': 0.}, 'xlevr.sample_age_s': age}


class SandboxTest(unittest.TestCase):
    def setUp(self):
        self.input = FakeInput()
        self.now = 100.
        self.seq = 0
        self.clock = patch('time.monotonic', lambda: self.now)
        self.clock.start()
        self.addCleanup(self.clock.stop)
        self.view = VRSandbox(self.input, {'left': {}, 'right': {}},
                              {'left': 'right', 'right': 'left'},
                              dict(rotation_follow_mode='incremental_velocity',
                                   enforce_tracking_error_limits=False))

    def tick(self, left=False, right=False, x=0., age=0., tracked=True, quaternion=None):
        self.now += 1. / 90.
        self.seq += 1
        self.input.rows = {'left': hand(self.seq, self.now, grip=left, x=x, age=age,
                                        tracked=tracked, quaternion=quaternion),
                           'right': hand(self.seq, self.now, grip=right)}
        return self.view.step()

    def test_one_read_two_independent_mappings_rotation_matches_and_no_send(self):
        self.tick()
        self.tick(True, True, quaternion=Rotation.from_euler('z', .5).as_quat())
        for i in range(1, 45):
            q = Rotation.from_euler('xyz', [.002*i, .001*i, .5]).as_quat()
            value = self.tick(True, True, i*.0004, quaternion=q)
        self.assertEqual(self.input.calls, 46)
        a, b = (value['modes'][mode]['left']['pose'] for mode in MODES)
        self.assertGreater(np.linalg.norm(np.array(a[:3])-b[:3]), .0001)
        np.testing.assert_allclose(a[3:], b[3:], atol=1e-12)
        self.assertEqual(value['hands']['left']['logical_arm'], 'right')
        self.assertFalse(value['transmitted'])
        self.assertEqual(value['feedback'], 'previous_virtual_command')
        self.assertAlmostEqual(value['control_hz'], 90.)

    def test_symmetric_bases_and_all_tool_axes_face_same_table_directions(self):
        snapshot = self.tick()
        self.assertEqual(snapshot['preset'], 'horizontal')
        self.assertEqual(snapshot['world_frame'], dict(handedness='right', x='forward', y='left', z='up'))
        self.assertEqual(snapshot['tool_frame'], dict(x='up', y='right', z='forward'))
        expected = np.array([[0., 0., 1.], [0., -1., 0.], [1., 0., 0.]])
        for mode in MODES:
            for side in ('left', 'right'):
                target = snapshot['modes'][mode][side]['pose']
                frame = snapshot['base_frames'][side]
                rotation = np.asarray(frame['rotation_table_from_base'])
                self.assertAlmostEqual(np.linalg.det(rotation), 1.)
                np.testing.assert_allclose(rotation @ Rotation.from_quat(target[3:]).as_matrix(), expected, atol=1e-12)
                np.testing.assert_allclose(target[:3], [0., 0., .4])
                forward_local = Rotation.from_quat(target[3:]).apply([0., 0., 1.])
                np.testing.assert_allclose(forward_local, [1. if side=='left' else -1., 0., 0.], atol=1e-12)

    def test_same_world_displacement_moves_both_arms_same_table_direction(self):
        self.tick()
        for i in range(45):
            self.now += 1. / 90.
            self.seq += 1
            self.input.rows = {side: hand(self.seq, self.now, grip=True, x=i*.0003)
                               for side in ('left', 'right')}
            snapshot = self.view.step()
        displacements = []
        for side in ('left', 'right'):
            frame = snapshot['base_frames'][side]
            target = snapshot['modes'][MODES[1]][side]['pose']
            displacements.append(np.asarray(frame['rotation_table_from_base']) @ (np.asarray(target[:3])-[0.,0.,.4]))
        np.testing.assert_allclose(displacements[0], displacements[1], atol=1e-12)
        self.assertGreater(np.linalg.norm(displacements[0]), .0001)
        left = np.asarray(snapshot['modes'][MODES[1]]['left']['position_axis_remap'])
        right = np.asarray(snapshot['modes'][MODES[1]]['right']['position_axis_remap'])
        np.testing.assert_allclose(right, np.diag([-1.,-1.,1.]) @ left)

    def test_world_forward_left_up_and_opposites_for_both_bases_and_presets(self):
        directions = [([0.,0.,-1.], [1.,0.,0.]),  # VR forward -> table forward.
                      ([-1.,0.,0.], [0.,1.,0.]),  # VR left -> table left.
                      ([0.,1.,0.], [0.,0.,1.]),   # VR up -> table up.
                      ([0.,0.,1.], [-1.,0.,0.]),
                      ([1.,0.,0.], [0.,-1.,0.]),
                      ([0.,-1.,0.], [0.,0.,-1.])]
        q = Rotation.from_euler('xyz', [.4,-.2,.3]).as_quat()
        for preset in ('horizontal', 'down'):
            for vr_axis, table_axis in directions:
                with self.subTest(preset=preset, vr_axis=vr_axis):
                    self.tick(False, False)
                    self.view.request_reset(preset)
                    initial = self.tick(False, False)
                    for i in range(35):
                        self.now += 1./90.
                        self.seq += 1
                        self.input.rows = {}
                        for side in ('left','right'):
                            raw = hand(self.seq,self.now,grip=True,quaternion=q)
                            raw['xlevr.target_position'] = (np.array([0.,.3,-.1])+np.array(vr_axis)*.004*i/34.).tolist()
                            self.input.rows[side] = raw
                        result = self.view.step()
                    for side in ('left','right'):
                        start = np.array(initial['modes'][MODES[1]][side]['pose'][:3])
                        end = np.array(result['modes'][MODES[1]][side]['pose'][:3])
                        rotation = np.asarray(result['base_frames'][side]['rotation_table_from_base'])
                        delta = rotation @ (end-start)
                        progress = np.dot(delta, table_axis)
                        self.assertGreater(progress,.0001)
                        np.testing.assert_allclose(delta-progress*np.array(table_axis),[0.,0.,0.],atol=1e-12)
                        np.testing.assert_allclose(result['modes'][MODES[0]][side]['pose'][3:],
                                                   result['modes'][MODES[1]][side]['pose'][3:],atol=1e-12)
        # The old mode retains its original local axis mapping.
        for side in ('left','right'):
            np.testing.assert_allclose(self.view._pipelines[MODES[0]][side]['mapper']._position_axes,
                                       [[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]])

    def test_sample_received_during_input_read_is_fresh(self):
        def receive():
            self.now += .001
            return {h: hand(1, self.now) for h in ('left', 'right')}
        with patch.object(self.input, 'get_hand_actions', receive):
            result = self.view.step()
        self.assertTrue(all(h['fresh'] for h in result['hands'].values()))

    def test_release_is_bounded_and_does_not_stop_other_hand(self):
        self.tick()
        for i in range(30):
            value = self.tick(True, True, i*.001)
        previous = {m: value['modes'][m]['left']['pose'] for m in MODES}
        value = self.tick(False, True, .03)
        for mode in MODES:
            self.assertIn(value['modes'][mode]['left']['state'], ('releasing', 'holding'))
            self.assertEqual(value['modes'][mode]['right']['state'], 'tracking')
            distance = np.linalg.norm(np.array(value['modes'][mode]['left']['pose'][:3])-previous[mode][:3])
            self.assertLessEqual(distance, self.view.config.max_linear_speed_m_s/90. + 1e-12)
        for _ in range(130):
            value = self.tick(False, True, .03)
        for mode in MODES:
            self.assertEqual(value['modes'][mode]['left']['state'], 'holding')
        held = {mode: value['modes'][mode]['left']['pose'] for mode in MODES}
        self.tick(True, True, .03)
        value = self.tick(True, True, .03)
        for mode in MODES:
            np.testing.assert_allclose(value['modes'][mode]['left']['pose'], held[mode], atol=1e-12)

    def test_stale_tracking_loss_and_recovery_require_release(self):
        self.tick()
        self.tick(True)
        before = self.tick(True, x=.001)
        stale = self.tick(True, x=.002, age=.3)
        self.assertFalse(stale['hands']['left']['fresh'])
        for mode in MODES:
            np.testing.assert_allclose(before['modes'][mode]['left']['pose'], stale['modes'][mode]['left']['pose'])
        self.assertEqual(self.tick(True)['hands']['left']['state'], 'release_required')
        self.tick(False)
        self.assertEqual(self.tick(True)['hands']['left']['state'], 'tracking')
        self.assertFalse(self.tick(True, tracked=False)['hands']['left']['fresh'])

    def test_duplicate_samples_dont_inflate_rate_or_freshness(self):
        self.tick()
        self.tick(True)
        sequence = self.view.snapshot()['hands']['left']['sequence']
        for _ in range(25):
            self.now += 1./90.
            result = self.view.step()
        self.assertEqual(result['hands']['left']['sequence'], sequence)
        self.assertFalse(result['hands']['left']['fresh'])
        self.assertEqual(len(self.view._input_ticks['left']), 2)

    def test_snapshot_staleness_detachment_and_nonfinite_inputs(self):
        value = self.tick()
        value['hands']['left']['pose'][0] = 999
        self.assertNotEqual(self.view.snapshot()['hands']['left']['pose'][0], 999)
        self.now += .3
        self.assertFalse(self.view.snapshot()['hands']['left']['fresh'])
        self.input.rows['left']['xlevr.target_position'] = [float('nan'), 0., 0.]
        self.input.rows['left']['xlevr.trigger'] = float('nan')
        value = self.view.step()
        self.assertIsNone(value['hands']['left']['pose'])
        json.dumps(value, allow_nan=False)

    def test_reset_is_queued_and_held_grips_block_or_cancel_it(self):
        self.tick(True)
        with self.assertRaises(RuntimeError):
            self.view.request_reset('horizontal')
        self.tick(False)
        generation = self.view.snapshot()['generation']
        self.view.request_reset('horizontal')
        self.assertEqual(self.view.snapshot()['generation'], generation)
        value = self.tick(True)
        self.assertEqual(value['generation'], generation)
        self.assertIn('取消', value['notice'])
        self.tick(False)
        self.view.request_reset('horizontal')
        value = self.tick(False)
        self.assertEqual(value['generation'], generation+1)
        for mode in MODES:
            for side, data in value['modes'][mode].items():
                z = Rotation.from_quat(data['pose'][3:]).apply([0., 0., 1.])
                table_from_base = np.asarray(value['base_frames'][side]['rotation_table_from_base'])
                np.testing.assert_allclose(table_from_base @ z, [1., 0., 0.], atol=1e-12)

    def test_trails_are_bounded_and_only_recent(self):
        self.tick()
        for i in range(1000):
            value = self.tick(False)
        for mode in MODES:
            for ctx in value['modes'][mode].values():
                self.assertLessEqual(len(ctx['trail']), 300)
                self.assertGreaterEqual(ctx['trail'][0][0], self.now-10.)


class SandboxIsolationTest(unittest.TestCase):
    def test_access_urls_exclude_proxy_and_virtual_interfaces(self):
        interfaces = [dict(ifname=name, operstate='UP', addr_info=[dict(family='inet', local=ip)])
                      for name, ip in [('Meta', '198.18.0.1'), ('enP2p1s0', '192.168.0.111'),
                                       ('wlP1p1s0', '192.168.110.119'), ('docker0', '172.17.0.1')]]
        with patch('subprocess.run', return_value=SimpleNamespace(stdout=json.dumps(interfaces))):
            desktop, headsets = sandbox_access_urls(dict(host_ip='0.0.0.0', https_port=8443))
        self.assertEqual(desktop, 'https://127.0.0.1:8443/vr-sandbox.html')
        self.assertEqual(headsets, ['https://192.168.0.111:8443/', 'https://192.168.110.119:8443/'])
        with patch('subprocess.run', side_effect=OSError('ip missing')):
            self.assertEqual(sandbox_access_urls(dict(host_ip='0.0.0.0', https_port=8443)), (desktop, []))
            self.assertEqual(sandbox_access_urls(dict(host_ip='192.168.0.111', https_port=8443)),
                             ('https://192.168.0.111:8443/vr-sandbox.html', ['https://192.168.0.111:8443/']))

    def test_live_profile_cannot_construct_hardware_or_read_calibration(self):
        cfg = Config.fromfile(str(ROOT / 'configs/a10/a10_dual_arm_vr_teleop_world_translation.py')).teleop
        cfg.pose_calibration_path = '/not-present/calibration.json'
        with patch('fluxvla.engines.operators.a10_operator.A10Operator.__init__', side_effect=AssertionError('Robot constructed')), \
             patch('fluxvla.engines.operators.vr.runtime.build_runtime', side_effect=AssertionError('Runtime constructed')), \
             patch.object(Path, 'read_text', side_effect=AssertionError('Calibration read')):
            view = build_sandbox(cfg)
        self.assertEqual(view.fps, 90)
        self.assertFalse(view.input.is_connected)
        self.assertTrue(view.input.sandbox_view is view)
        self.assertEqual(view.config.rotation_follow_mode, 'incremental_velocity')

    def test_cli_preview_never_connects(self):
        code = '''
from unittest.mock import patch
from scripts.preview_vr_mapping import main
with patch('fluxvla.engines.operators.vr.input.VRInput.connect', side_effect=AssertionError('Connected')):
    main(['--preview'])
'''
        result = subprocess.run([sys.executable, '-c', code], cwd=ROOT, capture_output=True, text=True, timeout=90)
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        self.assertIn('90 Hz', result.stdout)

    def test_http_observer_and_reset_api_are_isolated(self):
        view = VRSandbox(FakeInput(), {'left': {}, 'right': {}}, {'left':'left','right':'right'})
        server = ReuseHTTPServer(('127.0.0.1', 0), SimpleAPIHandler)
        server.web_root_path = str(ROOT / 'fluxvla/engines/operators/vr')
        server.sandbox_view = view
        server.config = SimpleNamespace(websocket_port=8442)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        base = f'http://127.0.0.1:{server.server_port}'
        for route in ('/vr-sandbox.html', '/vr-sandbox.js', '/api/vr-sandbox'):
            with urlopen(base+route, timeout=2) as response:
                self.assertEqual(response.status, 200)
                self.assertIn('no-store', response.headers['Cache-Control'])
        self.assertEqual(view.input.calls, 0)
        request = Request(base+'/api/vr-sandbox/reset', data=b'{"preset":"horizontal"}', headers={'Content-Type':'application/json'})
        with urlopen(request, timeout=2) as response:
            self.assertEqual(response.status, 202)
        with self.assertRaises(HTTPError) as error:
            urlopen(Request(base+'/api/vr-sandbox/reset', data=b'{"preset":"bad"}'), timeout=2)
        self.assertEqual(error.exception.code, 400)
        server.sandbox_view = None
        with self.assertRaises(HTTPError) as error:
            urlopen(base+'/api/vr-sandbox', timeout=2)
        self.assertEqual(error.exception.code, 404)
        self.assertFalse(view.input.connected)




class SandboxTriggerTest(unittest.TestCase):
    def setUp(self):
        SandboxTest.setUp(self)
        for contexts in self.view._pipelines.values():
            for ctx in contexts.values():
                ctx['mapper'].squeeze_translation_only = True

    def advance(self, *, trigger=0., grip=False, angle=0., x=0., age=0.):
        self.now += 1./90.
        self.seq += 1
        self.input.rows = {
            'left': hand(self.seq, self.now, grip=grip, x=x, age=age,
                         quaternion=Rotation.from_euler('z', angle).as_quat()),
            'right': hand(self.seq, self.now)}
        self.input.rows['left']['xlevr.trigger'] = trigger
        return self.view.step()

    def test_trigger_alone_moves_and_rotates_both_virtual_mappings(self):
        initial = self.advance()
        for i in range(1, 40):
            result = self.advance(trigger=1., x=.0004*i, angle=.015*i)
        info = result['hands']['left']
        self.assertFalse(info['grip'])
        self.assertTrue(info['active'])
        self.assertTrue(info['rotation_enabled'])
        self.assertEqual(info['state'], 'tracking')
        for mode in MODES:
            a, b = (np.asarray(v['modes'][mode]['left']['pose']) for v in (initial, result))
            self.assertGreater(np.linalg.norm(b[:3]-a[:3]), .0001)
            self.assertGreater((Rotation.from_quat(a[3:]).inv()*Rotation.from_quat(b[3:])).magnitude(), .001)
            self.assertEqual(result['modes'][mode]['left']['state'], 'tracking')
        self.assertFalse(result['transmitted'])

    def test_grip_to_trigger_switch_locks_then_unlocks_without_catchup(self):
        self.advance()
        for i in range(1, 25): locked = self.advance(grip=True, x=.0004*i, angle=.015*i)
        self.assertFalse(locked['hands']['left']['rotation_enabled'])
        switched = self.advance(trigger=1., x=.0096, angle=.7)
        for mode in MODES:
            np.testing.assert_allclose(locked['modes'][mode]['left']['pose'][3:],
                                       switched['modes'][mode]['left']['pose'][3:], atol=1e-12)
        for i in range(1, 20): rotated = self.advance(trigger=1., x=.0096, angle=.7+.015*i)
        for mode in MODES:
            a = Rotation.from_quat(switched['modes'][mode]['left']['pose'][3:])
            b = Rotation.from_quat(rotated['modes'][mode]['left']['pose'][3:])
            self.assertGreater((a.inv()*b).magnitude(), .001)

    def test_trigger_priority_and_release_braking(self):
        for i in range(1, 25): result = self.advance(grip=True, trigger=1., angle=.02*i, x=.0004*i)
        self.assertTrue(result['hands']['left']['active'])
        self.assertTrue(result['hands']['left']['rotation_enabled'])
        released = self.advance()
        self.assertFalse(released['hands']['left']['active'])
        for mode in MODES:
            self.assertIn(released['modes'][mode]['left']['state'], ('holding', 'releasing'))

    def test_trigger_must_be_released_after_tracking_loss(self):
        self.advance(trigger=1.)
        self.advance(trigger=1., age=1.)
        for _ in range(4): result = self.advance(trigger=1.)
        self.assertEqual(result['hands']['left']['state'], 'release_required')
        self.advance()
        for _ in range(3): result = self.advance(trigger=1.)
        self.assertEqual(result['hands']['left']['state'], 'tracking')

    def test_reset_is_rejected_or_cancelled_when_only_trigger_is_held(self):
        self.advance(trigger=1.)
        with self.assertRaisesRegex(RuntimeError, '前扳机'):
            self.view.request_reset('down')
        self.advance()
        generation = self.view.snapshot()['generation']
        self.view.request_reset('down')
        result = self.advance(trigger=1.)
        self.assertEqual(result['generation'], generation)
        self.assertIn('取消', result['notice'])

    def test_trigger_activation_uses_the_configured_threshold(self):
        for contexts in self.view._pipelines.values():
            contexts['left']['mapper'].fine_trigger_threshold = .8
        result = self.advance(trigger=.7)
        self.assertFalse(result['hands']['left']['active'])
        result = self.advance(trigger=.8)
        self.assertTrue(result['hands']['left']['active'])
        self.assertTrue(result['hands']['left']['rotation_enabled'])


if __name__ == '__main__':
    unittest.main()
