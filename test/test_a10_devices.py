"""USB identity matching and safe configuration updates, no physical devices."""

import copy
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location(
    '_a10_devices_test', Path(__file__).resolve().parents[1] / 'scripts/a10_devices.py')
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)

SOURCE = '''# 中文注释应当完整保留
cameras = dict(
    base_0_rgb=dict(identity=dict(model='D435i', serial='camera', usb_interface='03', video_index=0),
                    device='/old/外部相机', width=640, fps=30),
)
grippers = dict(
    left=dict(identity=dict(model='FT231X', serial='gripper', usb_interface='00'),
              port='/old/夹爪', baudrate=115200, device_id=1, torque=60),
)
'''


def camera(**kwargs):
    return dict(kind='camera', serial='camera', usb_interface='03', video_index=0,
                node='/dev/video16', path='/dev/v4l/by-path/new-color',
                formats=['YUYV'], error='', **kwargs)


def gripper():
    return dict(kind='gripper', serial='gripper', usb_interface='00',
                node='/dev/ttyUSB7', path='/dev/serial/by-id/usb-gripper', error='')


class DeviceTests(unittest.TestCase):
    def setUp(self):
        self.config, self.nodes = m.load_config(SOURCE)

    def test_match_serial_and_color_interface_after_port_change(self):
        depth = camera(); depth.update(usb_interface='00', formats=['Z16 '], node='/dev/video12')
        rows = m.match_devices(self.config, [camera(), depth, gripper()])
        self.assertEqual([r['status'] for r in rows], ['path_changed', 'path_changed'])
        self.assertEqual(rows[0]['node'], '/dev/video16')
        self.assertEqual(rows[1]['node'], '/dev/ttyUSB7')

    def test_new_serial_is_not_assigned_to_missing_role(self):
        wrong = camera(); wrong['serial'] = 'unknown'
        rows = m.match_devices(self.config, [wrong, gripper()])
        self.assertEqual(rows[0]['status'], 'missing')
        self.assertEqual(rows[0]['path'], '')

    def test_duplicate_serial_cannot_be_updated(self):
        other = camera(); other['node'] = '/dev/video20'
        rows = m.match_devices(self.config, [camera(), other])
        self.assertEqual(rows[0]['status'], 'ambiguous')

    def test_same_device_cannot_be_bound_to_two_roles(self):
        self.config['cameras']['left_wrist_0_rgb'] = copy.deepcopy(self.config['cameras']['base_0_rgb'])
        rows = m.match_devices(self.config, [camera()])
        self.assertEqual([r['status'] for r in rows[:2]], ['ambiguous', 'ambiguous'])

    def test_unreadable_or_depth_only_endpoint_is_not_accepted(self):
        for changes in ({'error': 'Permission denied'}, {'formats': ['Z16 ']}):
            item = camera(); item.update(changes)
            self.assertEqual(m.match_devices(self.config, [item])[0]['status'], 'unusable')

    def test_old_path_pointing_to_wrong_camera_is_not_trusted(self):
        self.config['cameras']['base_0_rgb']['device'] = '/dev/v4l/by-path/old'
        resolve = lambda p: Path('/dev/video99' if str(p).endswith('/old') else str(p))
        with patch.object(Path, 'exists', return_value=True), patch.object(Path, 'resolve', resolve):
            row = m.match_devices(self.config, [camera()])[0]
        self.assertEqual(row['status'], 'path_changed')

    def test_current_color_path_preserved_and_colliding_by_id_rejected(self):
        settings = self.config['cameras']['base_0_rgb']
        resolve = lambda p: Path('/dev/video16')
        for path, expected in [('/dev/v4l/by-path/color', 'matched'),
                               ('/dev/v4l/by-id/colliding', 'path_changed')]:
            settings['device'] = path
            item = camera(); item['by_id_unique'] = False
            with patch.object(Path, 'exists', return_value=True), patch.object(Path, 'resolve', resolve):
                self.assertEqual(m.match_devices(self.config, [item])[0]['status'], expected)

    def test_update_preserves_comments_units_and_backup_exact_bytes(self):
        rows = m.match_devices(self.config, [camera(), gripper()])
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'devices.py'; original = SOURCE.encode()
            path.write_bytes(original)
            backup = m.update_paths(path, original, self.nodes, rows)
            self.assertEqual(backup.read_bytes(), original)
            updated = path.read_text()
            self.assertIn('# 中文注释应当完整保留', updated)
            self.assertIn('torque=60', updated)
            self.assertIn('width=640, fps=30', updated)
            cfg, _ = m.load_config(updated)
            self.assertEqual(cfg['cameras']['base_0_rgb']['device'], rows[0]['path'])
            self.assertEqual(cfg['grippers']['left']['port'], rows[1]['path'])
            self.assertEqual(cfg['grippers']['left']['identity'], self.config['grippers']['left']['identity'])

    def test_missing_and_ambiguous_do_not_write_or_create_backup(self):
        rows = m.match_devices(self.config, [camera(), camera()])
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'devices.py'; path.write_text(SOURCE)
            self.assertIsNone(m.update_paths(path, SOURCE.encode(), self.nodes, rows))
            self.assertEqual(path.read_text(), SOURCE)
            self.assertEqual(list(Path(folder).iterdir()), [path])

    def test_concurrent_config_edit_is_not_overwritten(self):
        rows = m.match_devices(self.config, [camera()])
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'devices.py'; path.write_text(SOURCE + '# 新修改\n')
            with self.assertRaisesRegex(ValueError, '其他程序修改'):
                m.update_paths(path, SOURCE.encode(), self.nodes, rows)
            self.assertTrue(path.read_text().endswith('# 新修改\n'))
            self.assertEqual(len(list(Path(folder).iterdir())), 1)

    def test_config_does_not_execute_expressions(self):
        with self.assertRaises(ValueError):
            m.load_config("cameras = dict(x=__import__('os').system('false'))\ngrippers=dict()")

    def test_default_cli_does_not_open_gripper_or_update(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'devices.py'; path.write_text(SOURCE)
            with patch.object(m, 'scan_devices', return_value=[camera(), gripper()]), \
                    patch.object(m, 'check_grippers') as check, \
                    patch.object(m, 'update_paths') as update, \
                    patch('builtins.print'):
                self.assertEqual(m.main(['--config', str(path)]), 1)
                check.assert_not_called(); update.assert_not_called()
            self.assertEqual(path.read_text(), SOURCE)

    def test_read_only_driver_only_uses_modbus_read_requests(self):
        # Exercise the real driver through --check-grippers, with a fake serial transport.
        sys.path.insert(0, str(Path(__file__).parent))
        try:
            from test_zx_gripper import FakeSerial
        finally:
            sys.path.pop(0)
        transport = FakeSerial()
        rows = m.match_devices(self.config, [gripper()])
        with patch('serial.Serial', return_value=transport):
            m.check_grippers(rows, self.config)
        self.assertIn('gripper_state', rows[1])
        self.assertTrue(transport.writes)
        self.assertTrue(all(packet[1] == 3 for packet in transport.writes))


class DeviceVisualTests(unittest.TestCase):
    def setUp(self):
        self.config, _ = m.load_config(SOURCE)
        left = self.config['grippers']['left']
        left.update(speed=5, check=dict(positions_raw=(0, 1000), open_raw=None, close_raw=None))
        self.config['grippers']['right'] = copy.deepcopy(left)
        self.config['grippers']['right']['port'] = '/dev/serial/by-id/right'
        self.config['grippers']['right']['identity']['serial'] = 'right'
        self.rows = m.match_devices(self.config, [camera(), gripper()])
        for row in self.rows:
            if row['name'] == 'left':
                row['status'] = 'matched'
        self.rows[-1].update(configured_path='/dev/serial/by-id/right', node='/dev/ttyUSB8', status='matched')
        sys.path.insert(0, str(Path(__file__).parent))
        try:
            from test_zx_gripper import FakeSerial
        finally:
            sys.path.pop(0)
        self.transport = FakeSerial()
        self.transport.alarms = (0, 0)
        self.transport.position = 0

    def test_left_and_right_motion_open_only_selected_port(self):
        for side, target in [('left', 1000), ('right', 0)]:
            self.transport.closed = False
            with patch('serial.Serial', return_value=self.transport) as opened:
                result = m.move_gripper(self.config, self.rows, side, 'position', target)
            self.assertEqual(opened.call_count, 1)
            self.assertEqual(opened.call_args.args[0], self.config['grippers'][side]['port'])
            self.assertEqual(result['side'], side)
            self.assertEqual(result['final']['position_raw'], target)
            self.assertTrue(self.transport.closed)
            self.assertTrue(self.transport.enabled)  # no automatic release

    def test_alarm_blocks_all_motion_writes(self):
        self.transport.alarms = (0, 0x1000)
        with patch('serial.Serial', return_value=self.transport):
            with self.assertRaisesRegex(RuntimeError, '有报警'):
                m.move_gripper(self.config, self.rows, 'left', 'position', 1000)
        self.assertTrue(all(packet[1] == 3 for packet in self.transport.writes))

    def test_position_outside_check_range_blocks_motion(self):
        self.transport.position = 3000
        with patch('serial.Serial', return_value=self.transport):
            with self.assertRaisesRegex(ValueError, '实际位置'):
                m.move_gripper(self.config, self.rows, 'left', 'position', 1000)
        self.assertTrue(all(packet[1] == 3 for packet in self.transport.writes))

    def test_unconfirmed_direction_and_invalid_target_do_not_open_port(self):
        with patch.object(m, 'load_operator') as loader:
            for action, target in [('open', None), ('close', None), ('position', 500)]:
                with self.assertRaises(ValueError):
                    m.move_gripper(self.config, self.rows, 'left', action, target)
            loader.assert_not_called()

    def test_only_confirmed_direction_uses_open_close_labels(self):
        self.config['grippers']['left']['check'].update(open_raw=1000, close_raw=0)
        with patch('serial.Serial', return_value=self.transport):
            result = m.move_gripper(self.config, self.rows, 'left', 'open')
        self.assertEqual(result['target_raw'], 1000)
        self.assertEqual(result['final']['completion'], 'position_reached')

    def test_wrong_or_stale_identity_does_not_open_port(self):
        for status in ('missing', 'path_changed', 'ambiguous', 'unusable'):
            self.rows[1]['status'] = status
            with patch.object(m, 'load_operator') as loader:
                with self.assertRaises(ValueError):
                    m.move_gripper(self.config, self.rows, 'left', 'position', 1000)
                loader.assert_not_called()

    def test_excess_speed_rejected_before_any_serial_access(self):
        self.config['grippers']['left']['speed'] = 6
        with patch.object(m, 'load_operator') as loader:
            with self.assertRaisesRegex(ValueError, 'speed'):
                m.move_gripper(self.config, self.rows, 'left', 'position', 1000)
            loader.assert_not_called()

    def test_motion_failure_is_not_retried_or_followed_by_release(self):
        from types import SimpleNamespace
        from unittest.mock import MagicMock
        client = MagicMock()
        client.__enter__.return_value = client
        client.read_state.return_value = SimpleNamespace(has_error=False, position_raw=0)
        client.set_position_raw.side_effect = RuntimeError('timeout')
        module = SimpleNamespace(ZXGripper=MagicMock(return_value=client))
        with patch.object(m, 'load_operator', return_value=module):
            with self.assertRaisesRegex(RuntimeError, 'timeout'):
                m.move_gripper(self.config, self.rows, 'left', 'position', 1000)
        self.assertEqual(client.set_position_raw.call_count, 1)
        client.disable.assert_not_called()
        client.reset_alarms.assert_not_called()
        self.assertEqual(client.__exit__.call_count, 1)

    def test_snapshot_outputs_three_rgb_images_and_labeled_overview(self):
        import json
        import numpy as np
        from PIL import Image
        from types import SimpleNamespace
        from unittest.mock import MagicMock
        names = ('base_0_rgb', 'left_wrist_0_rgb', 'right_wrist_0_rgb')
        colors = ((250, 10, 10), (10, 250, 10), (10, 10, 250))
        frame = {name: np.full((48, 64, 3), color, dtype=np.uint8) for name, color in zip(names, colors)}
        frame['camera_received_monotonic_s'] = 12.5
        rows = [dict(group='cameras', name=name, label=name, serial=str(i), status='matched',
                     configured_path='/dev/fake' + str(i), node='/dev/video' + str(i)) for i, name in enumerate(names)]
        cfg = dict(cameras={name: dict(device=row['configured_path']) for name, row in zip(names, rows)})
        operator = MagicMock()
        operator.__enter__.return_value = operator
        operator.get_frame.return_value = frame
        module = SimpleNamespace(CameraOperator=MagicMock(return_value=operator))
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / 'photos'
            with patch.object(m, 'load_operator', return_value=module):
                result = m.take_snapshots(cfg, rows, output)
            self.assertEqual(len(list(output.glob('*.jpg'))), 4)
            with Image.open(output / 'overview.jpg') as overview:
                self.assertEqual(overview.size, (1920, 566))
            manifest = json.loads((output / 'manifest.json').read_text())
            for name, color in zip(names, colors):
                with Image.open(output / (name + '.jpg')) as image:
                    self.assertTrue(all(abs(a - b) <= 3 for a, b in zip(image.getpixel((0, 0)), color)))
                self.assertEqual(manifest['cameras'][name]['serial'], result['cameras'][name]['serial'])
            self.assertEqual(operator.__exit__.call_count, 1)
            with patch.object(m, 'load_operator', return_value=module):
                with self.assertRaises(FileExistsError):
                    m.take_snapshots(cfg, rows, output)

    def test_snapshot_missing_camera_refuses_before_capture(self):
        with patch.object(m, 'load_operator') as loader:
            with self.assertRaises(ValueError):
                m.take_snapshots(self.config, self.rows)
            loader.assert_not_called()

    def test_conflicting_cli_actions_rejected_without_scan(self):
        args = [ ['--snapshot', '--gripper', 'left', '--position-raw', '0'],
                 ['--snapshot', '--update'], ['--gripper', 'left'], ['--open'],
                 ['--gripper', 'left', '--position-raw', '0', '--check-grippers'],
                 ['--gripper', 'left', '--open', '--close'] ]
        with patch.object(m, 'scan_devices') as scan, patch('sys.stderr'):
            for argv in args:
                with self.assertRaises(SystemExit) as result:
                    m.main(argv)
                self.assertEqual(result.exception.code, 2)
            scan.assert_not_called()


if __name__ == '__main__':
    unittest.main()
