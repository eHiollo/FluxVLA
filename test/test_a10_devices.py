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


if __name__ == '__main__':
    unittest.main()
