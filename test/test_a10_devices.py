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
        left.update(speed=5, check=dict(min_raw=0, max_raw=3000, tolerance_raw=5, open_raw=None, close_raw=None))
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
        self.transport.position = 4000
        with patch('serial.Serial', return_value=self.transport):
            with self.assertRaisesRegex(ValueError, '实际位置'):
                m.move_gripper(self.config, self.rows, 'left', 'position', 1000)
        self.assertTrue(all(packet[1] == 3 for packet in self.transport.writes))

    def test_unconfirmed_direction_and_invalid_target_do_not_open_port(self):
        with patch.object(m, 'load_operator') as loader:
            for action, target in [('open', None), ('close', None), ('position', 3001)]:
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
                 ['--gripper', 'left', '--open', '--close'],
                 ['--gripper', 'left', '--calibrate', '--json'],
                 ['--gripper', 'left', '--calibrate', '--update'] ]
        with patch.object(m, 'scan_devices') as scan, patch('sys.stderr'):
            for argv in args:
                with self.assertRaises(SystemExit) as result:
                    m.main(argv)
                self.assertEqual(result.exception.code, 2)
            scan.assert_not_called()


class GripperRangeTests(unittest.TestCase):
    def setUp(self):
        DeviceVisualTests.setUp(self)

    def test_arbitrary_values_and_both_endpoints_follow_config(self):
        for target in (0, 1, 1739, 2999, 3000):
            with patch('serial.Serial', return_value=self.transport):
                result = m.move_gripper(self.config, self.rows, 'left', 'position', target)
            self.assertTrue(result['arrived'])
            self.assertEqual(result['final']['position_raw'], target)
            self.assertEqual(result['error_raw'], 0)

    def test_out_of_range_and_invalid_config_never_open_serial(self):
        with patch.object(m, 'load_operator') as loader:
            for target in (-1, 3001, True, 100.5):
                with self.assertRaises(ValueError):
                    m.move_gripper(self.config, self.rows, 'left', 'position', target)
            for changes in ({'min_raw': -1}, {'max_raw': 0}, {'max_raw': True},
                            {'max_raw': 2**32}, {'tolerance_raw': -1},
                            {'move_timeout_s': float('nan')}, {'move_timeout_s': 61}):
                cfg = copy.deepcopy(self.config)
                cfg['grippers']['left']['check'].update(changes)
                with self.assertRaises(ValueError):
                    m.move_gripper(cfg, self.rows, 'left', 'position', 0)
            loader.assert_not_called()

    def test_sides_have_independent_ranges(self):
        self.config['grippers']['right']['check']['max_raw'] = 700
        with patch.object(m, 'load_operator') as loader:
            with self.assertRaises(ValueError):
                m.move_gripper(self.config, self.rows, 'right', 'position', 1000)
            loader.assert_not_called()
        with patch('serial.Serial', return_value=self.transport):
            self.assertTrue(m.move_gripper(self.config, self.rows, 'left', 'position', 1000)['arrived'])

    def test_old_position_bit_without_actual_arrival_times_out(self):
        self.transport.follow = False
        self.config['grippers']['left']['check']['move_timeout_s'] = .01
        with patch('serial.Serial', return_value=self.transport):
            result = m.move_gripper(self.config, self.rows, 'left', 'position', 2000)
        self.assertFalse(result['arrived'])
        self.assertEqual(result['outcome'], 'timeout')
        self.assertEqual(result['final']['position_raw'], 0)
        self.assertTrue(self.transport.enabled)

    def test_torque_completion_does_not_count_as_arrival(self):
        self.transport.follow = False
        self.transport.status = 0x001C  # torque + zero speed, no position arrival
        with patch('serial.Serial', return_value=self.transport):
            result = m.move_gripper(self.config, self.rows, 'left', 'position', 2000)
        self.assertEqual(result['outcome'], 'torque_reached')
        self.assertFalse(result['arrived'])
        self.assertEqual(self.transport.target, 2000)  # target retained, no retreat

    def test_motion_cli_returns_nonzero_for_unconfirmed_position(self):
        from unittest.mock import MagicMock
        failure = dict(arrived=False, outcome='torque_reached')
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'devices.py'
            path.write_text('cameras = ' + repr(self.config['cameras']) + '\ngrippers = ' + repr(self.config['grippers']))
            with patch.object(m, 'scan_devices', return_value=[]), \
                    patch.object(m, 'match_devices', return_value=self.rows), \
                    patch.object(m, 'move_gripper', return_value=failure) as move, \
                    patch.object(m, 'print_report'):
                self.assertEqual(m.main(['--config', str(path), '--gripper', 'left', '--position-raw', '1739']), 1)
                self.assertEqual(move.call_args.args[-1], 1739)

    def run_wizard(self, commands, *, expected='saved', after_write=None):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'devices.py'
            source = '# 中文注释保留\ncameras = ' + repr(self.config['cameras']) + '\ngrippers = ' + repr(self.config['grippers']) + '\n'
            path.write_text(source)
            config, nodes = m.load_config(source)
            if after_write:
                after_write(path)
            with patch('serial.Serial', return_value=self.transport) as serial, \
                    patch('builtins.input', side_effect=commands), patch('builtins.print'):
                result = m.calibrate_gripper(config, self.rows, 'left', path, source.encode(), nodes)
            self.assertEqual(result['status'], expected)
            self.assertEqual(serial.call_count, 1)
            self.assertEqual(serial.call_args.args[0], self.config['grippers']['left']['port'])
            self.assertTrue(self.transport.closed)
            updated, _ = m.load_config(path.read_text())
            if expected == 'saved':
                self.assertEqual(Path(result['backup']).read_text(), source)
                self.assertTrue(path.read_text().startswith('# 中文注释保留\n'))
                self.assertEqual(updated['grippers']['right'], config['grippers']['right'])
                self.assertEqual(updated['cameras'], config['cameras'])
                self.assertEqual(updated['grippers']['left']['identity'], config['grippers']['left']['identity'])
            else:
                self.assertEqual(path.read_text(), source)
                self.assertEqual(list(Path(folder).glob('*.bak')), [])
            return result, updated

    def test_calibration_records_direction_and_saves_only_selected_check(self):
        result, cfg = self.run_wizard(['1739', 'open', '0', 'close', 'save', 'yes'])
        check = cfg['grippers']['left']['check']
        self.assertEqual((check['min_raw'], check['max_raw']), (0, 1739))
        self.assertEqual((check['open_raw'], check['close_raw']), (1739, 0))
        self.assertEqual(check['calibration']['serial'], 'gripper')
        self.assertEqual(check['calibration']['open_actual_raw'], 1739)
        self.assertEqual(cfg['grippers']['left']['speed'], 5)
        self.assertEqual(cfg['grippers']['left']['torque'], 60)

    def test_calibration_supports_reverse_direction_and_explicit_range_expansion(self):
        _, cfg = self.run_wizard(['0', 'open', 'range 0 4000', '3500', 'close', 'save', '确认'])
        check = cfg['grippers']['left']['check']
        self.assertEqual((check['min_raw'], check['max_raw']), (0, 3500))
        self.assertEqual((check['open_raw'], check['close_raw']), (0, 3500))

    def test_cancel_and_eof_never_save_or_automatically_move(self):
        for commands in (['range 0 4000', 'exit'], [EOFError()], [KeyboardInterrupt()]):
            self.transport.writes.clear()
            self.run_wizard(commands, expected='cancelled')
            self.assertTrue(all(packet[1] == 3 for packet in self.transport.writes))

    def test_declined_save_missing_points_and_equal_points_do_not_write(self):
        self.run_wizard(['save', 'open', '1000', 'open', 'close', 'save',
                         '0', 'close', 'save', 'no', 'exit'], expected='cancelled')

    def test_unreached_point_ends_calibration_without_saving(self):
        self.transport.follow = False
        self.transport.status = 0x001C
        self.run_wizard(['1000'], expected='failed')

    def test_range_command_alone_never_triggers_motion(self):
        self.run_wizard(['range 2 1', 'range 1000 3000', 'range 0 4000', '99999', 'exit'], expected='cancelled')
        self.assertTrue(all(packet[1] == 3 for packet in self.transport.writes))

    def test_calibration_refuses_to_overwrite_concurrent_edits(self):
        with self.assertRaisesRegex(ValueError, '其他程序修改'):
            self.run_wizard(['1000', 'open', '0', 'close', 'save', 'yes'],
                            after_write=lambda path: path.write_text(path.read_text() + '# other edit\n'))


    def test_configured_tolerance_is_used_instead_of_driver_default(self):
        original_write = self.transport.write
        def offset_feedback(packet):
            result = original_write(packet)
            if packet[1:6] == bytes.fromhex('06 01 08 00 01'):
                self.transport.position = self.transport.target + 3
            return result
        self.transport.write = offset_feedback
        self.config['grippers']['left']['check'].update(tolerance_raw=2, move_timeout_s=.01)
        with patch('serial.Serial', return_value=self.transport):
            result = m.move_gripper(self.config, self.rows, 'left', 'position', 1000)
        self.assertFalse(result['arrived'])
        self.assertEqual(result['error_raw'], 3)
        self.config['grippers']['left']['check'].update(tolerance_raw=5, move_timeout_s=1)
        with patch('serial.Serial', return_value=self.transport):
            self.assertTrue(m.move_gripper(self.config, self.rows, 'left', 'position', 1000)['arrived'])

    def test_nonzero_minimum_is_enforced(self):
        self.config['grippers']['left']['check']['min_raw'] = 100
        self.transport.position = 100
        with patch.object(m, 'load_operator') as loader:
            with self.assertRaises(ValueError):
                m.move_gripper(self.config, self.rows, 'left', 'position', 99)
            loader.assert_not_called()
        with patch('serial.Serial', return_value=self.transport):
            self.assertTrue(m.move_gripper(self.config, self.rows, 'left', 'position', 101)['arrived'])

    def test_calibrated_labels_are_bound_to_usb_serial_and_modbus_id(self):
        check = self.config['grippers']['left']['check']
        check.update(open_raw=1000, close_raw=0,
                     calibration=dict(serial='different-device', device_id=1))
        with patch.object(m, 'load_operator') as loader:
            with self.assertRaisesRegex(ValueError, '校准记录'):
                m.move_gripper(self.config, self.rows, 'left', 'open')
            check['calibration'] = dict(serial='gripper', device_id=2)
            with self.assertRaisesRegex(ValueError, '校准记录'):
                m.move_gripper(self.config, self.rows, 'left', 'close')
            loader.assert_not_called()

    def test_indistinguishable_endpoints_are_not_saved(self):
        self.run_wizard(['0', 'open', '5', 'close', 'save', 'exit'], expected='cancelled')


if __name__ == '__main__':
    unittest.main()
