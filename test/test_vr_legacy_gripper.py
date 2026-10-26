"""Single-arm VR -> legacy USB servo protocol; no hardware is opened."""
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

from mmengine import Config
from fluxvla.engines.operators.gripper_operator import GripperOperator, GripperError
from fluxvla.engines.operators.vr.runtime import build_runtime
from test_gripper_operator import FakeSerial
from test_vr_reference_runtime import Clock, runtime, sample

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'configs/a10/a10_vr_teleop_legacy_gripper.py'


def row(sequence, velocity, *, active=False, age=0.):
    result = sample(sequence, active=active, age=age)
    result['mapped']['gripper.pos'] = velocity
    result['mapped']['_xlevr.anchor_command']['gripper'] = velocity
    return result


def targets(serial):
    return [packet for packet in serial.writes if packet[4] == 0x83]


class LegacyVRGripperTest(unittest.TestCase):
    def setUp(self):
        self.serial = FakeSerial()
        patcher = patch('serial.Serial', return_value=self.serial)
        self.port_factory = patcher.start()
        self.addCleanup(patcher.stop)
        self.gripper = GripperOperator(port='/fake/legacy', timeout=.1)
        self.addCleanup(self.gripper.disconnect)

    def test_config_preview_and_vr_only_do_not_open_devices(self):
        cfg = Config.fromfile(str(CONFIG)).teleop
        rt = build_runtime(cfg)
        self.assertIsNone(rt.gripper)
        self.assertIsNone(rt.robot)
        rt.close()
        cfg.vr_only = False
        cfg.operator.allow_motion = True
        rt = build_runtime(cfg)
        self.assertIsInstance(rt.gripper, GripperOperator)
        self.assertEqual(rt.gripper.servo_id, 10)
        self.assertEqual(rt.gripper.baudrate, 1000000)
        self.assertEqual(rt.gripper.max_width_mm, 85.)
        self.assertEqual(rt.gripper_speed_mm_s, 10.)
        self.assertIsNone(rt.robot.gripper_operator)
        rt.close()
        self.port_factory.assert_not_called()

    def test_reference_usb_commands_without_squeeze_neutral_holds(self):
        rows = [row(1,1),row(2,1),row(3,0),row(4,-1)]
        rt, robot = runtime(rows, gripper=self.gripper, gripper_speed_mm_s=10.)
        clock = Clock()
        with patch('time.monotonic', clock.now):
            rt.connect()
            self.assertFalse(targets(self.serial))
            for raw in rows:
                clock.advance(); rt.step()
            packets = targets(self.serial)
            self.assertEqual(len(packets), 2)
            first = int.from_bytes(packets[0][8:10], 'little')
            last = int.from_bytes(packets[1][8:10], 'little')
            self.assertGreater(self.gripper.raw_to_position(first),40.)
            self.assertAlmostEqual(self.gripper.raw_to_position(last),40.,delta=.1)
            # SYNC_WRITE: ID10, PWM0x1000, speed800; existing protocol unchanged.
            self.assertEqual(packets[0][4:8], bytes([0x83,0x2A,6,10]))
            self.assertEqual(struct.unpack('<HH', packets[0][10:14]), (0x1000,800))
            self.assertEqual(robot.targets, [])
            self.assertNotIn(('start','cartesian'),robot.events)
            before = list(self.serial.writes)
            rt.close()
            self.assertEqual(self.serial.writes,before)  # No open/disable on exit.
            self.assertFalse(self.serial.is_open)

    def test_invert_direction_and_limits(self):
        for direction, width, velocity, expected in [(1,84.9,1,85), (1,.1,-1,0), (-1,40,1,39.)]:
            with self.subTest(direction=direction,width=width):
                self.serial.is_open=True
                self.serial.writes.clear()
                self.serial.position=self.gripper.position_to_raw(width)
                rt, _ = runtime([row(1,velocity),row(2,velocity)], gripper=self.gripper,
                                gripper_speed_mm_s=10.,gripper_direction=direction)
                clock=Clock()
                with patch('time.monotonic',clock.now):
                    rt.connect(); clock.advance(); rt.step()
                    clock.value += .1; rt.step()
                    value=self.gripper.get_position_mm()
                    self.assertAlmostEqual(value,expected,delta=.1)
                    rt.close()
        for direction in (0,2,True,float('nan')):
            with self.assertRaises(ValueError): runtime([],gripper_direction=direction)

    def test_stale_input_no_target_and_recovery_restarts_clock(self):
        rows=[row(1,1),row(2,1,age=1.),row(3,1),row(4,1)]
        rt,_=runtime(rows,gripper=self.gripper)
        clock=Clock()
        with patch('time.monotonic',clock.now):
            rt.connect()
            for _ in range(3): clock.advance(); rt.step()
            self.assertFalse(targets(self.serial))
            clock.advance(); rt.step()
            self.assertEqual(len(targets(self.serial)),1)
            rt.close()

    def test_input_expiring_during_serial_read_does_not_write(self):
        rt,_=runtime([row(1,1),row(2,1)],gripper=self.gripper)
        clock=Clock()
        with patch('time.monotonic',clock.now):
            rt.connect();clock.advance();rt.step()
            read=self.gripper.get_position_mm
            def slow_read():
                value=read();clock.value+=.3;return value
            with patch.object(self.gripper,'get_position_mm',side_effect=slow_read):
                clock.advance();rt.step()
            self.assertFalse(targets(self.serial))
            self.assertEqual(rt.last_gripper_command['reason'],'input_expired_during_read')
            self.assertIsNone(rt._last_gripper_time)
            rt.close()

    def test_readonly_and_fault_cleanup(self):
        rt,_=runtime([row(1,1),row(2,1)],observe=True)
        clock=Clock()
        with patch('time.monotonic',clock.now):
            rt.connect()
            for _ in range(2): clock.advance();rt.step()
            rt.close()
        self.port_factory.assert_not_called()
        rt,robot=runtime([row(1,0,active=True),row(2,0,active=True),
                          row(3,1,active=True)],gripper=self.gripper)
        with patch('time.monotonic',clock.now):
            rt.connect()
            for _ in range(2): clock.advance();rt.step()
            with patch.object(self.gripper,'get_position_mm',side_effect=GripperError('USB lost')):
                with self.assertRaisesRegex(GripperError,'USB lost'):
                    clock.advance();rt.step()
            self.assertIn('stop',robot.events)
            self.assertIn('wait_stopped',robot.events)
            rt.close()
        self.assertFalse(self.serial.is_open)
        self.assertFalse(targets(self.serial))

    def test_gripper_io_duration_is_recorded(self):
        rt,_=runtime([row(1,1),row(2,1)],gripper=self.gripper)
        clock=Clock()
        with tempfile.TemporaryDirectory() as folder, patch('time.monotonic',clock.now):
            rt.diagnostics_path=str(Path(folder)/'vr.jsonl')
            rt.connect()
            for _ in range(2):clock.advance();rt.step()
            rt.close()
            records=[json.loads(line) for line in Path(rt.diagnostics_path).read_text().splitlines()]
            command=records[-1]['gripper_command']
            self.assertTrue(command['transmitted'])
            self.assertFalse(command['arrival_confirmed'])
            self.assertIn('io_elapsed_s',command)


if __name__=='__main__':
    unittest.main()
