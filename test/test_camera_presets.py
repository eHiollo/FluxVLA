"""RealSense preset loading with fake devices; no hardware required."""
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import test_camera_operator as cameras

m = cameras.module


class CameraPresetTest(unittest.TestCase):
    def test_sdk_or_asic_serial_selects_exact_device_and_model_is_checked(self):
        info = SimpleNamespace(serial_number='serial', asic_serial_number='asic', name='name')
        dev = Mock(); dev.supports.return_value = True
        dev.get_info.side_effect = {'serial':'new-device-id','asic':'usb-id','name':'RealSense D405'}.__getitem__
        rs = SimpleNamespace(camera_info=info, context=lambda:SimpleNamespace(query_devices=lambda:[dev]))
        with patch.dict('sys.modules', pyrealsense2=rs):
            for serial in ('usb-id','new-device-id'):
                self.assertIs(m._realsense_device({'identity':{'serial':serial}}, {'device':{'name':'RealSense D405'}})[2], dev)
            with self.assertRaisesRegex(RuntimeError, 'exactly one'):
                m._realsense_device({'identity':{'serial':'other'}},{'device':{'name':'RealSense D405'}})
            with self.assertRaisesRegex(ValueError, 'model mismatch'):
                m._realsense_device({'identity':{'serial':'usb-id'}},{'device':{'name':'RealSense D435I'}})

    def test_each_open_reloads_file_and_readback_failure_releases_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'preset.json'
            path.write_text(json.dumps({'parameters':{'exposure':15000}}))
            rs=SimpleNamespace()
            device=Mock()
            captures=[cameras.FakeCapture(),cameras.FakeCapture(),cameras.FakeCapture()]
            camera=m.RGBCamera({'device':0,'identity':{'serial':'usb-id'},'realsense_json':str(path)})
            with patch.object(m,'_realsense_device',return_value=(rs,object(),device)), \
                 patch.object(m.cv2,'VideoCapture',side_effect=captures), \
                 patch.object(m,'_apply_realsense_options',return_value={'verified':True}) as verify:
                camera._open().release()
                path.write_text(json.dumps({'parameters':{'exposure':12000}}))
                camera._open().release()
                self.assertEqual([c.args[2]['parameters']['exposure']
                                  for c in verify.call_args_list],[15000,12000])
                verify.side_effect=RuntimeError('readback mismatch')
                with self.assertRaisesRegex(RuntimeError,'mismatch'): camera._open()
                self.assertTrue(captures[-1].released)

    def test_sensor_readback_manual_mismatch_fails_but_auto_exposure_can_change(self):
        rs=SimpleNamespace(option=SimpleNamespace(enable_auto_exposure='auto',exposure='exposure',gain='gain'))
        sensor=Mock(); sensor.supports.return_value=True
        sensor.get_option_range.return_value=SimpleNamespace(step=1.)
        actual={'auto':0.,'exposure':15000.,'gain':16.}
        sensor.get_option.side_effect=actual.__getitem__
        device=SimpleNamespace(first_depth_sensor=lambda:sensor)
        preset={'parameters':{'controls-autoexposure-auto':'False','controls-autoexposure-manual':'15000','controls-depth-gain':'16'}}
        self.assertEqual(m._apply_realsense_options(rs,device,preset)['controls-autoexposure-manual']['actual'],15000.)
        actual['exposure']=14000.
        with self.assertRaisesRegex(RuntimeError,'read back'):m._apply_realsense_options(rs,device,preset)
        actual['auto']=1.;preset['parameters']['controls-autoexposure-auto']='True'
        m._apply_realsense_options(rs,device,preset)
        sensor.supports.return_value=False
        with self.assertRaisesRegex(RuntimeError,'does not support'):m._apply_realsense_options(rs,device,preset)


if __name__=='__main__':unittest.main()
