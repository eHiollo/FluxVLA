"""m_init triggers opens once, without waiting for gripper arrival or real hardware."""
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch
from mmengine import Config
from fluxvla.engines.operators.kaanh_commands.init_grippers import open_configured_grippers
from fluxvla.engines.operators.vr.runtime import _ZXGripperChannel
from test_vr_zx_gripper import FakeZX, device, options, wait_for


class InitGripperTest(unittest.TestCase):
    def test_console_fires_once_without_waiting_for_position(self):
        devices = {side: device(side) for side in ('left', 'right')}
        robot = SimpleNamespace(init_grippers_config='explicit-config', _traj_stop_event=threading.Event())
        clients = [MagicMock(), MagicMock()]
        for client in clients:
            client.__enter__.return_value = client
            client.read_state.return_value = SimpleNamespace(enabled=True, alarm_a=0, alarm_b=0, position_raw=4500)
        with patch('mmengine.Config.fromfile', return_value={'grippers': devices}), patch(
                'fluxvla.engines.operators.zx_gripper.ZXGripper', side_effect=clients):
            self.assertTrue(open_configured_grippers(robot))
        for client in clients:
            client.set_position_raw.assert_called_once()
            self.assertEqual(client.set_position_raw.call_args.args, (1, 0))
            self.assertFalse(client.set_position_raw.call_args.kwargs['wait'])
            client.read_state.assert_called_once()  # No arrival feedback loop.
            client.enable.assert_not_called()

    def test_vr_existing_worker_fires_once_and_ignores_arrival(self):
        channel = _ZXGripperChannel('left', device(), options())
        self.addCleanup(channel.close)
        fake = FakeZX()
        fake.set_position_raw = MagicMock()  # Keep actual at 4500, never arrives.
        channel.client = fake
        channel.connect()
        self.assertIsNone(channel.request_init_open(stop_event=threading.Event()))
        wait_for(lambda: fake.set_position_raw.call_count == 1)
        self.assertEqual(fake.set_position_raw.call_args.args, (1, 0))
        self.assertFalse(fake.set_position_raw.call_args.kwargs['wait'])
        self.assertIsNone(channel._init_open_request)
        self.assertEqual(fake.position, 4500)

    def test_virtual_configuration_never_opens_live_usb(self):
        config = Config.fromfile(str(Path(__file__).resolve().parents[1] / 'configs/a10/a10_dual_vr_sim.py'))
        self.assertIsNone(config.operator.init_grippers_config)
        self.assertFalse(config.teleop.zx_grippers.enabled)
        with patch('fluxvla.engines.operators.zx_gripper.ZXGripper') as factory:
            self.assertFalse(open_configured_grippers(SimpleNamespace(init_grippers_config=None)))
        factory.assert_not_called()
