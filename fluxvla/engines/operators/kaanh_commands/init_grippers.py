"""Explicit m_init gripper opening; never enables a device or borrows an open port."""

import logging
from contextlib import ExitStack


LOG = logging.getLogger(__name__)


def open_configured_grippers(robot, *, cancel_event=None):
    """Use an existing VR owner, or explicitly configured exclusive serial ports."""
    callback = getattr(robot, '_open_init_grippers', None)
    if callback is not None:
        return callback(cancel_event=cancel_event)
    path = getattr(robot, 'init_grippers_config', None)
    if path is None:
        return False
    from mmengine import Config
    from ..zx_gripper import ZXGripper, has_blocking_alarm
    devices = Config.fromfile(str(path))['grippers']
    if set(devices) != {'left', 'right'}:
        raise ValueError('m_init requires both physical grippers in its device config')
    def check_cancelled():
        if (robot._traj_stop_event.is_set() or
                (cancel_event is not None and cancel_event.is_set())):
            raise InterruptedError('m_init gripper opening cancelled')
    with ExitStack() as stack:
        ready = {}
        # Validate both devices before sending a target to either one.
        for side, device in devices.items():
            check_cancelled()
            identity, check = device['identity'], device['check']
            client = stack.enter_context(ZXGripper(
                device['port'], baudrate=device['baudrate'], timeout=.2,
                max_position_raw=check['max_raw'],
                drop_alarm_policy='warn',
                expected_usb_serial=identity['serial'],
                expected_usb_interface=identity['usb_interface']))
            state = client.read_state(device['device_id'])
            if has_blocking_alarm(state.alarm_a, state.alarm_b, drop_alarm_policy='warn'):
                LOG.warning('m_init 跳过 %s 夹爪的打开动作：报警 0x%04X/0x%04X',
                            side, state.alarm_a, state.alarm_b)
                continue
            if not state.enabled:
                raise RuntimeError(f'ZX {side} is disabled; m_init does not enable grippers')
            ready[side] = (client, state)
        for side, (client, state) in ready.items():
            check_cancelled()
            device = devices[side]
            client.set_position_raw(
                device['device_id'], device['check']['open_raw'],
                speed=device['speed'], torque=device['torque'], feedback=state,
                wait=False, motion_cancel_event=cancel_event or robot._traj_stop_event)
    return len(ready) == len(devices)
