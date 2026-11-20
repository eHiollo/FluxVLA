"""Explicit m_init gripper opening; never enables a device or borrows an open port."""

from contextlib import ExitStack


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
        opened = {}
        # Open and validate BOTH devices before sending either target.
        for side, device in devices.items():
            check_cancelled()
            identity, check = device['identity'], device['check']
            client = stack.enter_context(ZXGripper(
                device['port'], baudrate=device['baudrate'], timeout=.2,
                max_position_raw=check['max_raw'],
                expected_usb_serial=identity['serial'],
                expected_usb_interface=identity['usb_interface']))
            state = client.read_state(device['device_id'])
            if not state.enabled:
                raise RuntimeError(f'ZX {side} is disabled; m_init does not enable grippers')
            if has_blocking_alarm(state.alarm_a, state.alarm_b):
                raise RuntimeError(f'ZX {side} alarm blocks m_init opening')
            opened[side] = (client, state)
        for side, device in devices.items():
            check_cancelled()
            client, state = opened[side]
            client.set_position_raw(
                device['device_id'], device['check']['open_raw'],
                speed=device['speed'], torque=device['torque'], feedback=state,
                wait=False, motion_cancel_event=cancel_event or robot._traj_stop_event)
    return True
