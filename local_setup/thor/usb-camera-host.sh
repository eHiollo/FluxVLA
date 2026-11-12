#!/bin/sh
# Restore the Thor USB-C camera hub after the vendor device-mode setup.
set -eu

case "$(tr -d '\000' < /proc/device-tree/model)" in
    *'Jetson AGX Thor'*) ;;
    *) echo 'This setup is only for the Jetson AGX Thor developer kit.' >&2; exit 1 ;;
esac

usb_role_path=/sys/class/usb_role/usb2-0-role-switch/role
usb_role_before=$(cat "$usb_role_path")
echo "USB-C data role before: $usb_role_before"
if [ "$usb_role_before" != host ]; then
    # Switching device -> host directly may fail to enumerate the attached hub.
    printf '%s\n' none > "$usb_role_path"
    sleep 0.5
    printf '%s\n' host > "$usb_role_path"
fi
test "$(cat "$usb_role_path")" = host
echo 'USB-C data role confirmed: host'
