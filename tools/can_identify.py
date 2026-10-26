"""LED semantics experiment.

Upstream candleLight firmware has no "both LEDs solid" state:
  * LED_MODE_NORMAL -> LEDs dark, 45 ms blink per RX/TX frame
  * LED_MODE_OFF    -> both dark
  * LED_MODE_SEQUENCE (identify) -> RX on 100 ms, TX on 100 ms, repeating

So: if the LEDs on this adapter are firmware-driven, sending IDENTIFY will switch them
from whatever they show now to an alternating 100 ms RX/TX pattern.
If they do not change at all, the LEDs are hardware/line-state driven (or vendor firmware).

Usage: python can_identify.py [seconds]
"""
import struct
import sys
import time

import usb.core
import libusb_package

from gs_usb.gs_usb import GsUsb
from gs_usb.gs_usb_frame import GsUsbFrame

GS_USB_BREQ_IDENTIFY = 7
RX_ECHO = 0xFFFFFFFF


def identify(gs, on):
    try:
        gs.gs_usb.ctrl_transfer(0x41, GS_USB_BREQ_IDENTIFY, 0, 0, struct.pack("<I", 1 if on else 0))
        return True
    except usb.core.USBError as e:
        print(f"  IDENTIFY {'on' if on else 'off'} failed: {e}")
        return False


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
    backend = libusb_package.get_libusb1_backend()
    dev = usb.core.find(custom_match=GsUsb.is_gs_usb_device, backend=backend)
    if dev is None:
        raise SystemExit("[FAIL] adapter not found")
    gs = GsUsb(dev)
    cap = gs.device_capability
    print(f"adapter feature=0x{cap.feature:08x}  IDENTIFY supported: {bool(cap.feature & (1 << 5))}")

    gs.set_bitrate(1000000)
    gs.start()
    time.sleep(0.2)
    print("==> BEFORE identify: channel started, no bus traffic. Watch the LEDs now.")
    time.sleep(3.0)

    print("==> IDENTIFY ON: if LEDs are firmware-driven you should now see RX and TX")
    print("    blinking alternately at 100 ms (about 5 blinks/second each).")
    ok = identify(gs, True)
    if ok:
        t_end = time.time() + secs
        while time.time() < t_end:
            f = GsUsbFrame()
            gs.read(f, 100)  # keep the endpoint drained
        identify(gs, False)
        print("==> IDENTIFY OFF: LEDs should return to their previous state.")
    gs.stop()


if __name__ == "__main__":
    main()
