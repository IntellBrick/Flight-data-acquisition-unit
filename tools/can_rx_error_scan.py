"""Steady-state RX error scan.

Goal: decide whether anything is actively transmitting on the bus that a classic-CAN
controller cannot decode (CAN FD framing, or a wildly different bitrate).

Method per bitrate: start -> discard a settling window -> measure a steady-state window.
With BERR_REPORTING unavailable, undecodable traffic still shows up because the receive
error counter crossing 96/128 produces controller state-change error frames.
Our own TX is disabled (no frames sent), so any ACK errors are impossible by construction.
"""
import sys
import time
from collections import Counter

import usb.core
import libusb_package

from gs_usb.gs_usb import GsUsb
from gs_usb.gs_usb_frame import GsUsbFrame
from gs_usb.constants import CAN_ERR_FLAG

RX_ECHO = 0xFFFFFFFF
ERRBITS = {0x1: "TX_TIMEOUT", 0x2: "LOSTARB", 0x4: "CRTL", 0x8: "PROT", 0x10: "TRX",
           0x20: "ACK", 0x40: "BUSOFF", 0x80: "BUSERROR", 0x100: "RESTARTED", 0x200: "CNT"}
CRTL = {0x01: "RX_OVERFLOW", 0x02: "TX_WARNING", 0x04: "RX_WARNING", 0x08: "TX_PASSIVE",
        0x10: "RX_PASSIVE", 0x20: "ACTIVE"}


def err_desc(code):
    return "+".join(n for b, n in ERRBITS.items() if code & b) or f"0x{code:X}"


def window(gs, seconds):
    valid = err = 0
    codes = Counter()
    t_end = time.time() + seconds
    while time.time() < t_end:
        f = GsUsbFrame()
        if not gs.read(f, 50) or f.echo_id != RX_ECHO:
            continue
        if f.can_id & CAN_ERR_FLAG:
            err += 1
            codes[(f.can_id & 0x1FFFFFFF, bytes(f.data[:f.can_dlc]))] += 1
        else:
            valid += 1
    return valid, err, codes


def main():
    rates = [int(x) for x in sys.argv[1:]] or [1000000, 500000]
    backend = libusb_package.get_libusb1_backend()
    dev = usb.core.find(custom_match=GsUsb.is_gs_usb_device, backend=backend)
    if dev is None:
        raise SystemExit("[FAIL] adapter not found")
    gs = GsUsb(dev)
    for rate in rates:
        gs.stop()
        if not gs.set_bitrate(rate):
            print(f"{rate}: unsupported")
            continue
        gs.start()
        v0, e0, _ = window(gs, 3.0)
        v1, e1, codes = window(gs, 8.0)
        print(f"{rate/1000:>6.0f} kbit/s | settling 3 s: valid={v0} err={e0} "
              f"| steady 8 s: valid={v1} err={e1} ({e1/8.0:.1f}/s)")
        for (code, data), n in codes.most_common(4):
            ctrl = data[1] if len(data) > 1 else None
            ctrl_s = f" crtl_state={CRTL.get(ctrl, hex(ctrl))}" if ctrl is not None else ""
            print(f"         x{n:<5} {err_desc(code)}{ctrl_s} data={data.hex(' ')}")
        gs.stop()
    print("\nno TX was performed: any error here means our controller is failing to")
    print("decode signalling that is actually present on the bus.")


if __name__ == "__main__":
    main()
