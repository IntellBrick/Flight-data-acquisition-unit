"""Long passive listen: print every received frame with deltas, detect patterns."""
import argparse
import sys
import time

import usb.core
import libusb_package

from gs_usb.gs_usb import GsUsb
from gs_usb.gs_usb_frame import GsUsbFrame
from gs_usb.constants import CAN_ERR_FLAG, CAN_EFF_MASK, CAN_EFF_FLAG

RX_ECHO = 0xFFFFFFFF


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bitrate", type=int, default=500000)
    ap.add_argument("--secs", type=float, default=30.0)
    args = ap.parse_args()

    backend = libusb_package.get_libusb1_backend()
    dev = usb.core.find(custom_match=GsUsb.is_gs_usb_device, backend=backend)
    if dev is None:
        raise SystemExit("[FAIL] adapter not found")
    gs = GsUsb(dev)
    if not gs.set_bitrate(args.bitrate):
        raise SystemExit("[FAIL] bitrate unsupported")
    gs.start()
    print(f"listening {args.secs:.0f} s at {args.bitrate} bit/s ...")
    t0 = time.time()
    last = {}
    n = 0
    errs = 0
    while time.time() - t0 < args.secs:
        f = GsUsbFrame()
        if not gs.read(f, 100):
            continue
        if f.echo_id != RX_ECHO:
            continue
        n += 1
        now = time.time() - t0
        arb = f.can_id & CAN_EFF_MASK
        if f.can_id & CAN_ERR_FLAG:
            errs += 1
            print(f"  {now:8.3f}  ERRORFRAME raw={f.can_id:08X}")
            continue
        data = bytes(f.data[:f.can_dlc])
        d = f"{now - last[arb]:7.3f}" if arb in last else "      -"
        last[arb] = now
        print(f"  {now:8.3f}  {arb:08X} [{f.can_dlc}] {data.hex(' ')}  dt={d}")
    print(f"\ntotal {n} frames ({errs} error frames) in {args.secs:.0f} s")
    print("per-ID counts:")
    gs.stop()


if __name__ == "__main__":
    sys.exit(main())
