"""Hunt for intermittent CAN traffic: alternate bitrates, dump and decode anything seen."""
import sys
import time
from pathlib import Path

import usb.core
import libusb_package

sys.path.insert(0, str(Path(__file__).parent))
from can_comm_test import Reassembler, decode_node_status, decode_fix, DT_NAMES, DT_NODE_STATUS, DT_GNSS_FIX

from gs_usb.gs_usb import GsUsb
from gs_usb.gs_usb_frame import GsUsbFrame
from gs_usb.constants import CAN_ERR_FLAG, CAN_EFF_MASK

RX_ECHO = 0xFFFFFFFF


def main():
    backend = libusb_package.get_libusb1_backend()
    dev = usb.core.find(custom_match=GsUsb.is_gs_usb_device, backend=backend)
    if dev is None:
        raise SystemExit("[FAIL] adapter not found")
    gs = GsUsb(dev)

    t0 = time.time()
    total = 0
    rx = Reassembler()
    per_id = {}
    while time.time() - t0 < 180:
        for rate in (500000, 1000000):
            gs.set_bitrate(rate)
            gs.start()
            t_stage = time.time()
            seen = 0
            while time.time() - t_stage < 5.0:
                f = GsUsbFrame()
                if not gs.read(f, 50) or f.echo_id != RX_ECHO:
                    continue
                arb = f.can_id & CAN_EFF_MASK
                data = bytes(f.data[:f.can_dlc])
                seen += 1
                total += 1
                per_id[arb] = per_id.get(arb, 0) + 1
                now = time.time() - t0
                print(f"  t+{now:7.1f}s @{rate//1000}k: {arb:08X} [{f.can_dlc}] {data.hex(' ')}", flush=True)
                if not (f.can_id & CAN_ERR_FLAG) and not arb & 0x01000000:
                    dtype = (arb >> 8) & 0xFFFF
                    src = arb & 0xFF
                    pl = rx.feed(src, dtype, 0, 0, data)
                    if pl is not None:
                        if dtype == DT_NODE_STATUS:
                            st = decode_node_status(pl)
                            print(f"      -> NodeStatus node {src}: {st} (crc_fails so far {rx.crc_fail_count})", flush=True)
                        elif dtype == DT_GNSS_FIX:
                            fx = decode_fix(pl)
                            print(f"      -> Fix node {src}: {fx} (crc_fails so far {rx.crc_fail_count})", flush=True)
            gs.stop()
            if seen:
                print(f"  ... {seen} frames at {rate} bit/s window", flush=True)
        print(f"  ...hunting ({time.time()-t0:.0f}s elapsed, {total} total frames)", flush=True)
    print(f"\ndone: {total} frames total; per-ID: "
          + ", ".join(f"{k:08X}={v}" for k, v in sorted(per_id.items())))
    print(f"crc failures: {rx.crc_fail_count}")


if __name__ == "__main__":
    main()
