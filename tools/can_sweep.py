"""Systematic bitrate sweep with raw frame dump + UAVCAN v0 decode attempt.

Real DroneCAN frames will pass the transfer CRC; mis-synced garbage will not.
"""
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


def sweep(gs, rates, secs):
    results = {}
    for rate in rates:
        if not gs.set_bitrate(rate):
            print(f"  {rate:>8}: unsupported timing, skip")
            continue
        gs.start()
        time.sleep(0.05)
        rx = Reassembler()
        raw = []
        decoded = []
        t_end = time.time() + secs
        while time.time() < t_end:
            f = GsUsbFrame()
            if not gs.read(f, 50) or f.echo_id != RX_ECHO:
                continue
            arb = f.can_id & CAN_EFF_MASK
            if f.can_id & CAN_ERR_FLAG:
                raw.append((time.time(), arb | 0x20000000, f.can_dlc, bytes(f.data[:f.can_dlc])))
                continue
            data = bytes(f.data[:f.can_dlc])
            raw.append((time.time(), arb, f.can_dlc, data))
            if not arb & 0x01000000:  # message frame
                dtype = (arb >> 8) & 0xFFFF
                src = arb & 0xFF
                pl = rx.feed(src, dtype, 0, 0, data)
                if pl is not None:
                    if dtype == DT_NODE_STATUS:
                        st = decode_node_status(pl)
                        if st:
                            decoded.append((src, dtype, st, rx.crc_fail_count))
                    elif dtype == DT_GNSS_FIX:
                        fx = decode_fix(pl)
                        if fx:
                            decoded.append((src, dtype, fx, rx.crc_fail_count))
        gs.stop()
        results[rate] = (raw, decoded, rx.crc_fail_count)
        print(f"  {rate:>8} bit/s: {len(raw)} raw frames, {len(decoded)} decoded transfers, "
              f"{rx.crc_fail_count} crc failures")
        for ts, arb, dlc, data in raw[:40]:
            err = " (ERROR)" if arb & 0x20000000 else ""
            print(f"      t+{ts % secs:6.2f}s  {arb & 0x1FFFFFFF:08X} [{dlc}] {data.hex(' ')}{err}")
        for src, dt, obj, _ in decoded:
            print(f"      decoded: node {src} {DT_NAMES.get(dt, dt)}: {obj}")
    return results


def main():
    backend = libusb_package.get_libusb1_backend()
    dev = usb.core.find(custom_match=GsUsb.is_gs_usb_device, backend=backend)
    if dev is None:
        raise SystemExit("[FAIL] adapter not found")
    gs = GsUsb(dev)
    print("bitrate sweep, 6 s per rate:")
    sweep(gs, (1000000, 800000, 500000, 250000, 125000, 100000), 6.0)


if __name__ == "__main__":
    main()
