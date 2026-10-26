"""Probe CAN sample-point variants: a bitrate-matched but sample-point-mismatched peer
can still fail to decode frames, especially with a marginal (unterminated) bus.
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

# (name, prop_seg, phase_seg1, phase_seg2, sjw, brp)  -- 48 MHz CAN clock, 16 tq total
COMBOS = [
    ("87.5%", 1, 12, 2, 1),
    ("81.25%", 1, 11, 3, 1),
    ("75.0%", 1, 10, 4, 1),
]
RATES = [1000000, 500000]


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
    backend = libusb_package.get_libusb1_backend()
    dev = usb.core.find(custom_match=GsUsb.is_gs_usb_device, backend=backend)
    if dev is None:
        raise SystemExit("[FAIL] adapter not found")
    gs = GsUsb(dev)
    print(f"sample-point probe, {secs:.0f} s per combination (normal mode, adapter ACKs traffic)")
    hits = 0
    for rate in RATES:
        brp = 48_000_000 // (rate * 16)
        for name, prop, seg1, seg2, sjw in COMBOS:
            gs.set_timing(prop, seg1, seg2, sjw, brp)
            gs.start()
            time.sleep(0.05)
            rx = Reassembler()
            frames = []
            t_end = time.time() + secs
            while time.time() < t_end:
                f = GsUsbFrame()
                if not gs.read(f, 50) or f.echo_id != RX_ECHO:
                    continue
                arb = f.can_id & CAN_EFF_MASK
                data = bytes(f.data[:f.can_dlc])
                frames.append((arb, f.can_dlc, data))
                if not (f.can_id & CAN_ERR_FLAG) and not arb & 0x01000000:
                    dtype = (arb >> 8) & 0xFFFF
                    pl = rx.feed(arb & 0xFF, dtype, 0, 0, data)
                    if pl is not None and dtype in (DT_NODE_STATUS, DT_GNSS_FIX):
                        dec = decode_node_status(pl) if dtype == DT_NODE_STATUS else decode_fix(pl)
                        print(f"    DECODED {DT_NAMES[dtype]} node {arb & 0xFF}: {dec}", flush=True)
            gs.stop()
            hits += len(frames)
            tag = "  <== TRAFFIC" if frames else ""
            print(f"  {rate/1000:>6.0f}k @ sample point {name:>7} (brp={brp}): {len(frames)} frames{tag}")
            for arb, dlc, data in frames[:10]:
                print(f"      {arb:08X} [{dlc}] {data.hex(' ')}")
    print(f"\ntotal frames across all combinations: {hits}")


if __name__ == "__main__":
    main()
