"""Live watcher: alternate 1M/500k, report valid DroneCAN frames (decoded) and
error-frame statistics per window, so wiring changes show up immediately.
"""
import sys
import time
from collections import Counter
from pathlib import Path

import usb.core
import libusb_package

sys.path.insert(0, str(Path(__file__).parent))
from can_comm_test import Reassembler, decode_node_status, decode_fix, DT_NAMES, DT_NODE_STATUS, DT_GNSS_FIX

from gs_usb.gs_usb import GsUsb
from gs_usb.gs_usb_frame import GsUsbFrame
from gs_usb.constants import CAN_ERR_FLAG, CAN_EFF_MASK

RX_ECHO = 0xFFFFFFFF
SOCKETCAN_ERR = {
    0x1: "TX_TIMEOUT", 0x2: "LOSTARB", 0x4: "CRTL", 0x8: "PROT", 0x10: "TRX",
    0x20: "ACK", 0x40: "BUSOFF", 0x80: "BUSERROR", 0x100: "RESTARTED", 0x200: "CNT",
}


def err_name(code):
    return "+".join(n for b, n in SOCKETCAN_ERR.items() if code & b) or f"0x{code:X}"


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
    backend = libusb_package.get_libusb1_backend()
    dev = usb.core.find(custom_match=GsUsb.is_gs_usb_device, backend=backend)
    if dev is None:
        raise SystemExit("[FAIL] adapter not found")
    gs = GsUsb(dev)
    print(f"watching up to {secs:.0f} s, alternating 1M/500k in 5 s windows.", flush=True)
    t0 = time.time()
    rx = Reassembler()
    total_valid = 0
    while time.time() - t0 < secs:
        for rate in (1000000, 500000):
            gs.stop()
            if not gs.set_bitrate(rate):
                continue
            gs.start()
            errs = Counter()
            valid = 0
            t_end = time.time() + 5.0
            while time.time() < t_end:
                f = GsUsbFrame()
                if not gs.read(f, 50) or f.echo_id != RX_ECHO:
                    continue
                if f.can_id & CAN_ERR_FLAG:
                    errs[f.can_id & 0x1FFFFFFF] += 1
                    continue
                valid += 1
                total_valid += 1
                arb = f.can_id & CAN_EFF_MASK
                data = bytes(f.data[:f.can_dlc])
                print(f"  [{rate//1000}k] VALID FRAME {arb:08X} [{f.can_dlc}] {data.hex(' ')}", flush=True)
                if not arb & 0x01000000:
                    dtype = (arb >> 8) & 0xFFFF
                    pl = rx.feed(arb & 0xFF, dtype, 0, 0, data)
                    if pl is not None and dtype in (DT_NODE_STATUS, DT_GNSS_FIX):
                        dec = decode_node_status(pl) if dtype == DT_NODE_STATUS else decode_fix(pl)
                        print(f"      -> {DT_NAMES[dtype]} node {arb & 0xFF}: {dec}", flush=True)
            gs.stop()
            esum = ", ".join(f"{err_name(c)}x{n}" for c, n in errs.most_common()) or "none"
            print(f"  [{rate//1000}k] window done: valid={valid}, errors: {esum} "
                  f"(elapsed {time.time()-t0:.0f}s, total valid {total_valid})", flush=True)
    print(f"watch finished, total valid frames: {total_valid}")


if __name__ == "__main__":
    main()
