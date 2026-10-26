"""Print RAW frames (unmasked can_id incl. flags, echo_id, dlc, data) to classify
the error-frame storm seen when transmitting. Also compares with pure listen-only.
"""
import sys
import time
from collections import Counter

import usb.core
import libusb_package

from gs_usb.gs_usb import GsUsb
from gs_usb.gs_usb_frame import GsUsbFrame
from gs_usb.constants import GS_CAN_MODE_LISTEN_ONLY, GS_CAN_MODE_HW_TIMESTAMP, CAN_EFF_FLAG

RX_ECHO = 0xFFFFFFFF


def collect(gs, seconds, keep=12):
    frames = []
    counts = Counter()
    t_end = time.time() + seconds
    while time.time() < t_end:
        f = GsUsbFrame()
        if not gs.read(f, 50):
            continue
        key = (f.echo_id, f.can_id, f.can_dlc, f.flags)
        counts[key] += 1
        if len(frames) < keep:
            frames.append((f.echo_id, f.can_id, f.can_dlc, f.flags, bytes(f.data[:f.can_dlc])))
    return counts, frames


def show(label, counts, frames, total):
    print(f"  {label}: {total} frames, {len(counts)} distinct (echo_id, can_id, dlc, flags) patterns")
    for (eid, cid, dlc, flags), c in counts.most_common(6):
        tag = ""
        if cid & 0x80000000:
            tag += " EFF"
        if cid & 0x40000000:
            tag += " RTR"
        if cid & 0x20000000:
            tag += " ERR"
        print(f"      x{c:<6} echo_id={eid:#010x} can_id={cid:#010x}{tag} dlc={dlc} flags={flags:#x}")
    for eid, cid, dlc, flags, data in frames[:6]:
        print(f"        sample: echo_id={eid:#010x} can_id={cid:#010x} dlc={dlc} data={data.hex(' ')}")


def main():
    backend = libusb_package.get_libusb1_backend()
    dev = usb.core.find(custom_match=GsUsb.is_gs_usb_device, backend=backend)
    if dev is None:
        raise SystemExit("[FAIL] adapter not found")
    gs = GsUsb(dev)

    for rate in (1000000, 500000):
        print(f"\n=== {rate/1000:.0f} kbit/s ===")

        # 1) pure listen-only, no transmission at all
        gs.stop()
        gs.set_bitrate(rate)
        gs.start(flags=GS_CAN_MODE_LISTEN_ONLY | GS_CAN_MODE_HW_TIMESTAMP)
        time.sleep(0.05)
        counts, frames = collect(gs, 4.0)
        show("listen-only 4 s", counts, frames, sum(counts.values()))
        gs.stop()

        # 2) normal mode, transmit 3 frames, then observe
        gs.set_bitrate(rate)
        gs.start(flags=GS_CAN_MODE_HW_TIMESTAMP)
        time.sleep(0.05)
        for i in range(3):
            fr = GsUsbFrame(can_id=CAN_EFF_FLAG | 0x0BD00000 | i, data=[0x11, i])
            fr.echo_id = 555 + i
            gs.send(fr)
            time.sleep(0.02)
        counts, frames = collect(gs, 1.0)
        show("after TX 3 frames, 1 s", counts, frames, sum(counts.values()))
        gs.stop()


if __name__ == "__main__":
    main()
