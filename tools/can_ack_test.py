"""TX-echo / ACK probe.

candleLight reports a TX echo for every frame it transmits. Comparing loopback mode
(internally self-acknowledged, no peer needed) with normal mode tells us:
  * whether the firmware emits echoes at all;
  * whether a real peer on the bus acknowledges our classic CAN frames
    (an FD-capable GPS still ACKs classic frames, so this detects "module present,
    powered, wired and at this bitrate" even when its own traffic is undecodable).
"""
import struct
import sys
import time

import usb.core
import libusb_package

from gs_usb.gs_usb import GsUsb
from gs_usb.gs_usb_frame import GsUsbFrame
from gs_usb.constants import GS_CAN_MODE_LOOP_BACK, GS_CAN_MODE_HW_TIMESTAMP, CAN_EFF_FLAG, CAN_EFF_MASK

RX_ECHO = 0xFFFFFFFF


def run_phase(gs, label, rate, flags, n=5, wait=1.0):
    gs.stop()
    if not gs.set_bitrate(rate):
        print(f"  {label}: bitrate unsupported")
        return
    gs.start(flags=flags)
    time.sleep(0.05)
    sent = []
    for i in range(n):
        eid = 100 + i
        sent.append(eid)
        gs.send(GsUsbFrame(can_id=CAN_EFF_FLAG | (0x0BD00000 | i), data=[0xAA, i, 0x55, 0x42]))
        time.sleep(0.01)
    echoes = []
    rx = []
    t_end = time.time() + wait
    while time.time() < t_end:
        f = GsUsbFrame()
        if not gs.read(f, 50):
            continue
        if f.echo_id == RX_ECHO:
            rx.append((f.arbitration_id, bytes(f.data[:f.can_dlc])))
        else:
            echoes.append((f.echo_id, f.arbitration_id))
    gs.stop()
    print(f"  {label}: sent {n}, TX-echoes={len(echoes)} {echoes[:6]}, RX frames={len(rx)} "
          f"{[f'{a:08X}' for a, _ in rx[:6]]}")
    return len(echoes), len(rx)


def main():
    backend = libusb_package.get_libusb1_backend()
    dev = usb.core.find(custom_match=GsUsb.is_gs_usb_device, backend=backend)
    if dev is None:
        raise SystemExit("[FAIL] adapter not found")
    gs = GsUsb(dev)
    print("TX echo / ACK probe")
    run_phase(gs, "loopback 1M (self-ACKed, no peer needed)", 1000000,
              GS_CAN_MODE_LOOP_BACK | GS_CAN_MODE_HW_TIMESTAMP)
    run_phase(gs, "normal   1M (needs real peer to ACK)", 1000000, GS_CAN_MODE_HW_TIMESTAMP)
    run_phase(gs, "normal 500k (needs real peer to ACK)", 500000, GS_CAN_MODE_HW_TIMESTAMP)
    print("\ninterpretation:")
    print("  loopback echoes present + normal echoes present -> a live peer ACKs us on that bus")
    print("  loopback echoes present + normal echoes absent  -> nobody ACKs (link/bitrate/power problem)")
    print("  loopback echoes absent                          -> firmware sends no echo; test inconclusive")


if __name__ == "__main__":
    main()
