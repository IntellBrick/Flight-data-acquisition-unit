"""Read CAN controller state + RX/TX error counters via GS_USB_BREQ_GET_STATE (14).

This is the hardest evidence available on this adapter:
  state ERROR_ACTIVE + txerr 0   -> our frames were acknowledged by a live peer
  txerr rising / BUS_OFF         -> nobody acknowledges us (no live peer / bad link)
  rxerr rising                   -> we are receiving corrupt signalling (bitrate/PHY issue)
"""
import struct
import sys
import time

import usb.core
import libusb_package

from gs_usb.gs_usb import GsUsb
from gs_usb.gs_usb_frame import GsUsbFrame
from gs_usb.constants import CAN_EFF_FLAG, CAN_ERR_FLAG, CAN_EFF_MASK

BREQ_BERR = 3
BREQ_IDENTIFY = 7
BREQ_GET_STATE = 14
RX_ECHO = 0xFFFFFFFF

STATES = {
    0: "ERROR_ACTIVE", 1: "ERROR_WARNING", 2: "ERROR_PASSIVE",
    3: "BUS_OFF", 4: "STOPPED", 5: "SLEEPING",
}


def get_state(gs):
    try:
        data = bytes(gs.gs_usb.ctrl_transfer(0xC1, BREQ_GET_STATE, 0, 0, 12))
        state, rxerr, txerr = struct.unpack_from("<3I", data)
        return state, rxerr, txerr
    except usb.core.USBError as e:
        return f"unsupported ({e.errno})"


def show(label, st):
    if isinstance(st, tuple):
        s, rx, tx = st
        print(f"    {label:<28} state={STATES.get(s, s)} ({s})  rxerr={rx}  txerr={tx}")
    else:
        print(f"    {label:<28} {st}")


def drain(gs, seconds, count_errors=False):
    valid = err = 0
    t_end = time.time() + seconds
    while time.time() < t_end:
        f = GsUsbFrame()
        if not gs.read(f, 50) or f.echo_id != RX_ECHO:
            continue
        if f.can_id & CAN_ERR_FLAG:
            err += 1
        else:
            valid += 1
    return valid, err


def main():
    backend = libusb_package.get_libusb1_backend()
    dev = usb.core.find(custom_match=GsUsb.is_gs_usb_device, backend=backend)
    if dev is None:
        raise SystemExit("[FAIL] adapter not found")
    gs = GsUsb(dev)
    cap = gs.device_capability
    print(f"adapter feature=0x{cap.feature:08x} fclk={cap.fclk_can}")

    # does this firmware support BERR?
    try:
        raw = bytes(gs.gs_usb.ctrl_transfer(0xC1, BREQ_BERR, 0, 0, 8))
        print(f"BERR(3) raw: {raw.hex(' ')}")
    except usb.core.USBError as e:
        print(f"BERR(3) unsupported: {e.errno}")

    for rate in (1000000, 500000, 250000):
        print(f"\n=== {rate/1000:.0f} kbit/s ===")
        gs.stop()
        if not gs.set_bitrate(rate):
            print("    bitrate unsupported")
            continue
        gs.start()
        time.sleep(0.1)
        show("after start (baseline)", get_state(gs))
        v, e = drain(gs, 2.0)
        print(f"    listen-only 2 s: valid={v} error_frames={e}")
        show("after listening", get_state(gs))
        for i in range(5):
            fr = GsUsbFrame(can_id=CAN_EFF_FLAG | 0x0BD00000 | i, data=[0x5A, i])
            gs.send(fr)
            time.sleep(0.01)
        time.sleep(0.3)
        show("after TX 5 frames", get_state(gs))
        v, e = drain(gs, 1.0)
        print(f"    after TX 1 s: valid={v} error_frames={e}")
        show("final", get_state(gs))
        gs.stop()


if __name__ == "__main__":
    main()
