"""Diagnostics when passive sniffing finds no traffic.

1) Loopback self-test of the candleLight adapter (proves USB<->CAN controller path).
2) Bitrate scan: 1M / 500k / 250k / 125k, short passive sniffs at each.
3) TX error counter probe at each bitrate: send one frame, read BERR counters.
   Rising TX error count => nobody on the bus ACKs (no powered node / broken wiring).
"""
import struct
import sys
import time

import usb.core
import libusb_package

from gs_usb.gs_usb import GsUsb
from gs_usb.gs_usb_frame import GsUsbFrame
from gs_usb.constants import GS_CAN_MODE_LOOP_BACK, GS_CAN_MODE_HW_TIMESTAMP, CAN_EFF_FLAG

GS_USB_BREQ_BERR = 3
RX_ECHO = 0xFFFFFFFF


def read_berr(gs):
    try:
        data = gs.gs_usb.ctrl_transfer(0xC1, GS_USB_BREQ_BERR, 0, 0, 8)
        txerr, rxerr = struct.unpack_from("<HH", bytes(data))
        return txerr, rxerr
    except usb.core.USBError as e:
        return None


def loopback_test(gs):
    print("== LOOPBACK SELF-TEST ==")
    gs.stop()
    gs.set_bitrate(1000000)
    gs.start(flags=GS_CAN_MODE_LOOP_BACK | GS_CAN_MODE_HW_TIMESTAMP)
    time.sleep(0.05)
    tx = GsUsbFrame(can_id=CAN_EFF_FLAG | 0x123, data=[1, 2, 3, 4, 5, 6, 7, 8])
    gs.send(tx)
    ok = False
    deadline = time.time() + 1.0
    while time.time() < deadline:
        f = GsUsbFrame()
        if gs.read(f, 100) and f.echo_id == RX_ECHO:
            print(f"   loopback RX: {f.arbitration_id:08X} [{f.can_dlc}] "
                  f"{bytes(f.data[:f.can_dlc]).hex(' ')}")
            ok = True
            break
    print("   loopback:", "PASS - adapter TX/RX path works" if ok else "FAIL - no loopback frame")
    gs.stop()
    return ok


def sniff_count(gs, seconds):
    n = 0
    t_end = time.time() + seconds
    while time.time() < t_end:
        f = GsUsbFrame()
        if gs.read(f, 50) and f.echo_id == RX_ECHO:
            n += 1
    return n


def bitrate_scan(gs):
    print("\n== BITRATE SCAN (passive 3 s per rate, then 1 TX frame + BERR probe) ==")
    results = {}
    for rate in (1000000, 500000, 250000, 125000):
        try:
            if not gs.set_bitrate(rate):
                print(f"  {rate:>8} bit/s: timing not supported")
                continue
            gs.start()
            time.sleep(0.05)
            passive = sniff_count(gs, 3.0)
            berr0 = read_berr(gs)
            tx = GsUsbFrame(can_id=CAN_EFF_FLAG | 0x00000055, data=[0xDE, 0xAD])
            gs.send(tx)
            time.sleep(0.2)
            after = sniff_count(gs, 0.5)
            berr1 = read_berr(gs)
            gs.stop()
            results[rate] = dict(passive=passive, after_tx=after, berr0=berr0, berr1=berr1)
            print(f"  {rate:>8} bit/s: passive_rx={passive:>4}  rx_after_tx={after:>4}  "
                  f"BERR tx/rx {berr0} -> {berr1}")
        except usb.core.USBError as e:
            print(f"  {rate:>8} bit/s: USBError {e}")
            try:
                gs.stop()
            except Exception:
                pass
    return results


def main():
    backend = libusb_package.get_libusb1_backend()
    dev = usb.core.find(custom_match=GsUsb.is_gs_usb_device, backend=backend)
    if dev is None:
        raise SystemExit("[FAIL] adapter not found")
    gs = GsUsb(dev)
    cap = gs.device_capability
    print(f"[ADAPTER] {gs} serial={gs.serial_number} fclk={cap.fclk_can} feature=0x{cap.feature:08x}")
    lb = loopback_test(gs)
    res = bitrate_scan(gs)
    print("\n== SUMMARY ==")
    print(f"  adapter loopback: {'PASS' if lb else 'FAIL'}")
    traffic = {r: v for r, v in res.items() if v["passive"] > 0}
    if traffic:
        for r, v in traffic.items():
            print(f"  traffic seen at {r} bit/s: {v['passive']} passive frames")
    else:
        print("  NO traffic at any scanned bitrate (1M/500k/250k/125k).")
        any_txerr = [v for v in res.values() if v["berr1"] and v["berr1"][0] > 0]
        if any_txerr:
            print("  TX error counter rises when sending => frames are NOT acknowledged.")
            print("  Likely causes: GPS not powered, CAN_H/CAN_L swapped/open,")
            print("  missing 120R termination, or module in standby.")
        else:
            print("  TX error counter did not rise (or unreadable) - inconclusive for ACK detection.")


if __name__ == "__main__":
    sys.exit(main())
