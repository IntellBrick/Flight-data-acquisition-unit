"""One-shot verification after a wiring change.

Pipeline:
  1. adapter present + loopback self-test (separates adapter faults from bus faults)
  2. short steady-state scan at 1M / 500k / 250k
  3. as soon as traffic appears, run the full test at that bitrate:
     decode uavcan.protocol.NodeStatus (341) and gnss.Fix (1060) transfers,
     then send uavcan.protocol.GetNodeInfo to every node seen and print
     the device name / software / hardware version.

Usage: python can_quickcheck.py [--sniff 10]
"""
import argparse
import json
import sys
import time
from pathlib import Path

import usb.core
import libusb_package

sys.path.insert(0, str(Path(__file__).parent))
from can_comm_test import (
    Reassembler, decode_node_status, decode_fix, parse_get_node_info,
    DT_NAMES, DT_NODE_STATUS, DT_GNSS_FIX, MY_NODE_ID,
)
from can_state_probe import get_state, STATES

from gs_usb.gs_usb import GsUsb
from gs_usb.gs_usb_frame import GsUsbFrame
from gs_usb.constants import (
    GS_CAN_MODE_LOOP_BACK, GS_CAN_MODE_HW_TIMESTAMP,
    CAN_EFF_FLAG, CAN_ERR_FLAG, CAN_EFF_MASK,
)

RX_ECHO = 0xFFFFFFFF
WS = Path(r"D:\Lab\Flight-data-acquisition-unit")


def open_adapter():
    backend = libusb_package.get_libusb1_backend()
    dev = usb.core.find(custom_match=GsUsb.is_gs_usb_device, backend=backend)
    if dev is None:
        raise SystemExit("[FAIL] candleLight adapter not found on USB")
    gs = GsUsb(dev)
    print(f"[ADAPTER] {gs} serial={gs.serial_number}")
    return gs


def loopback_selftest(gs):
    gs.stop()
    gs.set_bitrate(1000000)
    gs.start(flags=GS_CAN_MODE_LOOP_BACK | GS_CAN_MODE_HW_TIMESTAMP)
    time.sleep(0.05)
    gs.send(GsUsbFrame(can_id=CAN_EFF_FLAG | 0x123, data=[1, 2, 3, 4, 5, 6, 7, 8]))
    ok = False
    deadline = time.time() + 1.0
    while time.time() < deadline:
        f = GsUsbFrame()
        if gs.read(f, 100) and f.echo_id == RX_ECHO:
            ok = True
            break
    gs.stop()
    print(f"[SELFTEST] adapter loopback: {'PASS' if ok else 'FAIL'}")
    return ok


def collect(gs, seconds):
    valid = err = 0
    rx = Reassembler()
    nodes = {}
    fixes = {}
    t_end = time.time() + seconds
    while time.time() < t_end:
        f = GsUsbFrame()
        if not gs.read(f, 50) or f.echo_id != RX_ECHO:
            continue
        arb = f.can_id & CAN_EFF_MASK
        if f.can_id & CAN_ERR_FLAG:
            err += 1
            continue
        valid += 1
        data = bytes(f.data[:f.can_dlc])
        if not arb & 0x01000000:
            dtype = (arb >> 8) & 0xFFFF
            src = arb & 0xFF
            pl = rx.feed(src, dtype, 0, 0, data)
            if pl is not None:
                if dtype == DT_NODE_STATUS:
                    st = decode_node_status(pl)
                    if st:
                        nodes[src] = st
                elif dtype == DT_GNSS_FIX:
                    fx = decode_fix(pl)
                    if fx:
                        fixes.setdefault(src, []).append(fx)
    return valid, err, nodes, fixes


def get_node_info(gs, nodes, timeout=2.0):
    my = MY_NODE_ID if MY_NODE_ID not in nodes else 100
    out = {}
    rx = Reassembler()
    for n in sorted(nodes):
        tid = int(time.time() * 8) & 0x1F
        arb = (20 << 25) | (1 << 24) | (1 << 23) | (1 << 15) | (n << 8) | my
        gs.send(GsUsbFrame(can_id=CAN_EFF_FLAG | arb, data=[0xC0 | tid]))
        got = None
        deadline = time.time() + timeout
        while time.time() < deadline and got is None:
            f = GsUsbFrame()
            if not gs.read(f, 50) or f.echo_id != RX_ECHO:
                continue
            a = f.can_id & CAN_EFF_MASK
            if not (a & (1 << 24)) or (a & (1 << 23)) or ((a >> 15) & 0xFF) != 1:
                continue
            if ((a >> 8) & 0x7F) != my or (a & 0xFF) != n:
                continue
            got = rx.feed(n, 1, 1, 0, bytes(f.data[:f.can_dlc]))
        out[n] = parse_get_node_info(got) if got else None
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sniff", type=float, default=10.0)
    ap.add_argument("--rates", type=int, nargs="*", default=[1000000, 500000, 250000])
    args = ap.parse_args()

    gs = open_adapter()
    loopback_selftest(gs)

    found = None
    for rate in args.rates:
        gs.stop()
        if not gs.set_bitrate(rate):
            print(f"[SCAN] {rate} unsupported")
            continue
        gs.start()
        v0, e0, _, _ = collect(gs, 2.0)          # settling window
        v1, e1, nodes, fixes = collect(gs, 5.0)  # measurement window
        st = get_state(gs)
        st_s = f"{STATES.get(st[0], st[0])} rxerr={st[1]} txerr={st[2]}" if isinstance(st, tuple) else str(st)
        print(f"[SCAN] {rate/1000:>6.0f} kbit/s: settling valid={v0} err={e0} | "
              f"measure valid={v1} err={e1} | controller: {st_s}")
        if v1 > 0 or v0 > 0:
            found = rate
            gs.stop()
            break
        gs.stop()

    if found is None:
        print("\n[RESULT] still NO traffic on the bus - wiring/ground/power is still not right.")
        print("[HINT] check with a multimeter: adapter GND <-> GPS GND, adapter CAN_H <-> GPS CAN_H,")
        print("       adapter CAN_L <-> GPS CAN_L (by signal name, not by connector pin number),")
        print("       and 4.7-5.2 V at the GPS connector while powered.")
        return 1

    print(f"\n[TRAFFIC] found at {found} bit/s -> running full decode ({args.sniff:.0f} s)")
    gs.set_bitrate(found)
    gs.start()
    valid, err, nodes, fixes = collect(gs, args.sniff)
    print(f"[DECODE] frames={valid} error_frames={err}")
    for src, st in sorted(nodes.items()):
        print(f"    NodeStatus node {src}: uptime={st['uptime_sec']}s health={st['health']} mode={st['mode']}")
    for src, lst in sorted(fixes.items()):
        last = lst[-1]
        print(f"    gnss.Fix   node {src}: {len(lst)} msgs, lat={last['lat_deg']:.7f} "
              f"lon={last['lon_deg']:.7f} msl={last['height_msl_m']:.1f}m")
    if nodes:
        print("[IDENTIFY] sending GetNodeInfo ...")
        info = get_node_info(gs, nodes)
        for n, i in info.items():
            print(f"    node {n}: {i if i else 'no response'}")
    else:
        info = {}
    gs.stop()

    ts = time.strftime("%Y%m%d_%H%M%S")
    out = WS / "reports" / f"can_quickcheck_{ts}.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(dict(bitrate=found, frames=valid, error_frames=err,
                                   node_status={str(k): v for k, v in nodes.items()},
                                   node_info={str(k): v for k, v in info.items()}),
                              indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"[REPORT] {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
