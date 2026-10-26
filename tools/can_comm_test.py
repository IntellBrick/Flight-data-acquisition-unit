"""CANdleLight (gs_usb) + DroneCAN/UAVCAN v0 communication test for a CUAV GPS module.

Flow:
  1. Open the candleLight USB-CAN adapter (VID 1D50:606F) via libusb.
  2. Set bitrate (default 1 Mbit/s, CUAV DroneCAN default) and start the controller.
  3. Passively sniff for N seconds: count frames, decode UAVCAN v0 messages
     (NodeStatus 341, gnss.Fix 1060, gnss.Fix2 1063, gnss.Auxiliary 1061),
     reassemble multi-frame transfers.
  4. Actively send uavcan.protocol.GetNodeInfo (service 1) to every discovered node
     and decode the device name / software / hardware version.
  5. Save a JSON + Markdown report under reports/.

Usage:
  python can_comm_test.py [--bitrate 1000000] [--sniff 10]
"""
import argparse
import json
import struct
import sys
import time
from pathlib import Path

import usb.core
import usb.util
import libusb_package

from gs_usb.gs_usb import GsUsb
from gs_usb.gs_usb_frame import GsUsbFrame
from gs_usb.constants import CAN_EFF_FLAG, CAN_ERR_FLAG, CAN_EFF_MASK

WS = Path(r"D:\Lab\Flight-data-acquisition-unit")

# --- UAVCAN v0 constants -----------------------------------------------------
# Message frame: [28:25] prio | [24] 0 | [23:8] data type | [7:0] source node
# Service frame: [28:25] prio | [24] 1 | [23] request | [22:15] svc type | [14:8] dest | [7:0] src
DT_NODE_STATUS = 341
DT_GNSS_FIX = 1060
DT_GNSS_AUX = 1061
DT_RTCM = 1062
DT_GNSS_FIX2 = 1063
DT_V1_HEARTBEAT = 32085

SVC_GET_NODE_INFO = 1

DT_NAMES = {
    DT_NODE_STATUS: "uavcan.protocol.NodeStatus",
    DT_GNSS_FIX: "uavcan.equipment.gnss.Fix",
    DT_GNSS_AUX: "uavcan.equipment.gnss.Auxiliary",
    DT_RTCM: "uavcan.equipment.gnss.RTCMStream",
    DT_GNSS_FIX2: "uavcan.equipment.gnss.Fix2",
    DT_V1_HEARTBEAT: "uavcan.node.Heartbeat (v1!)",
    1030: "uavcan.equipment.esc.RawCommand",
    1034: "uavcan.equipment.esc.Status",
    1035: "uavcan.equipment.esc.Telemetry? (check)",
    281: "uavcan.protocol.debug.KeyValue",
    16382: "uavcan.protocol.dynamic_node_id.Allocation",
}

HEALTH = {0: "OK", 1: "WARNING", 2: "ERROR", 3: "CRITICAL"}
MODE = {0: "OPERATIONAL", 1: "INITIALIZATION", 2: "MAINTENANCE", 3: "SOFTWARE_UPDATE", 7: "OFFLINE"}

CAN_GSUSB_RX_ECHO = 0xFFFFFFFF
MY_NODE_ID = 10
SVC_FLAG = 1 << 24
REQ_FLAG = 1 << 23
EFF = CAN_EFF_FLAG


class Transfer:
    __slots__ = ("payload", "toggle", "crc_ok", "nframes", "ts0")

    def __init__(self):
        self.payload = b""
        self.toggle = 0
        self.crc_ok = None
        self.nframes = 0
        self.ts0 = 0.0


def crc16_uavcan(data: bytes) -> int:
    """UAVCAN v0 transfer CRC: CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class Reassembler:
    """UAVCAN v0 multi-frame transfer reassembly keyed on (src, dtype, svc, req, tid)."""

    def __init__(self):
        self.bufs = {}
        self.completed = []          # (src, dtype, svc, req, payload, crc_ok)
        self.crc_fail_count = 0

    def feed(self, src, dtype, svc, req, payload):
        if not payload:
            return None
        tail = payload[-1]
        start = bool(tail & 0x80)
        end = bool(tail & 0x40)
        toggle = bool(tail & 0x20)
        tid = tail & 0x1F
        data = payload[:-1]
        key = (src, dtype, svc, req, tid)

        if start and end:            # single-frame transfer
            self.completed.append((src, dtype, svc, req, data, True))
            return data
        if start:
            self.bufs[key] = Transfer()
            self.bufs[key].payload = data
            self.bufs[key].ts0 = time.time()
            return None
        cur = self.bufs.get(key)
        if cur is None or toggle != cur.toggle:
            self.bufs.pop(key, None)
            return None
        cur.payload += data
        cur.nframes += 1
        cur.toggle = not cur.toggle
        if end:
            self.bufs.pop(key)
            full = cur.payload
            if len(full) >= 2:
                want = struct.unpack_from("<H", full)[0]
                body = full[2:]
                ok = crc16_uavcan(body) == want
                if not ok:
                    self.crc_fail_count += 1
                self.completed.append((src, dtype, svc, req, body, ok))
                return body
        return None


def open_adapter():
    backend = libusb_package.get_libusb1_backend()
    dev = usb.core.find(custom_match=GsUsb.is_gs_usb_device, backend=backend)
    if dev is None:
        raise SystemExit("[FAIL] no candleLight/gs_usb adapter found")
    gs = GsUsb(dev)
    try:
        cap = gs.device_capability
    except usb.core.USBError as e:
        raise SystemExit(f"[FAIL] adapter present but cannot be opened: {e}\n"
                         "Hint: close the candleLight GUI app / other program using the adapter.")
    return gs, cap


def decode_node_status(p):
    if len(p) < 7:
        return None
    uptime, = struct.unpack_from("<I", p)
    b4 = p[4]
    health = b4 & 0x3
    mode = (b4 >> 2) & 0x7
    sub = (b4 >> 5) & 0x7
    vssc, = struct.unpack_from("<H", p, 5)
    return dict(uptime_sec=uptime, health=HEALTH.get(health, health),
                mode=MODE.get(mode, mode), sub_mode=sub, vssc=vssc)


def decode_fix(p):
    if len(p) < 34:
        return None
    ts = int.from_bytes(p[0:7], "little")
    gnss_ts = int.from_bytes(p[7:14], "little")
    lat, lon = struct.unpack_from("<2d", p, 14)
    hmsl, hell = struct.unpack_from("<2f", p, 30)
    out = dict(timestamp_us=ts, gnss_timestamp_us=gnss_ts, lat_deg=lat, lon_deg=lon,
               height_msl_m=hmsl, height_ellipsoid_m=hell)
    if len(p) > 38:
        b = p[38]
        out["sva"] = b & 0x3F
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bitrate", type=int, default=1000000)
    ap.add_argument("--sniff", type=float, default=10.0, help="passive sniff seconds")
    ap.add_argument("--getinfo-timeout", type=float, default=2.0)
    args = ap.parse_args()

    gs, cap = open_adapter()
    print(f"[ADAPTER] {gs}  serial={gs.serial_number}")
    print(f"[ADAPTER] CAN clock: {cap.fclk_can} Hz, feature=0x{cap.feature:08x}")

    if not gs.set_bitrate(args.bitrate):
        raise SystemExit(f"[FAIL] unsupported bitrate {args.bitrate} for fclk {cap.fclk_can}")
    print(f"[ADAPTER] bitrate set to {args.bitrate} bit/s (sample point 87.5%)")
    gs.start()
    print(f"[ADAPTER] started")

    rx = Reassembler()
    id_counts = {}
    msg_counts = {}          # (src, dtype) -> count
    node_status = {}         # src -> dict
    fixes = {}               # src -> list of decoded Fix
    err_frames = 0
    total_rx = 0
    sample_frames = []
    t_end = time.time() + args.sniff
    frame = GsUsbFrame()

    print(f"\n[SNIFF] listening {args.sniff:.0f} s at {args.bitrate} bit/s ...")
    while time.time() < t_end:
        if not gs.read(frame, 100):
            continue
        if frame.echo_id != CAN_GSUSB_RX_ECHO:
            continue
        total_rx += 1
        arb = frame.can_id & CAN_EFF_MASK
        dlc = frame.can_dlc
        data = bytes(frame.data[:dlc])
        if frame.can_id & CAN_ERR_FLAG:
            err_frames += 1
            continue
        if len(sample_frames) < 12:
            sample_frames.append(f"    {arb:08X} [{dlc}] {data.hex(' ')}")
        id_counts[arb] = id_counts.get(arb, 0) + 1

        svc = bool(arb & SVC_FLAG)
        if not svc:
            dtype = (arb >> 8) & 0xFFFF
            src = arb & 0xFF
            msg_counts[(src, dtype)] = msg_counts.get((src, dtype), 0) + 1
            pl = rx.feed(src, dtype, 0, 0, data)
            if pl is None:
                continue
            if dtype == DT_NODE_STATUS:
                st = decode_node_status(pl)
                if st:
                    node_status[src] = st
            elif dtype == DT_GNSS_FIX:
                fx = decode_fix(pl)
                if fx:
                    fixes.setdefault(src, []).append(fx)
        # service frames from devices are only expected in response phase

    print(f"[SNIFF] received {total_rx} frames, {len(id_counts)} unique CAN IDs, {err_frames} error frames")
    for line in sample_frames:
        print(line)

    if total_rx == 0:
        print("\n[RESULT] NO frames received at this bitrate.")
        report = dict(adapter=str(gs), serial=gs.serial_number, bitrate=args.bitrate,
                      rx_frames=0, ok=False,
                      note="no traffic; check wiring/termination/bitrate/GPS power")
        save_report(report)
        gs.stop()
        return 1

    # ---- summarize messages -------------------------------------------------
    print("\n[MESSAGES] (source node, data type) -> count")
    dtype_summary = {}
    for (src, dt), c in sorted(msg_counts.items()):
        name = DT_NAMES.get(dt, f"dtype {dt}")
        rate = c / args.sniff
        dtype_summary[f"node{src}/{dt}"] = dict(name=name, count=c, hz=round(rate, 2))
        print(f"    node {src:>3}  {name:<42} {c:>6} frames  ~{rate:5.1f} Hz")

    if node_status:
        print("\n[NODE STATUS]")
        for src, st in sorted(node_status.items()):
            print(f"    node {src:>3}: uptime={st['uptime_sec']}s health={st['health']} mode={st['mode']} vssc={st['vssc']}")

    if fixes:
        print("\n[GNSS FIX]")
        for src, lst in sorted(fixes.items()):
            last = lst[-1]
            print(f"    node {src:>3}: {len(lst)} complete Fix transfers, last: "
                  f"lat={last['lat_deg']:.7f} lon={last['lon_deg']:.7f} "
                  f"msl={last['height_msl_m']:.1f}m gnss_ts_us={last['gnss_timestamp_us']} sva={last.get('sva')}")

    # ---- active GetNodeInfo -------------------------------------------------
    nodes = sorted({src for (src, _) in msg_counts})
    if not nodes:
        nodes = sorted({arb & 0xFF for arb in id_counts if not arb & SVC_FLAG})
    if MY_NODE_ID in nodes:
        MY = (MY_NODE_ID + 1) % 128 or 1
    else:
        MY = MY_NODE_ID

    print(f"\n[GET_NODE_INFO] querying nodes {nodes} (our node id = {MY})")
    node_info = {}
    for n in nodes:
        tid = int(time.time() * 8) & 0x1F
        arb = (20 << 25) | SVC_FLAG | REQ_FLAG | (SVC_GET_NODE_INFO << 15) | (n << 8) | MY
        tail = 0xC0 | tid
        tx = GsUsbFrame(can_id=EFF | arb, data=[tail])
        gs.send(tx)
        got = None
        deadline = time.time() + args.getinfo_timeout
        while time.time() < deadline and got is None:
            f2 = GsUsbFrame()
            if not gs.read(f2, 50):
                continue
            if f2.echo_id != CAN_GSUSB_RX_ECHO:
                continue
            a = f2.can_id & CAN_EFF_MASK
            if not (a & SVC_FLAG) or (a & REQ_FLAG):
                continue
            if ((a >> 15) & 0xFF) != SVC_GET_NODE_INFO:
                continue
            if ((a >> 8) & 0x7F) != MY:
                continue
            src = a & 0xFF
            if src != n:
                continue
            body = rx.feed(src, SVC_GET_NODE_INFO, 1, 0, bytes(f2.data[:f2.can_dlc]))
            if body is not None:
                got = body
        if got is None:
            print(f"    node {n:>3}: NO response to GetNodeInfo")
            node_info[str(n)] = None
            continue
        info = parse_get_node_info(got)
        node_info[str(n)] = info
        if info:
            print(f"    node {n:>3}: name={info['name']!r} sw={info['sw']} hw={info['hw']} "
                  f"uptime={info['uptime_sec']}s health={info['health']}")
        else:
            print(f"    node {n:>3}: response too short / unparsable ({len(got)} bytes): {got.hex(' ')}")

    ok = bool(node_status or fixes or any(v for v in node_info.values()))
    print(f"\n[RESULT] communication {'OK' if ok else 'NOT established'}: "
          f"{total_rx} frames, nodes={nodes}, crc_fails={rx.crc_fail_count}")

    report = dict(
        adapter=str(gs), serial=gs.serial_number, bitrate=args.bitrate,
        rx_frames=total_rx, unique_can_ids=len(id_counts), error_frames=err_frames,
        crc_failures=rx.crc_fail_count,
        can_id_counts={f"{k:08X}": v for k, v in sorted(id_counts.items())},
        messages=dtype_summary, node_status=node_status, gnss_fix={str(k): v[-3:] for k, v in fixes.items()},
        node_info=node_info, ok=ok,
        sample_frames=[s.strip() for s in sample_frames],
    )
    save_report(report)
    gs.stop()
    return 0 if ok else 2


def parse_get_node_info(p):
    if len(p) < 7 + 2 + 4 + 1 + 2 + 16:
        return None
    st = decode_node_status(p[:7])
    off = 7
    sw_major, sw_minor = p[off], p[off + 1]
    off += 2
    vcs_commit, = struct.unpack_from("<i", p, off)
    off += 4
    crc_defined = p[off]
    off += 1
    if crc_defined:
        off += 8
    hw_major, hw_minor = p[off], p[off + 1]
    off += 2
    uid = p[off:off + 16]
    off += 16
    name = ""
    if off < len(p):
        nlen = p[off]
        name = p[off + 1:off + 1 + nlen].decode("ascii", "replace")
    return dict(name=name, sw=f"{sw_major}.{sw_minor}", hw=f"{hw_major}.{hw_minor}",
                vcs_commit=f"{vcs_commit:08x}", unique_id=uid.hex(),
                uptime_sec=st["uptime_sec"] if st else None,
                health=st["health"] if st else None, mode=st["mode"] if st else None)


def save_report(report):
    ts = time.strftime("%Y%m%d_%H%M%S")
    outdir = WS / "reports"
    outdir.mkdir(exist_ok=True)
    jf = outdir / f"can_comm_test_{ts}.json"
    jf.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"[REPORT] saved {jf}")


if __name__ == "__main__":
    sys.exit(main())
