"""DroneCAN/UAVCAN v0 test over a SLCAN (serial) CAN adapter - CANable 2.0.

Pipeline: open port -> query version -> set bitrate -> open channel -> passive sniff with
decode -> send GetNodeInfo to discovered nodes via slcan 'T' frames.

Usage:
  python slcan_can_test.py --port COM10 [--bitrate 1000000] [--scan] [--sniff 10]
"""
import argparse
import json
import sys
import time
from pathlib import Path

import serial

sys.path.insert(0, str(Path(__file__).parent))
from can_comm_test import (
    Reassembler, decode_node_status, decode_fix, parse_get_node_info,
    DT_NAMES, DT_NODE_STATUS, DT_GNSS_FIX, MY_NODE_ID,
)

WS = Path(r"D:\Lab\Flight-data-acquisition-unit")

# SLCAN bitrate codes
SLCAN_BITRATE = {10000: "S0", 20000: "S1", 50000: "S2", 100000: "S3", 125000: "S4",
                 250000: "S5", 500000: "S6", 800000: "S7", 1000000: "S8"}


class Slcan:
    def __init__(self, port, baud=115200):
        self.sp = serial.Serial(port, baud, timeout=0.05, write_timeout=1.0)
        time.sleep(0.2)
        self.sp.reset_input_buffer()
        self.buf = b""
        self.open = False

    def cmd(self, text, wait=0.15):
        self.sp.write((text + "\r").encode())
        time.sleep(wait)
        return self.read_all()

    def read_all(self):
        data = self.sp.read(4096)
        if data:
            self.buf += data
        return data

    def lines(self):
        """Split buffered stream into complete '\\r'-terminated lines."""
        out = []
        while b"\r" in self.buf:
            line, self.buf = self.buf.split(b"\r", 1)
            if line:
                out.append(line.decode("ascii", "replace"))
        return out

    def set_bitrate(self, rate):
        code = SLCAN_BITRATE.get(rate)
        if not code:
            return False
        r = self.cmd(code)
        return b"\a" not in r

    def start(self):
        r = self.cmd("O")
        self.open = b"\a" not in r
        return self.open

    def stop(self):
        try:
            self.cmd("C", wait=0.05)
        except Exception:
            pass
        self.open = False

    def send_ext(self, arb_id, data, retries=0):
        payload = bytes(data)
        frame = f"T{arb_id & 0x1FFFFFFF:08X}{len(payload):X}" + payload.hex().upper()
        self.sp.write((frame + "\r").encode())

    def close(self):
        self.stop()
        self.sp.close()


def parse_slcan_frame(line):
    """Return (arb_id, data) or None."""
    if not line or line[0] not in "tTrR":
        return None
    try:
        if line[0] in "tT":
            idlen = 3 if line[0] == "t" else 8
            arb = int(line[1:1 + idlen], 16)
            dlc = int(line[1 + idlen], 16)
            data = bytes.fromhex(line[2 + idlen:2 + idlen + dlc * 2])
            return arb, data
        return None
    except ValueError:
        return None


def collect(sl, seconds, rx, nodes, fixes, errs):
    valid = 0
    t_end = time.time() + seconds
    while time.time() < t_end:
        sl.read_all()
        for line in sl.lines():
            if line[0] not in "tT":
                errs.append(line)
                continue
            parsed = parse_slcan_frame(line)
            if not parsed:
                continue
            arb, data = parsed
            valid += 1
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
        time.sleep(0.02)
    return valid


def get_node_info(sl, nodes, timeout=2.0):
    my = MY_NODE_ID if MY_NODE_ID not in nodes else 100
    out = {}
    rx = Reassembler()
    for n in sorted(nodes):
        tid = int(time.time() * 8) & 0x1F
        arb = (20 << 25) | (1 << 24) | (1 << 23) | (1 << 15) | (n << 8) | my
        sl.send_ext(arb, [0xC0 | tid])
        got = None
        deadline = time.time() + timeout
        while time.time() < deadline and got is None:
            sl.read_all()
            for line in sl.lines():
                parsed = parse_slcan_frame(line)
                if not parsed:
                    continue
                a, data = parsed
                if not (a & (1 << 24)) or (a & (1 << 23)) or ((a >> 15) & 0xFF) != 1:
                    continue
                if ((a >> 8) & 0x7F) != my or (a & 0xFF) != n:
                    continue
                got = rx.feed(n, 1, 1, 0, data)
            time.sleep(0.02)
        out[n] = parse_get_node_info(got) if got else None
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--bitrate", type=int, default=1000000)
    ap.add_argument("--sniff", type=float, default=10.0)
    ap.add_argument("--scan", action="store_true", help="try several bitrates first")
    args = ap.parse_args()

    sl = Slcan(args.port, args.baud)
    ver = sl.cmd("V")
    print(f"[SLCAN] {args.port} firmware: {ver.decode('ascii', 'replace').strip()!r}")
    sl.cmd("N", wait=0.2)
    print(f"[SLCAN] serial query: {sl.lines()}")

    rates = [1000000, 500000, 250000] if args.scan else [args.bitrate]
    found = None
    for rate in rates:
        ok = sl.set_bitrate(rate)
        if not sl.start():
            print(f"[SLCAN] {rate}: could not open channel (is another program using {args.port}?)")
            continue
        rx, nodes, fixes, errs = Reassembler(), {}, {}, []
        v0 = collect(sl, 2.0, rx, nodes, fixes, errs)
        v1 = collect(sl, 5.0, rx, nodes, fixes, errs)
        print(f"[SCAN] {rate/1000:>6.0f} kbit/s: settling={v0} measure={v1} frames, "
              f"non-frame lines={errs[:4]}")
        sl.stop()
        if v0 or v1:
            found = rate
            break
    if found is None:
        print("\n[RESULT] no traffic at the scanned bitrates.")
        print("         (new adapter, same wiring -> if this also stays silent, the fault is")
        print("          on the GPS module / wiring / grounding side, not the USB-CAN adapter)")
        sl.close()
        return 1

    print(f"\n[TRAFFIC] {found} bit/s -> full decode")
    sl.set_bitrate(found)
    sl.start()
    rx, nodes, fixes, errs = Reassembler(), {}, {}, []
    valid = collect(sl, args.sniff, rx, nodes, fixes, errs)
    print(f"[DECODE] frames={valid}")
    for src, st in sorted(nodes.items()):
        print(f"    NodeStatus node {src}: uptime={st['uptime_sec']}s health={st['health']} "
              f"mode={st['mode']} vssc={st['vssc']}")
    for src, lst in sorted(fixes.items()):
        last = lst[-1]
        print(f"    gnss.Fix   node {src}: {len(lst)} msgs, lat={last['lat_deg']:.7f} "
              f"lon={last['lon_deg']:.7f} msl={last['height_msl_m']:.1f}m")
    info = get_node_info(sl, nodes) if nodes else {}
    for n, i in info.items():
        print(f"    GetNodeInfo node {n}: {i if i else 'no response'}")
    sl.close()

    ts = time.strftime("%Y%m%d_%H%M%S")
    out = WS / "reports" / f"slcan_test_{ts}.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(dict(port=args.port, bitrate=found, frames=valid,
                                   node_status={str(k): v for k, v in nodes.items()},
                                   node_info={str(k): v for k, v in info.items()}),
                              indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"[REPORT] {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
