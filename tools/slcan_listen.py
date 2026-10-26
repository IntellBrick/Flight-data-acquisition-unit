"""Raw SLCAN listener with UAVCAN v0/v1 field decoding, for identifying what the GPS sends.

Usage: python slcan_listen.py --port COM10 [--bitrate 1000000] [--secs 15] [--all]
"""
import argparse
import sys
import time
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from slcan_can_test import Slcan, parse_slcan_frame
from can_comm_test import (
    Reassembler, decode_node_status, decode_fix, DT_NAMES,
    DT_NODE_STATUS, DT_GNSS_FIX,
)

SVC_FLAG_V0 = 1 << 24
SVC_FLAG_V1 = 1 << 25


def describe(arb, data):
    dlc = len(data)
    svc = bool(arb & SVC_FLAG_V0)
    v1_like = (arb >> 26) & 0x7  # priority if v1
    dtype = (arb >> 8) & 0xFFFF
    src = arb & 0xFF
    prio_v0 = (arb >> 25) & 0xF
    if svc:
        req = bool(arb & (1 << 23))
        stype = (arb >> 15) & 0xFF
        dest = (arb >> 8) & 0x7F
        return (f"SERVICE type={stype} {'req' if req else 'rsp'} src={src} dest={dest} "
                f"prio_v0={prio_v0} data={data.hex(' ')}")
    name = DT_NAMES.get(dtype, f"dtype={dtype}")
    return (f"MSG {name} src={src} dtype={dtype} prio_v0={prio_v0} v1_prio={v1_like} "
            f"dlc={dlc} data={data.hex(' ')}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--bitrate", type=int, default=1000000)
    ap.add_argument("--secs", type=float, default=15.0)
    ap.add_argument("--all", action="store_true", help="print every frame")
    args = ap.parse_args()

    sl = Slcan(args.port)
    print(f"[SLCAN] firmware: {sl.cmd('V').decode('ascii', 'replace').strip()!r}")
    sl.set_bitrate(args.bitrate)
    print(f"[SLCAN] bitrate set to {args.bitrate}, opening channel: {sl.start()}")

    rx = Reassembler()
    ids = Counter()
    dtypes = Counter()
    n = 0
    t0 = time.time()
    while time.time() - t0 < args.secs:
        sl.read_all()
        for line in sl.lines():
            p = parse_slcan_frame(line)
            if not p:
                print(f"  [non-frame] {line!r}")
                continue
            arb, data = p
            n += 1
            ids[arb] += 1
            dtype = (arb >> 8) & 0xFFFF
            dtypes[dtype] += 1
            if args.all or n <= 40:
                print(f"  {time.time()-t0:6.2f}s  {arb:08X}  {describe(arb, data)}")
            if not (arb & SVC_FLAG_V0):
                pl = rx.feed(arb & 0xFF, dtype, 0, 0, data)
                if pl is not None:
                    if dtype == DT_NODE_STATUS:
                        print(f"        -> NodeStatus node {arb & 0xFF}: {decode_node_status(pl)}")
                    elif dtype == DT_GNSS_FIX:
                        print(f"        -> gnss.Fix node {arb & 0xFF}: {decode_fix(pl)}")
        time.sleep(0.02)

    print(f"\ntotal {n} frames in {args.secs:.0f}s ({n/args.secs:.1f} Hz overall)")
    print("top CAN IDs:")
    for k, v in ids.most_common(12):
        print(f"    {k:08X}  x{v:<6} ~{v/args.secs:5.1f} Hz   dtype={(k>>8)&0xFFFF} src={k&0xFF}")
    print("data types:")
    for k, v in dtypes.most_common(12):
        print(f"    dtype={k:<6} {DT_NAMES.get(k, '?'):<40} x{v}")
    print(f"crc failures during reassembly: {rx.crc_fail_count}")
    sl.close()


if __name__ == "__main__":
    main()
