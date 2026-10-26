"""DroneCAN allocator v6 - prompt with 6 bytes, then GRANT with the 12 collected bytes.

Confirmed behaviour of the module (CUAV/CAN GNSS family):
  * periodic stage-1 anonymous request : flag=1, uid chunk = 47 00 07 00 0a 51
  * after the allocator echoes those 6 bytes, it sends stage-2 chunk = 34 33 34 33 30 37
  * a response with node_id == 0 is not a grant -> it keeps waiting
  * so: echo 6 bytes to advance it, then grant (node_id != 0) with the 12 bytes it sent

Grant variants are rotated in case its unique id is 12 or 16 bytes long.
"""
import argparse
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from slcan_can_test import Slcan, parse_slcan_frame
from can_comm_test import crc16_uavcan, Reassembler, decode_node_status, decode_fix

ALLOC_DTYPE = 1
ALLOCATOR_NODE_ID = 100
PRIO = 12

VARIANTS = [
    ("grant 12B uid, node_id=1", 1, 12),
    ("grant 16B uid (12+0000), node_id=1", 1, 16),
    ("grant 12B uid, node_id=125", 125, 12),
    ("grant 16B uid (12+0000), node_id=125", 125, 16),
]


def tx_message(sl, arb, payload, tid):
    payload = bytes(payload)
    if len(payload) <= 7:
        sl.send_ext(arb, payload + bytes([0xC0 | (tid & 0x1F)]))
        return 1
    body = struct.pack("<H", crc16_uavcan(payload)) + payload
    chunks = [body[i:i + 7] for i in range(0, len(body), 7)]
    n = len(chunks)
    for i, ch in enumerate(chunks):
        tail = ((0x80 if i == 0 else 0) | (0x40 if i == n - 1 else 0)
                | (0x20 if (i & 1) else 0) | (tid & 0x1F))
        sl.send_ext(sl_arb(arb), bytes(ch) + bytes([tail]))
        time.sleep(0.002)
    return n


def sl_arb(a):
    return a


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--secs", type=float, default=90.0)
    ap.add_argument("--variant-seconds", type=float, default=20.0)
    args = ap.parse_args()

    sl = Slcan(args.port)
    print(f"[SLCAN] {sl.cmd('V').decode('ascii', 'replace').strip()!r}")
    sl.set_bitrate(1000000)
    print(f"[SLCAN] open: {sl.start()}")

    collected = b""
    last_ts = 0.0
    tid = 0
    rx = Reassembler()
    nodes, fixes = {}, {}
    t0 = time.time()
    rep_arb = (PRIO << 25) | (ALLOC_DTYPE << 8) | ALLOCATOR_NODE_ID
    reqs = 0
    vi = -1
    last_grant = 0.0
    grants = 0

    def advance_or_grant(now):
        nonlocal tid, vi, last_grant, grants
        if len(collected) >= 12:
            idx = min(int((now - t0) // args.variant_seconds), len(VARIANTS) - 1)
            label, gid, ulen = VARIANTS[idx]
            if idx != vi:
                vi = idx
                print(f"\n=== t+{now-t0:5.1f}s variant: {label} ===")
            if now - last_grant >= 1.0:
                uid = collected[:ulen].ljust(ulen, b"\x00")
                payload = bytes([(gid << 1)]) + uid
                nf = tx_message(sl, rep_arb, payload, tid)
                tid = (tid + 1) & 0x1F
                grants += 1
                last_grant = now
                print(f"  t+{now-t0:5.1f}s >>> GRANT node_id={gid} uid={uid.hex(' ')} ({nf} frames)")
        elif len(collected) >= 6:
            payload = bytes([0x00]) + collected
            nf = tx_message(sl, rep_arb, payload, tid)
            tid = (tid + 1) & 0x1F
            if reqs % 5 == 1:
                print(f"  t+{now-t0:5.1f}s -> echo {len(collected)}B uid={collected.hex(' ')} "
                      f"({nf} frames) [waiting for next chunk]")

    while time.time() - t0 < args.secs:
        sl.read_all()
        for line in sl.lines():
            p = parse_slcan_frame(line)
            if not p:
                continue
            arb, data = p
            src = arb & 0xFF
            now = time.time()
            if src == 0 and len(data) >= 2 and not (arb & (1 << 24)):
                reqs += 1
                disc = (arb >> 8) & 0xFFFF
                chunk = bytes(data[1:-1])
                if now - last_ts > 5.0 and collected:
                    print(f"  t+{now-t0:5.1f}s (idle reset)")
                    collected = b""
                last_ts = now
                if chunk and chunk not in collected:
                    collected = (collected + chunk)[:16]
                    print(f"  t+{now-t0:5.1f}s NEW CHUNK disc={disc:04X} flag={data[0] & 1} "
                          f"chunk={chunk.hex(' ')} -> collected={collected.hex(' ')} ({len(collected)}B)")
                advance_or_grant(now)
                continue

            dtype = (arb >> 8) & 0xFFFF
            pl = rx.feed(src, dtype, 0, 0, data)
            if pl is not None:
                if dtype == 341:
                    st = decode_node_status(pl)
                    print(f"  t+{now-t0:5.1f}s *** NodeStatus node {src}: {st}")
                    nodes[src] = st
                elif dtype == 1060:
                    fx = decode_fix(pl)
                    if fx:
                        fixes.setdefault(src, []).append(fx)
                        print(f"  t+{now-t0:5.1f}s *** gnss.Fix node {src}: lat={fx['lat_deg']:.7f} "
                              f"lon={fx['lon_deg']:.7f} msl={fx['height_msl_m']:.1f}")
                elif dtype != 1:
                    print(f"  t+{now-t0:5.1f}s msg dtype={dtype} src={src} data={pl[:16].hex(' ')}")
        time.sleep(0.01)

    print(f"\nrequests={reqs} grants={grants} collected={collected.hex(' ')}")
    print(f"nodes publishing NodeStatus: {sorted(nodes)}")
    for s, lst in sorted(fixes.items()):
        print(f"  node {s}: {len(lst)} Fix messages")
    sl.close()


if __name__ == "__main__":
    main()
