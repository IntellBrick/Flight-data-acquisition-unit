"""DroneCAN dynamic node-ID allocator over SLCAN (fixed packing) with strategy rotation.

Payload packing (uavcan/protocol/dynamic_node_id/1.Allocation.uavcan):
    byte0 = uint7 node_id | (bool first_part_of_unique_id << 7)
    then  unique_id bytes (tail-optimized array, no length prefix)
Request from the module: 01 47 00 07 00 0A 51 | tail  -> node_id=1, first_part=0, uid=47 00 07 00 0A 51

Strategies tried in turn:
  A  standard packing : node_id=grant, first_part=0, uid=echo(6)      -> 7 bytes, single frame
  B  standard packing : uid=echo+10 zero bytes (16)                   -> 17 bytes, multi frame
  C  inverted packing : first_part into bit0, node_id << 1, uid=echo
  D  standard packing : node_id=42, uid=echo
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
STRATEGIES = ["A", "B", "C", "D"]


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
        sl.send_ext(arb, bytes(ch) + bytes([tail]))
        time.sleep(0.002)
    return n


def build_payload(strategy, uid, grant_id):
    if strategy == "A":
        return bytes([grant_id & 0x7F]) + uid
    if strategy == "B":
        return bytes([grant_id & 0x7F]) + uid + b"\x00" * 10
    if strategy == "C":
        return bytes([(grant_id & 0x7F) << 1]) + uid
    if strategy == "D":
        return bytes([42 & 0x7F]) + uid
    raise ValueError(strategy)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--bitrate", type=int, default=1000000)
    ap.add_argument("--secs", type=float, default=40.0)
    ap.add_argument("--per-strategy", type=float, default=10.0)
    args = ap.parse_args()

    sl = Slcan(args.port)
    print(f"[SLCAN] {sl.cmd('V').decode('ascii', 'replace').strip()!r}")
    sl.set_bitrate(args.bitrate)
    print(f"[SLCAN] open: {sl.start()}")
    print("payload sent will be shown as: node_id | first_part | uid")

    tid = 0
    rx = Reassembler()
    nodes, fixes = {}, {}
    t0 = time.time()
    idx = 0
    last_uid = None
    reqs = 0

    while time.time() - t0 < args.secs:
        strat = STRATEGIES[min(int((time.time() - t0) // args.per_strategy), len(STRATEGIES) - 1)]
        if STRATEGIES.index(strat) != idx:
            idx = STRATEGIES.index(strat)
            print(f"\n=== switching to strategy {strat} at t+{time.time()-t0:.1f}s ===")

        sl.read_all()
        for line in sl.lines():
            p = parse_slcan_frame(line)
            if not p:
                continue
            arb, data = p
            src = arb & 0xFF
            if src == 0 and len(data) >= 2 and not (arb & (1 << 24)):
                reqs += 1
                uid = bytes(data[1:-1])
                node_id_field = data[0] & 0x7F
                first_part = bool(data[0] & 0x80)
                if uid != last_uid:
                    print(f"  t+{time.time()-t0:6.1f}s request uid CHANGED -> {uid.hex(' ')} "
                          f"(node_id_field={node_id_field} first_part={first_part})")
                    last_uid = uid
                grant = 1
                payload = build_payload(strat, uid, grant)
                rep_arb = (PRIO << 25) | (ALLOC_DTYPE << 8) | ALLOCATOR_NODE_ID
                nf = tx_message(sl, rep_arb, payload, tid)
                tid = (tid + 1) & 0x1F
                if reqs <= 3 or reqs % 10 == 0:
                    print(f"  t+{time.time()-t0:6.1f}s [{strat}] reply #{reqs}: node_id={payload[0] & 0x7F} "
                          f"first_part={payload[0] >> 7} uid={payload[1:].hex(' ')} ({nf} frame(s))")
                continue

            dtype = (arb >> 8) & 0xFFFF
            pl = rx.feed(src, dtype, 0, 0, data)
            if pl is not None:
                if dtype == 341:
                    st = decode_node_status(pl)
                    print(f"  t+{time.time()-t0:6.1f}s *** NodeStatus node {src}: {st}")
                    nodes[src] = st
                elif dtype == 1060:
                    fx = decode_fix(pl)
                    if fx:
                        fixes.setdefault(src, []).append(fx)
                        print(f"  t+{time.time()-t0:6.1f}s *** gnss.Fix node {src}: "
                              f"lat={fx['lat_deg']:.7f} lon={fx['lon_deg']:.7f} msl={fx['height_msl_m']:.1f}m")
                else:
                    print(f"  t+{time.time()-t0:6.1f}s msg dtype={dtype} src={src} data={pl[:16].hex(' ')}")
        time.sleep(0.02)

    print(f"\nrequests received: {reqs}")
    print(f"nodes publishing NodeStatus: {sorted(nodes)}")
    for s, lst in sorted(fixes.items()):
        print(f"  node {s}: {len(lst)} gnss.Fix messages")
    sl.close()


if __name__ == "__main__":
    main()
