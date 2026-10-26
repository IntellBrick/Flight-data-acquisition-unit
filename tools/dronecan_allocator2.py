"""DroneCAN allocator v2: staged unique-ID collection + response recipe rotation.

Per uavcan/protocol/dynamic_node_id/1.Allocation.uavcan:
  * allocatee broadcasts anonymous requests: byte0 = uint7 node_id | bool first_part<<7, then uid chunk
  * allocator broadcasts non-anonymous Allocation (dtype field = 1) echoing the uid bytes it has,
    and grants by sending node_id != 0 together with the full 16-byte unique id
  * Trequest = 600..1000 ms, follow-up delay <= 400 ms

This script:
  1. inventories anonymous requesters (per discriminator)
  2. tries several reply recipes (including an empty-uid prompt that should make the allocatee
     reveal its first unique-id chunk, and grants carrying full/partial ids)
  3. reports immediately if NodeStatus(341) / gnss.Fix(1060) start appearing
"""
import argparse
import struct
import sys
import time
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from slcan_can_test import Slcan, parse_slcan_frame
from can_comm_test import crc16_uavcan, Reassembler, decode_node_status, decode_fix

ALLOC_DTYPE = 1
ALLOCATOR_NODE_ID = 100
PRIO = 12

# (name, grant_id, first_part, uid_mode)
RECIPES = [
    ("R1 grant1 fp0 echo", 1, 0, "echo"),
    ("R2 grant1 fp1 echo", 1, 1, "echo"),
    ("R3 grant1 fp0 echo+zeros16", 1, 0, "echo16"),
    ("R4 grant1 fp1 echo+zeros16", 1, 1, "echo16"),
    ("R5 prompt0 fp0 empty", 0, 0, "empty"),
    ("R6 grant1 fp0 empty", 1, 0, "empty"),
    ("R7 grant0 fp0 echo", 0, 0, "echo"),
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
        sl.send_ext(arb, bytes(ch) + bytes([tail]))
        time.sleep(0.002)
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--bitrate", type=int, default=1000000)
    ap.add_argument("--secs", type=float, default=70.0)
    ap.add_argument("--recipe-seconds", type=float, default=10.0)
    args = ap.parse_args()

    sl = Slcan(args.port)
    print(f"[SLCAN] {sl.cmd('V').decode('ascii', 'replace').strip()!r}")
    sl.set_bitrate(args.bitrate)
    print(f"[SLCAN] open: {sl.start()}")

    tid = 0
    rx = Reassembler()
    nodes, fixes = {}, {}
    inv = {}            # disc -> dict(uid, node_id_field, first_part, count)
    t0 = time.time()
    cur_recipe = None
    reqs = 0

    while time.time() - t0 < args.secs:
        ri = min(int((time.time() - t0) // args.recipe_seconds), len(RECIPES) - 1)
        if ri != cur_recipe:
            cur_recipe = ri
            print(f"\n=== t+{time.time()-t0:5.1f}s recipe: {RECIPES[ri][0]} ===")

        sl.read_all()
        for line in sl.lines():
            p = parse_slcan_frame(line)
            if not p:
                continue
            arb, data = p
            src = arb & 0xFF
            if src == 0 and len(data) >= 2 and not (arb & (1 << 24)):
                reqs += 1
                disc = (arb >> 8) & 0xFFFF
                uid = bytes(data[1:-1])
                entry = inv.setdefault(disc, {"uid": uid, "node_id": data[0] & 0x7F,
                                              "fp": bool(data[0] & 0x80), "count": 0})
                entry["count"] += 1
                if uid != entry["uid"]:
                    print(f"  t+{time.time()-t0:5.1f}s disc={disc:04X} uid CHANGED "
                          f"{entry['uid'].hex(' ')} -> {uid.hex(' ')}")
                    entry["uid"] = uid
                name, grant, fp, mode = RECIPES[ri]
                if mode == "echo":
                    rep_uid = entry["uid"]
                elif mode == "echo16":
                    rep_uid = entry["uid"] + b"\x00" * (16 - len(entry["uid"]))
                else:
                    rep_uid = b""
                payload = bytes([(grant & 0x7F) | (0x80 if fp else 0)]) + rep_uid
                rep_arb = (PRIO << 25) | (ALLOC_DTYPE << 8) | ALLOCATOR_NODE_ID
                tx_message(sl, rep_arb, payload, tid)
                tid = (tid + 1) & 0x1F
                continue

            dtype = (arb >> 8) & 0xFFFF
            pl = rx.feed(src, dtype, 0, 0, data)
            if pl is not None:
                if dtype == 341:
                    st = decode_node_status(pl)
                    print(f"  t+{time.time()-t0:5.1f}s *** NodeStatus node {src}: {st}")
                    nodes[src] = st
                elif dtype == 1060:
                    fx = decode_fix(pl)
                    if fx:
                        fixes.setdefault(src, []).append(fx)
                        print(f"  t+{time.time()-t0:5.1f}s *** gnss.Fix node {src}: lat={fx['lat_deg']:.7f} "
                              f"lon={fx['lon_deg']:.7f} msl={fx['height_msl_m']:.1f}m")
                elif dtype not in (1,):
                    print(f"  t+{time.time()-t0:5.1f}s msg dtype={dtype} src={src} len={len(pl)}")
        time.sleep(0.02)

    print(f"\nanonymous requests seen: {reqs}")
    print("inventory of anonymous requesters:")
    for disc, e in sorted(inv.items()):
        ascii_hint = "".join(chr(b) if 32 <= b < 127 else "." for b in e["uid"])
        print(f"    disc={disc:04X} x{e['count']:<5} node_id_field={e['node_id']} first_part={e['fp']} "
              f"uid={e['uid'].hex(' ')}  ascii={ascii_hint!r}")
    print(f"nodes publishing NodeStatus: {sorted(nodes)}")
    for s, lst in sorted(fixes.items()):
        print(f"  node {s}: {len(lst)} gnss.Fix messages")
    sl.close()


if __name__ == "__main__":
    main()
