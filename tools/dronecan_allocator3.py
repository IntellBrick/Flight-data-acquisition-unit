"""Proper staged DroneCAN allocator over SLCAN (allocator v3).

Protocol (uavcan/protocol/dynamic_node_id/1.Allocation.uavcan):
  request : byte0 = uint7 node_id | (bool first_part_of_unique_id << 7), then uid chunk
  reply   : NON-anonymous broadcast (CAN ID dtype field = 1, src = allocator node id)
            byte0 = granted node id | first_part=0, then the uid bytes the allocator holds
  grant   : reply with node_id != 0 AND the complete 16-byte unique id
  timing  : Trequest 600..1000 ms (first-stage), Tfollowup 0..400 ms (follow-up)

Strategy: keep echoing back every unique-id byte collected so far so the allocatee advances
its offset; print each request so we can see the chunks; grant once we hold 16 bytes
(or after a bounded number of stuck rounds, grant with what we have).
"""
import argparse
import struct
import sys
import time
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from slcan_can_test import Slcan, parse_slcan_frame
from can_comm_test import crc16_uavcan, Reassembler, decode_node_status, decode_fix

ALLOC_DTYPE = 1
ALLOCATOR_NODE_ID = 100
PRIO = 12


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
    ap.add_argument("--secs", type=float, default=60.0)
    ap.add_argument("--grant", type=int, default=1, help="node id to grant (0 = use preference)")
    args = ap.parse_args()

    sl = Slcan(args.port)
    print(f"[SLCAN] {sl.cmd('V').decode('ascii', 'replace').strip()!r}")
    sl.set_bitrate(1000000)
    print(f"[SLCAN] open: {sl.start()}")

    state = defaultdict(lambda: {"collected": b"", "same_chunk_rounds": 0, "last_chunk": None,
                                 "reqs": 0, "granted": False})
    tid = 0
    rx = Reassembler()
    nodes, fixes = {}, {}
    t0 = time.time()
    rep_arb = (PRIO << 25) | (ALLOC_DTYPE << 8) | ALLOCATOR_NODE_ID

    while time.time() - t0 < args.secs:
        sl.read_all()
        for line in sl.lines():
            p = parse_slcan_frame(line)
            if not p:
                continue
            arb, data = p
            src = arb & 0xFF
            if src == 0 and len(data) >= 2 and not (arb & (1 << 24)):
                disc = (arb >> 8) & 0xFFFF
                st = state[disc]
                chunk = bytes(data[1:-1])
                fp = bool(data[0] & 0x80)
                pref = data[0] & 0x7F
                st["reqs"] += 1
                ts = time.time() - t0

                if fp and chunk:
                    st["collected"] = chunk
                elif not st["collected"]:
                    st["collected"] = chunk
                elif chunk != st["collected"]:
                    # a new chunk -> append (dedupe if it is a prefix repeat)
                    if chunk.startswith(st["collected"][:len(chunk)]):
                        pass
                    else:
                        st["collected"] = (st["collected"] + chunk)[:16]

                if chunk == st["last_chunk"]:
                    st["same_chunk_rounds"] += 1
                else:
                    st["same_chunk_rounds"] = 0
                    st["last_chunk"] = chunk
                collected = st["collected"]

                print(f"  t+{ts:6.1f}s REQ #{st['reqs']:<3} disc={disc:04X} pref={pref} fp={fp} "
                      f"chunk={chunk.hex(' ')} | collected={collected.hex(' ')} ({len(collected)}B)")
                if st["reqs"] == 1 or st["reqs"] % 10 == 0 or chunk != st["last_chunk"]:
                    print(f"          same_chunk_rounds={st['same_chunk_rounds']}")

                full = len(collected) >= 16
                grant_id = args.grant if args.grant else (pref if pref else 125)
                stuck = st["same_chunk_rounds"] >= 4 and len(collected) >= 6
                if full or stuck:
                    payload = bytes([grant_id & 0x7F]) + collected[:16].ljust(16, b"\x00")
                    nf = tx_message(sl, rep_arb, payload, tid)
                    tid = (tid + 1) & 0x1F
                    st["granted"] = True
                    print(f"          >>> GRANT node_id={grant_id} uid={collected[:16].hex(' ')} "
                          f"({'full id' if full else 'forcing with partial id'} , {nf} frames)")
                else:
                    payload = bytes([0x00]) + collected
                    nf = tx_message(sl, rep_arb, payload, tid)
                    tid = (tid + 1) & 0x1F
                    if st["reqs"] <= 3 or st["reqs"] % 10 == 0:
                        print(f"          -> echo reply: node_id=0 uid={collected.hex(' ')} ({nf} frames)")
                continue

            dtype = (arb >> 8) & 0xFFFF
            pl = rx.feed(src, dtype, 0, 0, data)
            if pl is not None:
                if dtype == 341:
                    stt = decode_node_status(pl)
                    print(f"  t+{time.time()-t0:6.1f}s *** NodeStatus node {src}: {stt}")
                    nodes[src] = stt
                elif dtype == 1060:
                    fx = decode_fix(pl)
                    if fx:
                        fixes.setdefault(src, []).append(fx)
                        print(f"  t+{time.time()-t0:6.1f}s *** gnss.Fix node {src}: "
                              f"lat={fx['lat_deg']:.7f} lon={fx['lon_deg']:.7f} msl={fx['height_msl_m']:.1f}")
                elif dtype != 1:
                    print(f"  t+{time.time()-t0:6.1f}s msg dtype={dtype} src={src} data={pl[:16].hex(' ')}")
        time.sleep(0.01)

    print("\nsummary:")
    for disc, st in sorted(state.items()):
        print(f"  disc{disc:04X}: requests={st['reqs']} collected={st['collected'].hex(' ')} "
              f"granted={st['granted']}")
    print(f"  nodes publishing NodeStatus: {sorted(nodes)}")
    for s, lst in sorted(fixes.items()):
        print(f"  node {s}: {len(lst)} Fix messages")
    sl.close()


if __name__ == "__main__":
    main()
