"""DroneCAN allocator v4 - follows the reference algorithm (global unique-id accumulation).

Reference (legacy.uavcan.org, dynamic node ID allocation, allocator pseudocode):
    stage 1 : first_part_of_unique_id == true        (6 uid bytes)
    stage 2 : unique_id size == 6                    (6 uid bytes)
    stage 3 : unique_id size == 4                    (last 4 uid bytes)
    expected stage = 1 if nothing collected; 2 if >=6 collected; 3 if >=12 collected
    on match: append chunk; if 16 bytes -> allocate; else broadcast Allocation(node_id=0, uid=collected)

Observed device packing: first_part_of_unique_id in bit0, node_id in bits 1..7
    stage1 frame: 01 47 00 07 00 0A 51   (flag=1, uid=47 00 07 00 0A 51)
    stage2 frame: 00 34 33 34 33 30 37   (flag=0, uid="434307")
Replies mirror the same packing so the device accepts them.
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
    args = ap.parse_args()

    sl = Slcan(args.port)
    print(f"[SLCAN] {sl.cmd('V').decode('ascii', 'replace').strip()!r}")
    sl.set_bitrate(1000000)
    print(f"[SLCAN] open: {sl.start()}")

    current = b""            # accumulated unique id
    last_ts = 0.0
    FOLLOWUP_TIMEOUT = 5.0   # reset accumulation if silent for a while
    tid = 0
    rx = Reassembler()
    nodes, fixes = {}, {}
    t0 = time.time()
    rep_arb = (PRIO << 25) | (ALLOC_DTYPE << 8) | ALLOCATOR_NODE_ID
    grants = 0

    def expected_stage():
        if not current:
            return 1
        if len(current) >= 12:
            return 3
        if len(current) >= 6:
            return 2
        return 0

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
                disc = (arb >> 8) & 0xFFFF
                uid_chunk = bytes(data[1:-1])
                flag = data[0] & 0x01          # first_part_of_unique_id (device packing)
                node_id_field = data[0] >> 1
                if now - last_ts > FOLLOWUP_TIMEOUT:
                    if current:
                        print(f"  t+{now-t0:6.1f}s (follow-up timeout, clearing {current.hex(' ')})")
                    current = b""
                last_ts = now

                if flag:
                    stage = 1
                elif len(uid_chunk) == 6:
                    stage = 2
                elif len(uid_chunk) == 4:
                    stage = 3
                else:
                    stage = 0
                exp = expected_stage()
                print(f"  t+{now-t0:6.1f}s REQ disc={disc:04X} flag={flag} node_id={node_id_field} "
                      f"chunk={uid_chunk.hex(' ')} ({len(uid_chunk)}B) stage={stage} expected={exp} "
                      f"collected={current.hex(' ')}")

                if stage == 0 or stage != exp:
                    print("          -> stage mismatch, ignored (per reference algorithm)")
                    continue

                current += uid_chunk
                if len(current) >= 16:
                    grant_id = node_id_field if node_id_field else 125
                    payload = bytes([(grant_id << 1)]) + current[:16]
                    nf = tx_message(sl, rep_arb, payload, tid)
                    tid = (tid + 1) & 0x1F
                    grants += 1
                    print(f"          >>>>>> GRANT node_id={grant_id} uid={current[:16].hex(' ')} "
                          f"({nf} frames)")
                    current = b""
                else:
                    payload = bytes([0x00]) + current
                    nf = tx_message(sl, rep_arb, payload, tid)
                    tid = (tid + 1) & 0x1F
                    print(f"          -> follow-up response ({nf} frame(s)) uid={current.hex(' ')} "
                          f"({len(current)}B)")
                continue

            dtype = (arb >> 8) & 0xFFFF
            pl = rx.feed(src, dtype, 0, 0, data)
            if pl is not None:
                if dtype == 341:
                    st = decode_node_status(pl)
                    print(f"  t+{now-t0:6.1f}s *** NodeStatus node {src}: {st}")
                    nodes[src] = st
                elif dtype == 1060:
                    fx = decode_fix(pl)
                    if fx:
                        fixes.setdefault(src, []).append(fx)
                        print(f"  t+{now-t0:6.1f}s *** gnss.Fix node {src}: lat={fx['lat_deg']:.7f} "
                              f"lon={fx['lon_deg']:.7f} msl={fx['height_msl_m']:.1f}m")
                elif dtype != 1:
                    print(f"  t+{now-t0:6.1f}s msg dtype={dtype} src={src} data={pl[:16].hex(' ')}")
        time.sleep(0.01)

    print(f"\ngrants issued: {grants}")
    print(f"nodes publishing NodeStatus: {sorted(nodes)}")
    for s, lst in sorted(fixes.items()):
        print(f"  node {s}: {len(lst)} Fix messages")
    sl.close()


if __name__ == "__main__":
    main()
