"""DroneCAN allocator v5 - grant using the 12 bytes the device actually sends.

Hypothesis: the module's unique ID is a 96-bit (12 byte) STM32 UID, delivered as two 6-byte
chunks; it adopts a node ID when it receives an Allocation message with node_id != 0 whose
unique_id fully matches what it sent (12 bytes).

The script accumulates chunks (permissively - repeats ignored) and then tries several grant
variants, watching for NodeStatus(341) / gnss.Fix(1060) to appear.
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

# grant variants: (label, node_id, uid_padding_to_16)
VARIANTS = [
    ("12B uid, node_id=1", 1, False),
    ("16B uid (12+0000), node_id=1", 1, True),
    ("12B uid, node_id=125", 125, False),
    ("16B uid (12+0000), node_id=125", 125, True),
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
    ap.add_argument("--secs", type=float, default=80.0)
    ap.add_argument("--variant-seconds", type=float, default=15.0)
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
    vi = -1
    last_grant = 0.0
    grants = 0

    def maybe_grant():
        nonlocal vi, last_grant, grants, tid
        if len(collected) < 12:
            return
        idx = min(int((time.time() - t0) // args.variant_seconds), len(VARIANTS) - 1)
        label, gid, pad = VARIANTS[idx]
        if time.time() - last_grant < 1.5:
            return
        if idx != vi:
            vi = idx
            print(f"\n=== t+{time.time()-t0:5.1f}s grant variant: {label} ===")
        uid = collected[:12]
        if pad:
            uid = uid + b"\x00" * 4
        payload = bytes([(gid << 1)]) + uid      # device packing: node_id in bits 1..7
        nf = tx_message(sl, rep_arb, payload, tid)
        tid = (tid + 1) & 0x1F
        grants += 1
        print(f"  t+{time.time()-t0:5.1f}s >>> GRANT node_id={gid} uid={uid.hex(' ')} ({nf} frames)")
        last_grant = time.time()

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
                chunk = bytes(data[1:-1])
                if now - last_ts > 5.0:
                    collected = b""
                last_ts = now
                if chunk and chunk not in collected:
                    if collected.endswith(chunk):
                        pass
                    else:
                        collected = (collected + chunk)[:16]
                        print(f"  t+{now-t0:5.1f}s chunk disc={disc:04X} flag={data[0] & 1} "
                              f"chunk={chunk.hex(' ')} -> collected={collected.hex(' ')} "
                              f"({len(collected)}B)")
                maybe_grant()
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

    print(f"\ngrants issued: {grants}, final collected={collected.hex(' ')}")
    print(f"nodes publishing NodeStatus: {sorted(nodes)}")
    for s, lst in sorted(fixes.items()):
        print(f"  node {s}: {len(lst)} Fix messages")
    sl.close()


if __name__ == "__main__":
    main()
