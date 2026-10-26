"""Send DroneCAN GetNodeInfo toward candidate node IDs (incl. 0) over SLCAN and watch.

The module we see transmits with source node id 0 (anonymous), which normally means
"waiting for dynamic node id allocation". This probe checks whether it answers a service
request anyway, and dumps everything seen on the bus during the probe.
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from slcan_can_test import Slcan, parse_slcan_frame

SVC_FLAG_V0 = 1 << 24
REQ_FLAG = 1 << 23


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--bitrate", type=int, default=1000000)
    ap.add_argument("--secs", type=float, default=12.0)
    ap.add_argument("--targets", type=int, nargs="*", default=[0])
    args = ap.parse_args()

    sl = Slcan(args.port)
    print(f"[SLCAN] {sl.cmd('V').decode('ascii', 'replace').strip()!r}")
    sl.set_bitrate(args.bitrate)
    print(f"[SLCAN] open: {sl.start()}  bitrate={args.bitrate}")

    my = 10
    t0 = time.time()
    sent = 0
    tx_times = []
    seen = {}
    while time.time() - t0 < args.secs:
        # send a GetNodeInfo request to each target every 2 seconds
        if all(time.time() - t > 2.0 for t in tx_times) or not tx_times:
            for target in args.targets:
                tid = sent & 0x1F
                arb = (20 << 25) | SVC_FLAG_V0 | REQ_FLAG | (1 << 15) | ((target & 0x7F) << 8) | my
                sl.send_ext(arb, [0xC0 | tid])
                print(f"  {time.time()-t0:6.2f}s TX GetNodeInfo -> node {target} (id={arb:08X})")
                tx_times.append(time.time())
                sent += 1
        sl.read_all()
        for line in sl.lines():
            p = parse_slcan_frame(line)
            if not p:
                print(f"  [line] {line!r}")
                continue
            arb, data = p
            key = (arb, bytes(data))
            seen[key] = seen.get(key, 0) + 1
            svc = bool(arb & SVC_FLAG_V0)
            print(f"  {time.time()-t0:6.2f}s RX {arb:08X} svc={svc} dtype={(arb>>8)&0xFFFF} "
                  f"src={arb&0xFF} data={bytes(data).hex(' ')}")
        time.sleep(0.02)

    print(f"\nsent {sent} requests; distinct frames seen: {len(seen)}")
    for (arb, data), n in sorted(seen.items(), key=lambda kv: -kv[1])[:10]:
        print(f"    {arb:08X} x{n} data={data.hex(' ')}")
    sl.close()


if __name__ == "__main__":
    main()
