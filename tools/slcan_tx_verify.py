"""TX verification, take 2 - drains the adapter FIFO first and works per allocatee.

Phase 0: drain 6 s (adapter buffers frames while the SLCAN channel is closed)
Phase A: baseline request rate per discriminator
Phase B: spam Allocation broadcasts every 20 ms -> per Rule C their request timers reset,
         so their own request rate must collapse if they receive our frames
Phase C: after spamming -> recovery
"""
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from slcan_can_test import Slcan, parse_slcan_frame

ALLOC_DTYPE = 1
ALLOCATOR_NODE_ID = 100
PRIO = 12


def run_phase(sl, label, secs, spam=False, spam_period=0.02):
    t_end = time.time() + secs
    per_disc = Counter()
    last_ts = {}
    gaps = defaultdict(list)
    spam_arb = (PRIO << 25) | (ALLOC_DTYPE << 8) | ALLOCATOR_NODE_ID
    next_spam = 0.0
    tid = 0
    n_spam = 0
    while time.time() < t_end:
        if spam and time.time() >= next_spam:
            sl.send_ext(spam_arb, bytes([0x00, 0xC0 | (tid & 0x1F)]))
            tid = (tid + 1) & 0x1F
            n_spam += 1
            next_spam = time.time() + spam_period
        sl.read_all()
        now = time.time()
        for line in sl.lines():
            p = parse_slcan_frame(line)
            if not p:
                continue
            arb, data = p
            if (arb & 0xFF) == 0 and len(data) >= 2 and not (arb & (1 << 24)):
                disc = (arb >> 8) & 0xFFFF
                per_disc[disc] += 1
                if disc in last_ts:
                    gaps[disc].append(now - last_ts[disc])
                last_ts[disc] = now
        time.sleep(0.005)
    rate = {d: c / secs for d, c in per_disc.items()}
    print(f"  {label:<22} requests: " + ", ".join(f"disc{d:04X}={c} ({rate[d]:.2f}/s)"
                                                   for d, c in sorted(per_disc.items()))
          + (f"   [sent {n_spam} Allocation frames]" if spam else ""))
    avg_gap = {d: (sum(g) / len(g) if g else 0) for d, g in gaps.items()}
    if avg_gap:
        print("      mean gap between that device's requests: "
              + ", ".join(f"disc{d:04X}={g*1000:.0f}ms" for d, g in sorted(avg_gap.items())))
    return per_disc


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM10"
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0

    sl = Slcan(port)
    print(f"[SLCAN] {sl.cmd('V').decode('ascii', 'replace').strip()!r}")
    sl.set_bitrate(1000000)
    print(f"[SLCAN] open: {sl.start()}")
    run_phase(sl, "drain (discard)", 6.0)
    a = run_phase(sl, "A baseline", secs)
    b = run_phase(sl, "B spam Allocation", secs, spam=True)
    c = run_phase(sl, "C after spam", secs)
    sl.close()

    print("\nverdict (per device):")
    for disc in sorted(set(a) | set(b) | set(c)):
        ra, rb, rc = a[disc] / secs, b[disc] / secs, c[disc] / secs
        verdict = ("SUPPRESSED -> device receives our frames"
                   if rb < min(ra, rc) * 0.5 else "unchanged -> device does not react to our frames")
        print(f"  disc{disc:04X}: A={ra:.2f}/s  B={rb:.2f}/s  C={rc:.2f}/s  -> {verdict}")


if __name__ == "__main__":
    main()
