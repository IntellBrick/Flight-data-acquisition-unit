from collections import defaultdict

log = r"D:\Lab\Flight-data-acquisition-unit\reports\gps_alloc_test_20261013_164441_frames.log"
frames = defaultdict(int)
starts = defaultdict(int)
payloads = defaultdict(list)
cur = {}
for line in open(log):
    parts = line.split()
    if len(parts) < 4:
        continue
    try:
        arb = int(parts[1], 16)
    except ValueError:
        continue
    if arb & 0x80 or (arb & 0x7F) == 0:
        continue
    dt = (arb >> 8) & 0xFFFF
    data = bytes.fromhex("".join(parts[3:]))
    if not data:
        continue
    frames[dt] += 1
    tail = data[-1]
    if tail & 0x80:
        starts[dt] += 1
        cur[dt] = None if (tail & 0x40) else 2  # CRC bytes already counted, minus this frame's tail
        if cur[dt] is not None:
            cur[dt] += len(data) - 1
    elif dt in cur and cur[dt] is not None:
        cur[dt] += len(data) - 1
    if (tail & 0x40) and dt in cur:
        if cur[dt] is not None:
            payloads[dt].append(cur[dt])
        cur[dt] = None

T = 180.0
print("%6s %7s %9s %8s %9s %16s" % ("dtid", "frames", "transfers", "rate_Hz", "frms/xfer", "payload B"))
for dt in sorted(frames):
    xfer = frames[dt] / max(starts[dt], 1)
    pl = payloads.get(dt) or [0]
    print("%6d %7d %9d %8.2f %9.1f %6d..%-6d" % (dt, frames[dt], starts[dt], starts[dt] / T, xfer, min(pl), max(pl)))
total = sum(frames.values())
print("total frames %d, %.1f frames/s" % (total, total / T))
