"""Dump raw SLCAN lines verbatim (hex + ascii) to verify the exact line format."""
import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else "COM10"
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0

sp = serial.Serial(port, 115200, timeout=0.1, write_timeout=1.0)
time.sleep(0.2)
sp.reset_input_buffer()
sp.write(b"V\r")
time.sleep(0.2)
sp.write(b"S8\r")      # 1 Mbit/s
time.sleep(0.2)
sp.write(b"O\r")       # open
time.sleep(0.2)
sp.reset_input_buffer()

buf = b""
lines = 0
t_end = time.time() + secs
while time.time() < t_end and lines < 25:
    data = sp.read(4096)
    if not data:
        continue
    buf += data
    while b"\r" in buf:
        line, buf = buf.split(b"\r", 1)
        if not line:
            continue
        lines += 1
        print(f"  raw[{lines:02d}] len={len(line):2d} hex={line.hex(' ')}  ascii={line!r}")
        if lines >= 25:
            break

sp.write(b"C\r")
sp.close()
print(f"dumped {lines} raw lines")
