"""Check SLCAN status flags (F) before/after transmitting, to verify our TX is acknowledged.

SLCAN 'F' returns one hex byte of status flags:
  bit0 RX overflow, bit1 TX overflow, bit2 TX error warning, bit3 RX error warning,
  bit4 TX error passive, bit5 RX error passive, bit6 bus off
"""
import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else "COM10"
sp = serial.Serial(port, 115200, timeout=0.4, write_timeout=1.0)
time.sleep(0.2)
sp.reset_input_buffer()


def query(text, wait=0.3):
    sp.reset_input_buffer()
    sp.write((text + "\r").encode())
    time.sleep(wait)
    return sp.read(256)


print("V ->", query("V"))
print("S8 ->", query("S8"))
print("O  ->", query("O"))
print("F  ->", query("F"))
print("F  ->", query("F"))

FLAGS = {0: "RX_OVERFLOW", 1: "TX_OVERFLOW", 2: "TX_ERROR_WARNING", 3: "RX_ERROR_WARNING",
         4: "TX_ERROR_PASSIVE", 5: "RX_ERROR_PASSIVE", 6: "BUS_OFF"}

for i in range(20):
    # standard-format frame 't', extended 'T'; send a DroneCAN-ish extended frame
    sp.write(f"T0BD0000{i%10:X}2AABB\r".encode())
    time.sleep(0.01)
time.sleep(0.3)
r = query("F", wait=0.5)
print("F after 20 TX ->", r)
try:
    val = int(r.decode().strip(), 16)
    active = [n for b, n in FLAGS.items() if val & (1 << b)]
    print(f"   flags=0x{val:02X} -> {active if active else 'clean (no errors)'}")
except Exception as e:
    print("   could not parse:", e)

sp.write(b"C\r")
time.sleep(0.1)
sp.close()
