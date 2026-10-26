"""Identify which COM port is the CANable 2.0 (slcan firmware) and report its version/serial."""
import sys
import time

import serial
from serial.tools import list_ports

PORTS = [p.device for p in list_ports.comports()]


def describe_ports():
    print("serial ports:")
    for p in list_ports.comports():
        print(f"  {p.device:<8} {p.description}  hwid={p.hwid}")


def probe(port, baud=115200):
    try:
        with serial.Serial(port, baud, timeout=0.3, write_timeout=1.0) as sp:
            time.sleep(0.15)
            sp.reset_input_buffer()
            sp.write(b"V\r")          # firmware version
            time.sleep(0.15)
            sp.write(b"N\r")          # serial number
            time.sleep(0.15)
            sp.write(b"F\r")          # status flags
            time.sleep(0.25)
            data = sp.read(256)
            return data
    except Exception as e:
        return f"<{type(e).__name__}: {e}>"


def main():
    describe_ports()
    print("\nprobing (V/N/F queries):")
    for port in PORTS:
        r = probe(port)
        if isinstance(r, bytes):
            txt = r.decode("ascii", "replace").replace("\r", "\\r").replace("\n", "\\n")
            print(f"  {port:<8} -> {len(r):>3} bytes: {txt!r}")
        else:
            print(f"  {port:<8} -> {r}")


if __name__ == "__main__":
    sys.exit(main())
