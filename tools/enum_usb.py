"""Enumerate all USB devices visible to libusb and flag likely CAN adapters."""
import sys

import usb.core
import libusb_package

backend = libusb_package.get_libusb1_backend()
devices = usb.core.find(find_all=True, backend=backend)
print(f"{'VID:PID':>10}  {'CLASS':>6}  MANUFACTURER / PRODUCT / SERIAL")
for d in devices:
    vid, pid = d.idVendor, d.idProduct
    try:
        man = usb.util.get_string(d, d.iManufacturer) or ""
    except Exception as e:
        man = f"<{type(e).__name__}>"
    try:
        prod = usb.util.get_string(d, d.iProduct) or ""
    except Exception as e:
        prod = f"<{type(e).__name__}>"
    try:
        ser = usb.util.get_string(d, d.iSerialNumber) or ""
    except Exception as e:
        ser = f"<{type(e).__name__}>"
    cls = f"{d.bDeviceClass:#04x}"
    flag = ""
    if (vid, pid) == (0x1D50, 0x606F):
        flag = "  <-- gs_usb / candleLight CAN adapter"
    elif vid in (0x1D50, 0x1209):
        flag = "  <-- open-source VID, check product"
    print(f"{vid:04X}:{pid:04X}  {cls:>6}  {man} / {prod} / {ser}{flag}")
print("done")
