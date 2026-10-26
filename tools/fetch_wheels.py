"""Download wheels for gs_usb / pyusb / libusb-package from PyPI and unzip into pylibs.

Avoids pip's internal temp-file handling which trips the sandbox.
"""
import json
import platform
import sys
import urllib.request
import zipfile
from pathlib import Path

WS = Path(r"D:\Lab\Flight-data-acquisition-unit")
WHEELS = WS / ".tmp" / "wheels"
PYLIBS = WS / "pylibs"

PKGS = ["gs_usb", "pyusb", "libusb-package", "importlib_resources", "pyserial",
        "python-can", "pydronecan", "packaging", "typing_extensions", "wrapt"]
# Selection predicates per package: (must_contain, must_not_contain)
def pick(pkg, urls):
    machine = platform.machine().lower()
    win_tag = "win_amd64" if "64" in machine else "win32"
    for u in urls:
        n = u.rsplit("/", 1)[-1].lower()
        if pkg == "libusb-package":
            if win_tag in n and "py3" in n and "none-any" not in n:
                return u
        else:
            if n.endswith(".whl") and ("py3-none-any" in n or "py2.py3-none-any" in n):
                return u
    return None


def main():
    WHEELS.mkdir(parents=True, exist_ok=True)
    PYLIBS.mkdir(parents=True, exist_ok=True)
    for pkg in PKGS:
        api = f"https://pypi.org/pypi/{pkg}/json"
        with urllib.request.urlopen(api, timeout=30) as r:
            data = json.loads(r.read().decode())
        urls = [f["url"] for f in data["urls"]]
        chosen = pick(pkg, urls)
        if chosen is None:
            print(f"[FAIL] no suitable wheel for {pkg}; candidates: {urls}", file=sys.stderr)
            return 2
        name = chosen.rsplit("/", 1)[-1]
        dest = WHEELS / name
        print(f"{pkg}: {name}")
        if not dest.exists():
            with urllib.request.urlopen(chosen, timeout=60) as r, open(dest, "wb") as f:
                f.write(r.read())
        with zipfile.ZipFile(dest) as z:
            z.extractall(PYLIBS)
        print(f"  extracted -> {PYLIBS}")
    print("ALL_OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
