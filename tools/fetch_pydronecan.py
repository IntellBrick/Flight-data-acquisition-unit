"""Fetch pydronecan into a fresh directory (pylibs2) and report its allocation server API."""
import io
import tarfile
import urllib.request
from pathlib import Path

WS = Path(r"D:\Lab\Flight-data-acquisition-unit")
TARGET = WS / "pylibs2"
TARGET.mkdir(exist_ok=True)

URL = "https://codeload.github.com/DroneCAN/pydronecan/tar.gz/refs/heads/master"
print(f"downloading {URL}")
data = urllib.request.urlopen(URL, timeout=180).read()
print(f"  {len(data)} bytes")

with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as tf:
    n = 0
    for m in tf.getmembers():
        parts = m.name.split("/")
        if len(parts) >= 2 and parts[1] == "dronecan":
            m.name = "/".join(parts[1:])
            tf.extract(m, TARGET)
            n += 1
    print(f"  extracted {n} files into {TARGET}")

pkg = TARGET / "dronecan"
print("package present:", pkg.exists())
for p in sorted(pkg.rglob("*.py")):
    try:
        t = p.read_text(encoding="utf-8", errors="replace")
    except Exception:
        continue
    score = t.lower().count("allocation")
    if score:
        print(f"  {p.relative_to(TARGET)}  allocation mentions: {score}")
        for line in t.splitlines():
            low = line.lower()
            if ("allocation" in low or "dynamic_node_id" in low) and (
                    "def " in line or "class " in line or "import" in line):
                print(f"       {line.strip()[:120]}")
