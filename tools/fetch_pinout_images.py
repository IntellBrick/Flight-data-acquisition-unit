import sys
import urllib.request
from pathlib import Path

WS = Path(r"D:\Lab\Flight-data-acquisition-unit")
OUT = WS / ".tmp" / "imgs"
OUT.mkdir(parents=True, exist_ok=True)

URLS = {
    "neo4se_pinouts.jpg": "https://doc.cuav.net/gps/neo-series-gnss/assets/neo-4se/neo-4se-pinouts.jpg",
    "neo4se_connect.jpg": "https://doc.cuav.net/gps/neo-series-gnss/assets/neo-4se/neo-4se_connect.jpg",
    "candlelight_pins.png": "http://www.chinalctech.com/uploads/allimg/240907/1-240ZG35032292.png",
    "candlelight_front.png": "http://www.chinalctech.com/uploads/allimg/240907/1-240ZG32016431.png",
}

for name, url in URLS.items():
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
        with urllib.request.urlopen(req, timeout=30) as r:
            data = r.read()
        (OUT / name).write_bytes(data)
        print(f"OK   {name}: {len(data)} bytes")
    except Exception as e:
        print(f"FAIL {name}: {type(e).__name__} {e}")
