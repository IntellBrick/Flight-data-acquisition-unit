#!/usr/bin/env python3
"""CUAV SKYE2 DroneCAN airspeed sensor communication test over CANable 2.0 (slcan).

Target: CUAV SKYE2 airspeed sensor (M4C firmware family, DroneCAN, heated pitot)
on a Makerbase CANable 2.0 adapter in SLCAN mode (default port COM10, 1 Mbit/s).
Like the CUAV NEO 4SE GPS (see reports/gps_allocation_success_20261013.md), the
module boots with CAN_NODE_ID=0 and waits for a UAVCAN v0 dynamic node ID
allocator before it publishes anything. This script:

  1. Acts as a spec-compliant UAVCAN v0 dynamic node ID allocator
     (reuses the hardware-validated allocator from tools/gps_alloc_test.py).
  2. Verifies the node comes online: uavcan.protocol.NodeStatus (dtid 341).
  3. Queries uavcan.protocol.GetNodeInfo (service 1) to identify the device.
  4. Queries uavcan.protocol.GetDataTypeInfo (service 2) for the air data type
     IDs so the device itself reports which types it publishes and their exact
     64-bit data type signatures (best effort - some firmwares don't implement it).
  5. Decodes the air data messages into engineering values:
       uavcan.equipment.air_data.RawAirData (dtid 1027)
         flags (incl. pitot heater state), static & differential pressure (Pa),
         sensor/air/pitot temperatures (K), covariance
       uavcan.equipment.air_data.StaticPressure (dtid 1028)
       uavcan.equipment.air_data.StaticTemperature (dtid 1029)
     and derives indicated airspeed IAS = sqrt(2*dP/rho) and pressure altitude.
  6. Records per-dtid message rates, CRC failures, unknown-dtid raw payloads.
  7. Writes JSON + Markdown reports and a raw frame log under reports/.

Wire format notes (all verified against the UAVCAN v0 spec and pydronecan):
  * Data type IDs follow the DroneCAN/ArduPilot DSDL numbering
    (DroneCAN/DSDL repo, imported 2021-10-20, pinned by ArduPilot):
      1027 RawAirData, 1028 StaticPressure, 1029 StaticTemperature.
    (The pre-2021 legacy numbering was 1024/1021/1022; frames at those IDs are
     logged as unknown dtids with raw payloads so nothing is silently missed.)
  * 64-bit data type signatures below were computed with the spec's CRC-64/WE
    signature algorithm over the normalized DSDL definitions; the algorithm was
    cross-validated byte-for-byte against the hardware-proven signatures of
    NodeStatus (0x0F0868D0C1A7C6F1), Allocation (0x0B2A812620A11D40) and
    GetNodeInfo (0xEE468A8121C46A9E) from the 2026-10-13 GPS session.
  * RawAirData payload is byte-aligned: u8 flags | f32 static_pressure |
    f32 differential_pressure | f16 x4 temperatures | f16[<=16] covariance
    (tail-optimized: no length prefix, count inferred from transfer length).

Exit codes: 0 = PASS (node online + air data decoded),
            3 = partial (node online, no air data messages),
            1 = FAIL (node never came online),  4 = self-test failed.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from slcan_can_test import Slcan, parse_slcan_frame  # SLCAN serial link only
from gps_alloc_test import (          # hardware-validated UAVCAN v0 machinery
    Allocator, Reassembler, BitStream,
    decode_node_status, parse_get_node_info,
    SIG_ALLOCATION, SIG_GET_NODE_INFO, DT_NODE_STATUS, PRIO,
)

WS = Path(__file__).resolve().parent.parent
REPORTS = WS / "reports"

# ---------------- DroneCAN (ArduPilot DSDL) air data types ----------------
DT_RAW_AIR_DATA = 1027        # uavcan.equipment.air_data.RawAirData
DT_STATIC_PRESSURE = 1028     # uavcan.equipment.air_data.StaticPressure
DT_STATIC_TEMPERATURE = 1029  # uavcan.equipment.air_data.StaticTemperature
DT_HYGROMETER = 1032          # dronecan.sensors.hygrometer.Hygrometer
SVC_GET_NODE_INFO = 1
SVC_GET_DATA_TYPE_INFO = 2    # uavcan.protocol.GetDataTypeInfo

# 64-bit data type signatures (CRC-64/WE over normalized DSDL; see module doc)
SIG_RAW_AIR_DATA = 0xC77DF38BA122F5DA
SIG_STATIC_PRESSURE = 0xCDC7C43412BDC89A
SIG_STATIC_TEMPERATURE = 0x49272A6477D96271
SIG_GET_DATA_TYPE_INFO = 0x1B283338A7BED2D8

DTID_NAMES = {
    DT_NODE_STATUS: "uavcan.protocol.NodeStatus",
    DT_RAW_AIR_DATA: "uavcan.equipment.air_data.RawAirData",
    DT_STATIC_PRESSURE: "uavcan.equipment.air_data.StaticPressure",
    DT_STATIC_TEMPERATURE: "uavcan.equipment.air_data.StaticTemperature",
    DT_HYGROMETER: "dronecan.sensors.hygrometer.Hygrometer",
}

HEATER_FLAGS = {0x01: "HEATER_AVAILABLE", 0x02: "HEATER_WORKING",
                0x04: "HEATER_OVERCURRENT", 0x08: "HEATER_OPENCIRCUIT"}
RHO0 = 1.225  # kg/m^3, ISA sea level density for IAS


# ---------------- air data decoders ----------------
def _f16(v):
    """float16 -> float, mapping +/-inf and huge values to NaN-ish markers."""
    if v != v or math.isinf(v):
        return None
    return v


def decode_raw_air_data(p):
    """uavcan.equipment.air_data.RawAirData (DroneCAN DSDL, ex-0.2 layout)."""
    if len(p) < 17:
        return None
    bs = BitStream(p)
    out = dict(msg="RawAirData")
    out["flags"] = bs.read(8)
    out["static_pressure"] = bs.read_float(32)
    out["differential_pressure"] = bs.read_float(32)
    out["static_pressure_sensor_temperature"] = bs.read_float(16)
    out["differential_pressure_sensor_temperature"] = bs.read_float(16)
    out["static_air_temperature"] = bs.read_float(16)
    out["pitot_temperature"] = bs.read_float(16)
    cov = []
    while len(bs.s) >= 16:          # tail-optimized dynamic array
        cov.append(round(bs.read_float(16), 4))
    out["covariance"] = cov
    out["heater"] = [n for m, n in HEATER_FLAGS.items() if out["flags"] & m]
    dp = out["differential_pressure"]
    out["ias_mps"] = math.sqrt(2.0 * dp / RHO0) if dp == dp and dp > 0 else None
    sp = out["static_pressure"]
    out["pressure_alt_m"] = (44330.0 * (1.0 - (sp / 101325.0) ** 0.1903)
                             if sp == sp and sp > 0 else None)
    return out


def decode_static_pressure(p):
    """uavcan.equipment.air_data.StaticPressure (f32 Pa + f16 variance)."""
    if len(p) < 6:
        return None
    bs = BitStream(p)
    sp = bs.read_float(32)
    var = bs.read_float(16)
    return dict(msg="StaticPressure", static_pressure=sp, variance=var,
                pressure_alt_m=(44330.0 * (1.0 - (sp / 101325.0) ** 0.1903)
                                if sp == sp and sp > 0 else None))


def decode_static_temperature(p):
    """uavcan.equipment.air_data.StaticTemperature (f16 K + f16 variance)."""
    if len(p) < 4:
        return None
    bs = BitStream(p)
    return dict(msg="StaticTemperature", static_temperature=bs.read_float(16),
                variance=bs.read_float(16))


def decode_hygrometer(p):
    """dronecan.sensors.hygrometer.Hygrometer: f16 temperature (DSDL: degC),
    f16 humidity (%), u8 id. NOTE: the SKYE2 (sw 1.3) emits the temperature in
    KELVIN despite the DSDL saying degC (313.75 observed = 40.6 C), so both
    readings are returned."""
    if len(p) < 5:
        return None
    bs = BitStream(p)
    t = bs.read_float(16)
    rh = bs.read_float(16)
    sid = bs.read(8)
    return dict(msg="Hygrometer", temperature=t,
                temperature_c_if_kelvin=(t - 273.15 if t == t and t > 200 else None),
                humidity=rh, sensor_id=sid)


def decode_get_data_type_info(p):
    """uavcan.protocol.GetDataTypeInfo response."""
    if len(p) < 13:
        return None
    bs = BitStream(p)
    out = dict(signature="%016x" % bs.read(64), id=bs.read(16), kind=bs.read(8),
               flags=bs.read(8))
    out["flag_names"] = [n for m, n in ((1, "KNOWN"), (2, "SUBSCRIBED"),
                                        (4, "PUBLISHING"), (8, "SERVICE"))
                         if out["flags"] & m]
    out["name"] = bs.rest_bytes().decode("ascii", "replace").rstrip("\x00")
    return out


def fmt_rad(rd, src):
    def tc(v):
        """float16 K value -> display; 65504.0 is the saturated-NaN 'unfilled' marker."""
        if v is None or v != v or v >= 65500:
            return "unfilled"
        s = "%.1fC" % (v - 273.15)
        if 0 < v < 200:      # implausible as K, plausible as degC (SKYE2 sw1.3 pitot field)
            s += " (=%.1fC if field is degC)" % v
        return s
    return ("node %d RawAirData: dP=%+.1fPa IAS=%s static=%s alt=%s T_air=%s T_pitot=%s "
            "heater=[%s] cov=%d"
            % (src, rd["differential_pressure"],
               "%.2fm/s" % rd["ias_mps"] if rd["ias_mps"] is not None else "n/a",
               "%.1fPa" % rd["static_pressure"]
               if rd["static_pressure"] == rd["static_pressure"] else "unfilled(NaN)",
               "%.0fm" % rd["pressure_alt_m"] if rd["pressure_alt_m"] is not None else "n/a",
               tc(rd["static_air_temperature"]), tc(rd["pitot_temperature"]),
               ",".join(rd["heater"]) or "none", len(rd["covariance"])))


# ---------------- main test ----------------
def run(args):
    my_id = args.node_id
    sl = Slcan(args.port)
    print("[SLCAN] %s firmware: %r" % (args.port, sl.cmd("V").decode("ascii", "replace").strip()))
    sl.set_bitrate(args.bitrate)
    if not sl.start():
        print("[FAIL] could not open CAN channel on %s (another program using it?)" % args.port)
        sl.close()
        return 1
    print("[SLCAN] channel open at %d bit/s" % args.bitrate)

    ts_tag = time.strftime("%Y%m%d_%H%M%S")
    REPORTS.mkdir(exist_ok=True)
    rawlog_path = REPORTS / ("skye2_test_%s_frames.log" % ts_tag)
    rawlog = rawlog_path.open("w", encoding="ascii")

    rx = Reassembler()                       # with-CRC reassembly
    rx_nc = Reassembler()                    # no-CRC fallback (version mismatch detection)
    alloc = Allocator(sl, my_id, print)
    stats = {"frames": 0, "svc_frames": 0, "nonframe_lines": 0,
             "msg_counts": {}, "msg_first": {}, "msg_last": {},
             "xfer_counts": {}}          # completed transfers per dtid (message rate)
    node_status = {}
    first_nodestatus = [None]
    node_info = {}
    air_data = []                            # decoded RawAirData payloads
    air_sources = []
    sp_msgs, st_msgs = [], []
    hygro_msgs = []
    dti = {}                                 # dtid -> GetDataTypeInfo response
    unknown_payloads = {}                    # dtid -> first raw payload hex
    rad_crc_fail = [0]
    getinfo = {"pending": None, "sent_ts": 0.0, "tries": 0, "done": False}
    dti_queue = []                           # filled once node is known
    dti_state = {"cur": None, "sent_ts": 0.0, "tries": 0}
    _ltp = [0.0]

    def note_msg(dtype):
        stats["msg_counts"][dtype] = stats["msg_counts"].get(dtype, 0) + 1
        now = time.time()
        stats["msg_first"].setdefault(dtype, now)
        stats["msg_last"][dtype] = now

    def handle_air_data(src, dtype, pl):
        if dtype == DT_RAW_AIR_DATA:
            rd = decode_raw_air_data(pl)
            if rd:
                air_data.append(rd)
                air_sources.append(src)
                print("[t+%5.2f] AIR %s" % (rel, fmt_rad(rd, src)))
        elif dtype == DT_STATIC_PRESSURE:
            m = decode_static_pressure(pl)
            if m:
                sp_msgs.append(m)
                print("[t+%5.2f] AIR node %d StaticPressure: %.1f Pa (alt %.0f m)"
                      % (rel, src, m["static_pressure"], m["pressure_alt_m"] or 0))
        elif dtype == DT_STATIC_TEMPERATURE:
            m = decode_static_temperature(pl)
            if m:
                st_msgs.append(m)
                t = m["static_temperature"]
                print("[t+%5.2f] AIR node %d StaticTemperature: %.2f K (%.1f C)"
                      % (rel, src, t, t - 273.15))
        elif dtype == DT_HYGROMETER:
            m = decode_hygrometer(pl)
            if m:
                hygro_msgs.append(m)
                tc = m["temperature_c_if_kelvin"]
                print("[t+%5.2f] AIR node %d Hygrometer: T=%s RH=%.1f%% id=%d"
                      % (rel, src,
                         ("%.2f (=%.1fC if K)" % (m["temperature"], tc)) if tc is not None
                         else "%.2f" % m["temperature"],
                         m["humidity"], m["sensor_id"]))

    t0 = time.time()
    alloc_deadline = t0 + args.alloc_timeout
    monitor_end = None

    while True:
        now = time.time()
        rel = now - t0
        if monitor_end is None and (alloc.granted_node is not None or first_nodestatus[0] is not None):
            monitor_end = now + args.monitor
            print("[t+%5.2f] allocation phase finished -> monitoring %ds for air data"
                  % (rel, args.monitor))
            dti_queue.extend(args.dti_ids)
        if monitor_end is not None and now >= monitor_end:
            break
        if monitor_end is None and now >= alloc_deadline:
            print("[t+%5.2f] allocation timeout reached without the node coming online" % rel)
            break

        target = alloc.granted_node if alloc.granted_node is not None else (
            min(node_status) if node_status else None)

        # GetNodeInfo once the node is alive (retry once)
        if (target is not None and not getinfo["done"] and args.getinfo
                and getinfo["tries"] < 2
                and (getinfo["pending"] != target or now - getinfo["sent_ts"] > 2.0)):
            getinfo["pending"] = target
            getinfo["sent_ts"] = now
            getinfo["tries"] += 1
            arb = ((PRIO << 24) | (SVC_GET_NODE_INFO << 16) | (1 << 15)
                   | (target << 8) | (1 << 7) | my_id)
            sl.send_ext(arb, [0xC0 | (getinfo["tries"] & 0x1F)])
            print("[t+%5.2f] -> GetNodeInfo request to node %d" % (rel, target))

        # GetDataTypeInfo queries for the candidate dtids (2 tries each)
        if target is not None and dti_queue:
            if dti_state["cur"] is None:
                dti_state["cur"] = dti_queue[0]
                dti_state["tries"] = 0
            cur = dti_state["cur"]
            if dti_state["tries"] < 2 and (now - dti_state["sent_ts"] > 1.2
                                           or dti_state["tries"] == 0):
                dti_state["sent_ts"] = now
                dti_state["tries"] += 1
                # request: u16 id (LE) | u8 kind(MESSAGE=1) | u8[<=80] name (empty, TAO)
                req = bytes([cur & 0xFF, (cur >> 8) & 0xFF, 1,
                             0xC0 | (dti_state["tries"] & 0x1F)])
                arb = ((PRIO << 24) | (SVC_GET_DATA_TYPE_INFO << 16) | (1 << 15)
                       | (target << 8) | (1 << 7) | my_id)
                sl.send_ext(arb, req)
                print("[t+%5.2f] -> GetDataTypeInfo request to node %d for dtid %d"
                      % (rel, target, cur))
            elif dti_state["tries"] >= 2:
                dti_queue.pop(0)
                dti_state["cur"] = None

        sl.read_all()
        for line in sl.lines():
            p = parse_slcan_frame(line)
            if not p:
                stats["nonframe_lines"] += 1
                continue
            arb, data = p
            stats["frames"] += 1
            rel = time.time() - t0
            rawlog.write("%8.3f  %08X  %02X  %s\n"
                         % (rel, arb, len(data), data.hex(" ").upper()))

            if arb & 0x80:                                   # service frame
                stats["svc_frames"] += 1
                stype = (arb >> 16) & 0xFF
                req = (arb >> 15) & 1
                dest = (arb >> 8) & 0x7F
                src = arb & 0x7F
                if stype == SVC_GET_NODE_INFO and not req and dest == my_id:
                    pl = rx.feed(src, SVC_GET_NODE_INFO, 1, 0, data, SIG_GET_NODE_INFO)
                    if pl is not None:
                        info = parse_get_node_info(pl)
                        if info:
                            node_info = info
                            getinfo["done"] = True
                            print("[t+%5.2f] GetNodeInfo node %d: name=%r sw=%s hw=%s uid=%s"
                                  % (rel, src, info["name"], info["sw"], info["hw"],
                                     info["unique_id"]))
                elif stype == SVC_GET_DATA_TYPE_INFO and not req and dest == my_id:
                    pl = rx.feed(src, SVC_GET_DATA_TYPE_INFO, 1, 0, data,
                                 SIG_GET_DATA_TYPE_INFO)
                    if pl is not None:
                        r = decode_get_data_type_info(pl)
                        if r and r["id"] not in dti:
                            dti[r["id"]] = r
                            print("[t+%5.2f] GetDataTypeInfo: dtid %d name=%r sig=0x%s flags=[%s]"
                                  % (rel, r["id"], r["name"], r["signature"],
                                     ",".join(r["flag_names"]) or "none"))
                            if dti_state["cur"] == r["id"]:
                                if dti_queue and dti_queue[0] == r["id"]:
                                    dti_queue.pop(0)
                                dti_state["cur"] = None
                continue

            src = arb & 0x7F
            if src == 0:                                     # anonymous message
                if (arb >> 8) & 3 == 1:                      # Allocation (dtid low 2 bits)
                    disc = (arb >> 10) & 0x3FFF
                    alloc.handle_request(rel, data, disc)
                continue

            dtype = (arb >> 8) & 0xFFFF
            note_msg(dtype)
            if dtype not in DTID_NAMES and dtype not in unknown_payloads:
                unknown_payloads[dtype] = data[:-1].hex(" ")
                print("[t+%5.2f] NOTE unknown dtid %d from node %d: %s"
                      % (rel, dtype, src, data.hex(" ")))

            def note_xfer(dtype, pl):
                if pl is not None:
                    stats["xfer_counts"][dtype] = stats["xfer_counts"].get(dtype, 0) + 1

            if dtype == DT_NODE_STATUS:
                pl = rx.feed(src, dtype, 0, 0, data, None)
                note_xfer(dtype, pl)
                if pl is not None:
                    st = decode_node_status(pl)
                    if st:
                        node_status[src] = st
                        if first_nodestatus[0] is None:
                            first_nodestatus[0] = rel
                            print("[t+%5.2f] *** NODE ONLINE: NodeStatus from node %d "
                                  "(uptime=%ds health=%s mode=%s)"
                                  % (rel, src, st["uptime_sec"], st["health"], st["mode"]))
                        elif rel - _ltp[0] >= 5.0:
                            _ltp[0] = rel
                            print("[t+%5.2f] NodeStatus node %d: uptime=%ds health=%s mode=%s vssc=%d"
                                  % (rel, src, st["uptime_sec"], st["health"], st["mode"], st["vssc"]))
            elif dtype in (DT_RAW_AIR_DATA, DT_STATIC_PRESSURE, DT_STATIC_TEMPERATURE,
                          DT_HYGROMETER):
                sig = {DT_RAW_AIR_DATA: SIG_RAW_AIR_DATA,
                       DT_STATIC_PRESSURE: SIG_STATIC_PRESSURE,
                       DT_STATIC_TEMPERATURE: SIG_STATIC_TEMPERATURE,
                       DT_HYGROMETER: None}[dtype]
                pl = rx.feed(src, dtype, 0, 0, data, sig)
                if pl is None and dtype == DT_RAW_AIR_DATA:
                    # fall back to CRC-free reassembly to detect version mismatch
                    before = rx_nc.completed
                    pl = rx_nc.feed(src, dtype, 0, 0, data, None)
                    if pl is not None:
                        rad_crc_fail[0] += 1
                        print("[t+%5.2f] WARNING: RawAirData reassembled WITHOUT valid CRC "
                              "(dtid %d) - signature/layout mismatch? raw=%s"
                              % (rel, dtype, pl.hex(" ")))
                note_xfer(dtype, pl)
                if pl is not None:
                    handle_air_data(src, dtype, pl)
            else:
                note_xfer(dtype, rx.feed(src, dtype, 0, 0, data, None))
        time.sleep(0.01)

    rawlog.close()
    sl.stop()
    sl.sp.close()

    # ---------------- summarize ----------------
    online = bool(node_status)
    air_ok = bool(air_data or sp_msgs or st_msgs)
    if online and air_ok:
        verdict, code = "PASS", 0
    elif online:
        verdict, code = "PASS (partial: node online but no air data messages)", 3
    else:
        verdict, code = "FAIL (module never came online)", 1

    print("\n" + "=" * 74)
    print("VERDICT: %s" % verdict)
    print("  adapter .......... %s @ %d bit/s (slcan)" % (args.port, args.bitrate))
    print("  raw frames ....... %d (%d non-frame lines, %d service frames)"
          % (stats["frames"], stats["nonframe_lines"], stats["svc_frames"]))
    print("  allocation ....... requests=%d followups=%d grants=%d granted_node=%s"
          % (alloc.requests, alloc.followups, alloc.grants, alloc.granted_node))
    if alloc.granted_uid_hex:
        print("  unique id ........ %s" % alloc.granted_uid_hex)
    for src, st in sorted(node_status.items()):
        print("  node %d ........... uptime=%ds health=%s mode=%s vssc=%d"
              % (src, st["uptime_sec"], st["health"], st["mode"], st["vssc"]))
    if node_info:
        print("  device ........... name=%r sw=%s hw=%s vcs=%s"
              % (node_info.get("name"), node_info.get("sw"), node_info.get("hw"),
                 node_info.get("vcs_commit")))
        print("  device uid ....... %s" % node_info.get("unique_id"))
    if dti:
        for k in sorted(dti):
            r = dti[k]
            print("  dtype info ....... dtid %-5d %r sig=0x%s [%s]"
                  % (k, r["name"], r["signature"], ",".join(r["flag_names"])))
    rates = []
    for k in sorted(stats["msg_counts"]):
        n = stats["xfer_counts"].get(k, stats["msg_counts"][k])
        dt = stats["msg_last"][k] - stats["msg_first"][k]
        hz = (n - 1) / dt if dt > 0.5 else float("nan")
        rates.append("%s(%s)=%.1fHz" % (DTID_NAMES.get(k, "?"), k, hz))
    print("  message rates .... %s" % ", ".join(rates))
    if air_data:
        last = air_data[-1]
        print("  air data ......... %d RawAirData decoded (last: %s)"
              % (len(air_data), fmt_rad(last, air_sources[-1])))
        dps = [r["differential_pressure"] for r in air_data
               if r["differential_pressure"] == r["differential_pressure"]]
        if dps:
            print("  differential ..... min=%+.1f Pa max=%+.1f Pa (blow into the pitot to "
                  "see it move)" % (min(dps), max(dps)))
        pits = [r["pitot_temperature"] for r in air_data
                if r["pitot_temperature"] is not None
                and r["pitot_temperature"] == r["pitot_temperature"]
                and r["pitot_temperature"] < 65500]
        if pits:
            print("  pitot temp ....... %.1f C .. %.1f C" % (min(pits) - 273.15, max(pits) - 273.15))
        hf = set()
        for r in air_data:
            hf.update(r["heater"])
        print("  heater flags ..... %s" % (",".join(sorted(hf)) or "none reported"))
    if sp_msgs:
        print("  static pressure .. %d msgs, last %.1f Pa" % (len(sp_msgs), sp_msgs[-1]["static_pressure"]))
    if st_msgs:
        t = st_msgs[-1]["static_temperature"]
        print("  static temp ...... %d msgs, last %.2f K (%.1f C)" % (len(st_msgs), t, t - 273.15))
    if hygro_msgs:
        last = hygro_msgs[-1]
        rhs = [m["humidity"] for m in hygro_msgs if m["humidity"] == m["humidity"]]
        print("  hygrometer ....... %d msgs, last T=%s RH=%.1f%%"
              % (len(hygro_msgs),
                 ("%.2f (K?=%.1fC)" % (last["temperature"],
                                       last["temperature_c_if_kelvin"] or 0)
                  if last["temperature_c_if_kelvin"] is not None
                  else "%.2f" % last["temperature"]),
                 last["humidity"]))
        if rhs:
            print("  humidity ......... min=%.1f%% max=%.1f%%" % (min(rhs), max(rhs)))
    if unknown_payloads:
        print("  unknown dtids .... %s" % ", ".join("%d(x%d)" % (k, stats["msg_counts"][k])
                                                    for k in sorted(unknown_payloads)))
    print("  crc failures ..... %d (RawAirData CRC-free fallbacks: %d)"
          % (rx.crc_fail_count, rad_crc_fail[0]))
    print("  raw frame log .... %s" % rawlog_path)
    print("=" * 74)

    # per-dtid rate table for the report
    dt_rates = {}
    for k in sorted(stats["msg_counts"]):
        dt = stats["msg_last"][k] - stats["msg_first"][k]
        n = stats["xfer_counts"].get(k, stats["msg_counts"][k])
        hz = (n - 1) / dt if dt > 0.5 else None
        dt_rates[str(k)] = dict(name=DTID_NAMES.get(k, "unknown"),
                                frames=stats["msg_counts"][k],
                                transfers=n, hz=hz)

    result = dict(
        verdict=verdict, exit_code=code, port=args.port, bitrate=args.bitrate,
        allocator_node_id=my_id,
        allocation=dict(requests=alloc.requests, followups=alloc.followups,
                        grants=alloc.grants, granted_node=alloc.granted_node,
                        granted_unique_id=alloc.granted_uid_hex, table=dict(alloc.table)),
        node_status={str(k): v for k, v in node_status.items()},
        node_info=node_info,
        data_type_info=dti,
        air_data=dict(raw_air_data_count=len(air_data),
                      last_raw_air_data=air_data[-1] if air_data else None,
                      static_pressure_msgs=len(sp_msgs),
                      last_static_pressure=sp_msgs[-1] if sp_msgs else None,
                      static_temperature_msgs=len(st_msgs),
                      last_static_temperature=st_msgs[-1] if st_msgs else None,
                      hygrometer_msgs=len(hygro_msgs),
                      last_hygrometer=hygro_msgs[-1] if hygro_msgs else None),
        message_rates=dt_rates, frames=stats["frames"],
        crc_failures=rx.crc_fail_count,
        raw_air_data_crc_free_fallbacks=rad_crc_fail[0],
        unknown_dtid_payloads=unknown_payloads,
        raw_frame_log=str(rawlog_path),
    )
    jf = REPORTS / ("skye2_test_%s.json" % ts_tag)
    jf.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    md = REPORTS / ("skye2_test_%s.md" % ts_tag)
    md.write_text(report_md(result, air_data), encoding="utf-8")
    print("[REPORT] %s" % jf)
    print("[REPORT] %s" % md)
    return code


def report_md(r, air_data):
    lines = ["# CUAV SKYE2 空速计 DroneCAN 通信测试报告", "",
             "- 时间：%s" % time.strftime("%Y-%m-%d %H:%M:%S"),
             "- 适配器：%s（slcan，%d bit/s）" % (r["port"], r["bitrate"]),
             "- 结论：**%s**" % r["verdict"], ""]
    a = r["allocation"]
    lines += ["## 动态节点 ID 分配", "",
              "- 分配器节点 ID：%d" % r["allocator_node_id"],
              "- 分配请求：%d 次；follow-up：%d 次；授权：%d 次" %
              (a["requests"], a["followups"], a["grants"]),
              "- 被分配节点 ID：%s" % a["granted_node"],
              "- unique ID（16 字节）：`%s`" % (a["granted_unique_id"] or "未取得"), ""]
    if r["node_status"]:
        lines += ["## 节点状态（NodeStatus, dtid 341）", ""]
        for src, st in r["node_status"].items():
            lines.append("- 节点 %s：uptime=%ss，health=%s，mode=%s，vssc=%s"
                         % (src, st["uptime_sec"], st["health"], st["mode"], st["vssc"]))
        lines.append("")
    if r["node_info"]:
        ni = r["node_info"]
        lines += ["## 设备信息（GetNodeInfo）", "",
                  "- 名称：`%s`，软件 %s，硬件 %s，vcs_commit %s"
                  % (ni.get("name"), ni.get("sw"), ni.get("hw"), ni.get("vcs_commit")),
                  "- unique ID：`%s`" % ni.get("unique_id"), ""]
    if r["data_type_info"]:
        lines += ["## 数据类型信息（GetDataTypeInfo，由设备自报）", ""]
        for k in sorted(r["data_type_info"], key=int):
            d = r["data_type_info"][k]
            lines.append("- dtid %s：`%s`，签名 `0x%s`，flags=[%s]"
                         % (k, d["name"], d["signature"], ",".join(d["flag_names"]) or "none"))
        lines.append("")
    ad = r["air_data"]
    lines += ["## 空气数据", "",
              "- RawAirData（dtid 1027）解码：%d 条" % ad["raw_air_data_count"],
              "- StaticPressure（dtid 1028）：%d 条" % ad["static_pressure_msgs"],
              "- StaticTemperature（dtid 1029）：%d 条" % ad["static_temperature_msgs"],
              "- Hygrometer（dtid 1032）：**%d 条**" % ad["hygrometer_msgs"], ""]
    hy = ad.get("last_hygrometer")
    if hy:
        lines += ["最新 Hygrometer 解码（DSDL 定义 temperature 为摄氏度，但设备实测发送值按开尔文解释才合理）：", "",
                  "| 字段 | 值 |", "|---|---|",
                  "| temperature 原始值 | %.2f |" % hy["temperature"],
                  "| 若按开尔文解释 | %.2f K = %.1f °C |" % (
                      hy["temperature"], hy["temperature_c_if_kelvin"] or 0),
                  "| 湿度 | %.1f %% |" % hy["humidity"],
                  "| 传感器 id | %d |" % hy["sensor_id"], ""]
    last = ad["last_raw_air_data"]
    if last:
        def c(k):
            v = last.get(k)
            if v is None or v != v:
                return "n/a"
            return "%.2f K (%.1f C)" % (v, v - 273.15) if "temperature" in k else "%.1f" % v
        lines += ["最新 RawAirData 解码：", "",
                  "| 字段 | 值 |", "|---|---|",
                  "| 加热 flags | 0x%02X（%s） |" % (last["flags"], ",".join(last["heater"]) or "none"),
                  "| 静压 | %s Pa |" % c("static_pressure"),
                  "| 压差 | %s Pa |" % c("differential_pressure"),
                  "| 空速 IAS | %s |" % ("%.2f m/s" % last["ias_mps"] if last["ias_mps"] is not None else "n/a"),
                  "| 气压高度 | %s |" % ("%.0f m" % last["pressure_alt_m"] if last["pressure_alt_m"] is not None else "n/a"),
                  "| 静压传感器温度 | %s |" % c("static_pressure_sensor_temperature"),
                  "| 压差传感器温度 | %s |" % c("differential_pressure_sensor_temperature"),
                  "| 静态气温 | %s |" % c("static_air_temperature"),
                  "| 皮托管温度 | %s |" % c("pitot_temperature"),
                  "| 协方差元素数 | %d |" % len(last["covariance"]), ""]
    lines += ["## 消息统计（按 dtid）", ""]
    for k in sorted(r["message_rates"], key=int):
        d = r["message_rates"][k]
        hz = ("%.2f Hz" % d["hz"]) if d["hz"] else "—"
        lines.append("- dtid %s（%s）：%d 帧 / %d 条消息，%s"
                     % (k, d["name"], d["frames"], d.get("transfers", d["frames"]), hz))
    lines += ["", "- CRC 校验失败：%d；RawAirData 无 CRC 回退重组：%d"
              % (r["crc_failures"], r["raw_air_data_crc_free_fallbacks"])]
    if r["unknown_dtid_payloads"]:
        lines += ["- 未知 dtid（含原始负载）："]
        for k, v in sorted(r["unknown_dtid_payloads"].items(), key=lambda x: int(x[0])):
            lines.append("  - dtid %s：`%s`" % (k, v))
    lines += ["", "- 原始帧日志：`%s`" % r["raw_frame_log"], ""]
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--bitrate", type=int, default=1000000)
    ap.add_argument("--node-id", type=int, default=100, help="allocator node ID (default 100)")
    ap.add_argument("--alloc-timeout", type=float, default=45.0,
                    help="seconds to wait for the allocation exchange to complete")
    ap.add_argument("--monitor", type=float, default=60.0,
                    help="seconds to monitor air data after the node is online")
    ap.add_argument("--dti-ids", type=int, nargs="+",
                    default=[DT_RAW_AIR_DATA, DT_STATIC_PRESSURE, DT_STATIC_TEMPERATURE],
                    help="data type IDs to query via GetDataTypeInfo")
    ap.add_argument("--no-getinfo", dest="getinfo", action="store_false",
                    help="skip the GetNodeInfo query")
    args = ap.parse_args()
    try:
        return run(args)
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 1


if __name__ == "__main__":
    sys.exit(main())
