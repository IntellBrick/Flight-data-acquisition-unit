#!/usr/bin/env python3
"""GPS DroneCAN dynamic-node-ID allocation + GNSS position test.

Target: CUAV NEO 4SE (ArduPilot-family DroneCAN GNSS) on a Makerbase CANable 2.0
adapter in SLCAN mode (default port COM10, 1 Mbit/s). The module boots with
CAN_NODE_ID=0, i.e. it waits for a UAVCAN v0 dynamic node ID allocator before it
publishes anything. This script:

  1. Acts as a spec-compliant UAVCAN v0 dynamic node ID allocator
     (uavcan.protocol.dynamic_node_id.Allocation, 3-stage 6+6+4 exchange,
      follow-up after each partial stage, grant only on the full 16-byte unique ID).
  2. Verifies the module comes online: uavcan.protocol.NodeStatus (dtid 341).
  3. Queries uavcan.protocol.GetNodeInfo (service 1) to identify the device and
     cross-check its HardwareVersion.unique_id against the granted one.
  4. Decodes uavcan.equipment.gnss.Fix (1060) / Fix2 (1063) / Auxiliary (1061)
     into position / velocity / status and prints a live summary.
  5. Writes JSON + Markdown reports and a raw frame log under reports/.

Why the earlier allocators (tools/dronecan_allocator1..6.py) failed:
  they treated 12 bytes as the complete unique ID and granted immediately.
  The unique ID is 16 bytes, delivered in three single-frame stages (6+6+4);
  a grant is only accepted (Rule E) when it echoes the FULL 16-byte unique ID.
  After stage 2 the allocator must send a follow-up echoing the accumulated
  12 bytes, which triggers the allocatee's stage-3 request with the last 4 bytes.
  This implementation follows the state machine from the UAVCAN v0 specification
  and is cross-validated byte-for-byte against the spec's own example log
  (run --self-test to see the validation).

Wire-format facts used (all verified against spec + pydronecan):
  * CAN ID (message): prio[28:24] | data_type_id[23:8] | 0[7] | source[6:0]
  * CAN ID (anonymous): prio[28:24] | discriminator[23:10] | dtid_low2[9:8] | 0[7] | 0[6:0]
  * CAN ID (service):   prio[28:24] | svc_type[23:16] | request[15] | dest[14:8] | 1[7] | source[6:0]
  * Allocation payload: byte0 = (node_id << 1) | first_part_of_unique_id, then
    tail-optimized uint8[<=16] unique_id (no length prefix).
  * DSDL bit packing: fields in declaration order fill the stream; each field's
    value is stored little-endian (low bits first at the stream position), i.e.
    byte-aligned integers are little-endian, sub-byte fields are MSB-first.
    (mirrors pydronecan's bits_from_bytes / be_from_le_bits exactly)
  * Multi-frame transfers: first frame begins with the 2-byte LE transfer CRC;
    tail byte = start<<7 | end<<6 | toggle<<5 | transfer_id (toggle starts 0 and
    alternates on every frame after the first).
  * The transfer CRC is CRC-16-CCITT-FALSE over (data type signature, 8 bytes
    little-endian) + payload. Signatures below were computed with pydronecan's
    DSDL compiler (dronecan.dsdl.parser) and the Allocation one additionally
    verified against the spec's example log (CRC 0xB005).

Exit codes: 0 = PASS (node online + GNSS messages decoded),
            3 = partial (node online, no GNSS messages),
            1 = FAIL (module never came online),  4 = self-test failed.
"""
import argparse
import json
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from slcan_can_test import Slcan, parse_slcan_frame  # SLCAN serial link only

WS = Path(__file__).resolve().parent.parent
REPORTS = WS / "reports"

# ---------------- UAVCAN v0 constants ----------------
ALLOC_DTID = 1            # uavcan.protocol.dynamic_node_id.Allocation
DT_NODE_STATUS = 341      # uavcan.protocol.NodeStatus
DT_GNSS_AUX = 1061        # uavcan.equipment.gnss.Auxiliary
DT_GNSS_FIX = 1060        # uavcan.equipment.gnss.Fix
DT_GNSS_FIX2 = 1063       # uavcan.equipment.gnss.Fix2
SVC_GET_NODE_INFO = 1     # uavcan.protocol.GetNodeInfo
PRIO = 30                 # same priority the spec example allocator uses
FOLLOWUP_TIMEOUT = 0.5    # s, per spec FOLLOWUP_TIMEOUT_MS = 500
UID_LEN = 16              # unique ID is 128-bit
STAGE_MAX = 6             # MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST

HEALTH = {0: "OK", 1: "WARNING", 2: "ERROR", 3: "CRITICAL"}
MODE = {0: "OPERATIONAL", 1: "INITIALIZATION", 2: "MAINTENANCE", 3: "SOFTWARE_UPDATE", 7: "OFFLINE"}
FIX_STATUS = {0: "NO_FIX", 1: "TIME_ONLY", 2: "2D_FIX", 3: "3D_FIX"}
TIME_STD = {0: "NONE", 1: "TAI", 2: "UTC", 3: "GPS"}
GNSS_MODE = {0: "SINGLE", 1: "DGPS", 2: "RTK", 3: "PPP"}

# 64-bit data type signatures (computed with pydronecan's DSDL compiler)
SIG_ALLOCATION = 0x0B2A812620A11D40   # uavcan.protocol.dynamic_node_id.Allocation
SIG_GET_NODE_INFO = 0xEE468A8121C46A9E
DTYPE_SIGNATURE = {
    DT_NODE_STATUS: 0x0F0868D0C1A7C6F1,
    DT_GNSS_AUX: 0x9BE8BDC4C3DBBFD2,
    DT_GNSS_FIX: 0x54C1572B9E07F297,
    DT_GNSS_FIX2: 0xCA41E7000F37435F,
}


# ---------------- CRC + transfer reassembly ----------------
def crc16_uavcan(data: bytes) -> int:
    """UAVCAN v0 transfer CRC: CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def transfer_crc(payload: bytes, signature: int) -> int:
    """UAVCAN v0 transfer CRC: CRC-16-CCITT-FALSE over signature(LE) + payload."""
    return crc16_uavcan(signature.to_bytes(8, "little") + bytes(payload))


class Transfer:
    __slots__ = ("payload", "last_toggle", "ts0")

    def __init__(self):
        self.payload = b""
        self.last_toggle = False
        self.ts0 = 0.0


class Reassembler:
    """UAVCAN v0 multi-frame reassembly keyed on (src, dtype, svc, req, tid).

    Note: tools/can_comm_test.py has a Reassembler with (a) an inverted toggle
    check (it drops the 2nd frame of every multi-frame transfer) and (b) a CRC
    computed without the data-type-signature prefix, so it would fail every
    multi-frame CRC check. This is a corrected implementation: the toggle bit
    must ALTERNATE between frames after the first, and the CRC covers
    signature(LE) + payload.
    """

    def __init__(self):
        self.bufs = {}
        self.crc_fail_count = 0
        self.completed = 0

    def _prune(self, now):
        stale = [k for k, v in self.bufs.items() if now - v.ts0 > 1.0]
        for k in stale:
            self.bufs.pop(k, None)

    def feed(self, src, dtype, svc, req, payload, signature=None):
        """payload = raw CAN data incl. tail byte. Returns completed body or None.

        signature: data type signature for the CRC check (None = skip check)."""
        if not payload:
            return None
        now = time.time()
        self._prune(now)
        tail = payload[-1]
        start = bool(tail & 0x80)
        end = bool(tail & 0x40)
        toggle = bool(tail & 0x20)
        tid = tail & 0x1F
        data = payload[:-1]
        key = (src, dtype, svc, req, tid)

        if start and end:                      # single-frame transfer
            self.completed += 1
            return data
        if start:
            t = Transfer()
            t.payload = data
            t.last_toggle = toggle
            t.ts0 = now
            self.bufs[key] = t
            return None
        cur = self.bufs.get(key)
        if cur is None or toggle == cur.last_toggle:   # toggle must alternate
            self.bufs.pop(key, None)
            return None
        cur.last_toggle = toggle
        cur.payload += data
        if end:
            self.bufs.pop(key)
            full = cur.payload
            if len(full) >= 2:
                want = struct.unpack_from("<H", full)[0]
                body = full[2:]
                if signature is not None:
                    if transfer_crc(body, signature) != want:
                        self.crc_fail_count += 1
                        return None
                self.completed += 1
                return body
        return None


# ---------------- transfer framing (TX) ----------------
def build_frames(payload: bytes, tid: int, signature: int):
    """Split a payload into UAVCAN v0 CAN data fields (each incl. tail byte)."""
    payload = bytes(payload)
    if len(payload) <= 7:
        return [payload + bytes([0xC0 | (tid & 0x1F)])]
    body = struct.pack("<H", transfer_crc(payload, signature)) + payload
    chunks = [body[i:i + 7] for i in range(0, len(body), 7)]
    n = len(chunks)
    out = []
    for i, ch in enumerate(chunks):
        tail = ((0x80 if i == 0 else 0) | (0x40 if i == n - 1 else 0)
                | (0x20 if (i & 1) else 0) | (tid & 0x1F))
        out.append(bytes(ch) + bytes([tail]))
    return out


# ---------------- DSDL bit-stream decoder (pydronecan-compatible) ----------------
class BitStream:
    """UAVCAN v0 serialized form: little-endian bit stream, byte groups reversed.

    Mirrors pydronecan's bits_from_bytes()/be_from_le_bits() pair exactly.
    """

    def __init__(self, data):
        self.s = "".join(format(c, "08b") for c in bytes(data))

    def read(self, n):
        if n == 0:
            return 0
        if len(self.s) < n:
            raise EOFError("bitstream exhausted")
        seg = self.s[:n]
        self.s = self.s[n:]
        groups = [seg[i:i + 8] for i in range(0, len(seg), 8)]
        return int("".join(groups[::-1]), 2)

    def read_signed(self, n):
        v = self.read(n)
        return v - (1 << n) if v & (1 << (n - 1)) else v

    def read_float(self, n):
        v = self.read(n)
        if n == 16:
            return struct.unpack("<e", v.to_bytes(2, "little"))[0]
        if n == 32:
            return struct.unpack("<f", v.to_bytes(4, "little"))[0]
        if n == 64:
            return struct.unpack("<d", v.to_bytes(8, "little"))[0]
        raise ValueError("bad float width %d" % n)

    def rest_bytes(self):
        s = self.s + "0" * ((-len(self.s)) % 8)
        return bytes(int(s[i:i + 8], 2) for i in range(0, len(s), 8))


def _le_bits(value, bitlen):
    """Encode a field value the way pydronecan does (for self-tests)."""
    be = format(value & ((1 << bitlen) - 1), "0%db" % bitlen)
    return "".join([be[max(0, i - 8):i] for i in range(bitlen, 0, -8)])


def _bits_to_bytes(s):
    s = s + "0" * ((-len(s)) % 8)
    return bytes(int(s[i:i + 8], 2) for i in range(0, len(s), 8))


# ---------------- message decoders ----------------
def decode_node_status(p):
    if len(p) < 7:
        return None
    bs = BitStream(p)
    uptime = bs.read(32)
    health = bs.read(2)
    mode = bs.read(3)
    sub_mode = bs.read(3)
    vssc = bs.read(16)
    return dict(uptime_sec=uptime, health=HEALTH.get(health, health), health_code=health,
                mode=MODE.get(mode, mode), mode_code=mode, sub_mode=sub_mode, vssc=vssc)


def decode_fix(p):
    """uavcan.equipment.gnss.Fix (dtid 1060)."""
    if len(p) < 41:
        return None
    bs = BitStream(p)
    out = dict(msg="Fix")
    out["timestamp_us"] = bs.read(56)
    out["gnss_timestamp_us"] = bs.read(56)
    out["gnss_time_standard"] = TIME_STD.get(bs.read(3), "?")
    bs.read(5)                       # void5
    out["num_leap_seconds"] = bs.read(8)
    out["lon_deg"] = bs.read_signed(37) / 1e8
    out["lat_deg"] = bs.read_signed(37) / 1e8
    out["height_ellipsoid_m"] = bs.read_signed(27) / 1000.0
    out["height_msl_m"] = bs.read_signed(27) / 1000.0
    out["vel_n_mps"], out["vel_e_mps"], out["vel_d_mps"] = (
        bs.read_float(16), bs.read_float(16), bs.read_float(16))
    out["sats_used"] = bs.read(6)
    out["status"] = FIX_STATUS.get(bs.read(2), "?")
    out["pdop"] = bs.read_float(16)
    try:
        bs.read(4)                   # void4
        n_pc = bs.read(4)            # position_covariance length prefix
        out["position_covariance"] = [round(bs.read_float(16), 4) for _ in range(n_pc)]
        n_vc = len(bs.s) // 16       # velocity_covariance: tail-optimized
        out["velocity_covariance"] = [round(bs.read_float(16), 4) for _ in range(n_vc)]
    except EOFError:
        pass
    return out


def decode_fix2(p):
    """uavcan.equipment.gnss.Fix2 (dtid 1063)."""
    if len(p) < 47:
        return None
    bs = BitStream(p)
    out = dict(msg="Fix2")
    out["timestamp_us"] = bs.read(56)
    out["gnss_timestamp_us"] = bs.read(56)
    out["gnss_time_standard"] = TIME_STD.get(bs.read(3), "?")
    bs.read(13)                      # void13
    out["num_leap_seconds"] = bs.read(8)
    out["lon_deg"] = bs.read_signed(37) / 1e8
    out["lat_deg"] = bs.read_signed(37) / 1e8
    out["height_ellipsoid_m"] = bs.read_signed(27) / 1000.0
    out["height_msl_m"] = bs.read_signed(27) / 1000.0
    out["vel_n_mps"], out["vel_e_mps"], out["vel_d_mps"] = (
        bs.read_float(32), bs.read_float(32), bs.read_float(32))
    out["sats_used"] = bs.read(6)
    out["status"] = FIX_STATUS.get(bs.read(2), "?")
    out["mode"] = GNSS_MODE.get(bs.read(4), "?")
    out["sub_mode"] = bs.read(6)
    try:
        n = bs.read(6)               # covariance length prefix (float16[<=36])
        cov = [round(bs.read_float(16), 4) for _ in range(n)]
        out["covariance"] = cov
        out["pdop"] = bs.read_float(16)
    except EOFError:
        pass
    return out


def decode_auxiliary(p):
    """uavcan.equipment.gnss.Auxiliary (dtid 1061)."""
    if len(p) < 4:
        return None
    bs = BitStream(p)
    return dict(msg="Auxiliary", vdop=bs.read_float(16), hdop=bs.read_float(16),
                cnt=bs.rest_bytes().hex(" "))


def parse_get_node_info(p):
    """uavcan.protocol.GetNodeInfo response."""
    try:
        bs = BitStream(p)
        st = dict(uptime_sec=bs.read(32), health=HEALTH.get(bs.read(2), "?"),
                  mode=MODE.get(bs.read(3), "?"))
        bs.read(3)      # sub_mode
        bs.read(16)     # vssc
        sw_major, sw_minor = bs.read(8), bs.read(8)
        opt_flags = bs.read(8)
        vcs_commit = bs.read(32)
        image_crc = bs.read(64)
        hw_major, hw_minor = bs.read(8), bs.read(8)
        uid = bytes(bs.read(8) for _ in range(16))
        cert_len = bs.read(8)
        cert = bytes(bs.read(8) for _ in range(cert_len))
        name = bs.rest_bytes().decode("ascii", "replace").rstrip("\x00")
        return dict(status=st, sw="%d.%d" % (sw_major, sw_minor), hw="%d.%d" % (hw_major, hw_minor),
                    optional_field_flags=opt_flags, vcs_commit="%08x" % vcs_commit,
                    image_crc="%016x" % image_crc, unique_id=uid.hex(" "), name=name)
    except EOFError:
        return None


# ---------------- allocator ----------------
def parse_alloc_request(data):
    """Allocation request CAN data (incl. tail) -> (first_part, preferred_id, chunk)."""
    if len(data) < 3:
        return None
    first_part = data[0] & 1
    preferred = (data[0] >> 1) & 0x7F
    chunk = bytes(data[1:-1])
    return first_part, preferred, chunk


class Allocator:
    """UAVCAN v0 dynamic node ID allocator state machine (single allocatee)."""

    def __init__(self, sl, my_id, log):
        self.sl = sl
        self.my_id = my_id
        self.arb = (PRIO << 24) | (ALLOC_DTID << 8) | my_id
        self.uid = b""
        self.last_req_ts = 0.0
        self.tid = 0
        self.table = {}          # unique_id hex -> granted node id
        self.requests = 0
        self.followups = 0
        self.grants = 0
        self.granted_node = None
        self.granted_uid_hex = None
        self.log = log

    def _tx(self, payload):
        frames = build_frames(payload, self.tid, SIG_ALLOCATION)
        self.tid = (self.tid + 1) & 0x1F
        for f in frames:
            self.sl.send_ext(self.arb, f)
            if len(frames) > 1:
                time.sleep(0.002)

    def handle_request(self, now, data, disc):
        pr = parse_alloc_request(data)
        if pr is None:
            return
        first_part, preferred, chunk = pr
        self.requests += 1

        # Spec: reset expected stage if no follow-up request within FOLLOWUP_TIMEOUT.
        if self.uid and now - self.last_req_ts > FOLLOWUP_TIMEOUT:
            self.log("[t+%5.2f] ALLOC state reset (followup timeout)" % now)
            self.uid = b""

        # detectRequestStage()
        if first_part and len(chunk) == STAGE_MAX:
            stage = 1
        elif not first_part and len(chunk) == STAGE_MAX:
            stage = 2
        elif not first_part and len(chunk) == UID_LEN - 2 * STAGE_MAX:
            stage = 3
        else:
            self.log("[t+%5.2f] ALLOC malformed request (first_part=%d, %d bytes) ignored"
                     % (now, first_part, len(chunk)))
            return
        # getExpectedStage()
        expected = 1 if not self.uid else (3 if len(self.uid) >= 2 * STAGE_MAX else 2)
        if stage != expected:
            return          # stage mismatch: ignore, allocatee will restart on its own
        if len(chunk) > UID_LEN - len(self.uid):
            return

        if stage == 1:
            self.uid = chunk       # restart collection
        else:
            self.uid += chunk
        self.last_req_ts = now

        if len(self.uid) < UID_LEN:
            # follow-up: echo ALL bytes collected so far (Rule D trigger)
            payload = bytes([0x00]) + self.uid
            self._tx(payload)
            self.followups += 1
            self.log("[t+%5.2f] ALLOC stage %d ok, %d/16 uid bytes -> follow-up echo"
                     % (now, stage, len(self.uid)))
        else:
            uid, self.uid = self.uid, b""
            gid = self.table.get(uid.hex())
            if gid is None:
                gid = preferred if (1 <= preferred <= 125 and preferred != self.my_id) else 125
                self.table[uid.hex()] = gid
            payload = bytes([gid << 1]) + uid      # node_id != 0 => grant
            self._tx(payload)
            self.grants += 1
            self.granted_node = gid
            self.granted_uid_hex = uid.hex(" ")
            self.log("[t+%5.2f] ALLOC unique ID complete (16/16) -> GRANT node_id=%d uid=%s"
                     % (now, gid, uid.hex(" ")))


# ---------------- self-test against the spec's example log ----------------
def self_test():
    # Spec example allocatee unique ID: 44 C0 8B 63 5E 05 F4 BC 10 96 DF 11 A8 BA 54 47
    uid12 = bytes.fromhex("44C08B635E05F4BC1096DF11")
    followup1 = bytes([0x00]) + bytes.fromhex("44C08B635E05")   # after stage 1
    followup2 = bytes([0x00]) + uid12                           # after stage 2
    grant = bytes([0xFA]) + uid12 + bytes.fromhex("A8BA5447")   # node_id=125, full uid

    # 1) transfer CRCs must match the spec log (signature-prepended CRC)
    assert transfer_crc(followup2, SIG_ALLOCATION) == 0xB005, \
        "CRC followup2 mismatch: %04X" % transfer_crc(followup2, SIG_ALLOCATION)
    assert transfer_crc(grant, SIG_ALLOCATION) == 0xBA29, \
        "CRC grant mismatch: %04X" % transfer_crc(grant, SIG_ALLOCATION)

    # 2) our frame builder must reproduce the spec log byte-for-byte
    assert build_frames(followup1, 0, SIG_ALLOCATION) == [bytes.fromhex("0044C08B635E05C0")]
    assert build_frames(followup2, 1, SIG_ALLOCATION) == [bytes.fromhex(x) for x in
                                                         ("05B00044C08B6381", "5E05F4BC1096DF21", "1141")]
    assert build_frames(grant, 2, SIG_ALLOCATION) == [bytes.fromhex(x) for x in
                                                      ("29BAFA44C08B6382", "5E05F4BC1096DF22", "11A8BA544742")]

    # 3) request parsing
    fp, pref, chunk = parse_alloc_request(bytes.fromhex("0144C08B635E05C0"))
    assert (fp, pref, chunk) == (1, 0, bytes.fromhex("44C08B635E05"))
    fp, pref, chunk = parse_alloc_request(bytes.fromhex("00F4BC1096DF11C1"))
    assert (fp, pref, chunk) == (0, 0, bytes.fromhex("F4BC1096DF11"))
    fp, pref, chunk = parse_alloc_request(bytes.fromhex("00A8BA5447C2"))
    assert (fp, pref, chunk) == (0, 0, bytes.fromhex("A8BA5447"))

    # 4) reassembler toggle handling + CRC (spec example 3-frame follow-up)
    rx = Reassembler()
    frames = build_frames(followup2, 1, SIG_ALLOCATION)
    bodies = [rx.feed(1, ALLOC_DTID, 0, 0, f, SIG_ALLOCATION) for f in frames]
    assert bodies == [None, None, followup2], bodies
    assert rx.crc_fail_count == 0
    rx2 = Reassembler()
    wrong_sig = [rx2.feed(1, ALLOC_DTID, 0, 0, f, 0x1122334455667788) for f in frames]
    assert wrong_sig[-1] is None and rx2.crc_fail_count == 1, "CRC check did not reject wrong signature"

    # 5) NodeStatus bit layout round-trip
    s = _le_bits(42, 32) + _le_bits(1, 2) + _le_bits(2, 3) + _le_bits(0, 3) + _le_bits(513, 16)
    st = decode_node_status(_bits_to_bytes(s))
    assert st["uptime_sec"] == 42 and st["health"] == "WARNING" and st["mode"] == "MAINTENANCE"
    assert st["vssc"] == 513, st

    # 6) Fix round-trip with known values
    lon_raw, lat_raw = int(116.3975 * 1e8), int(39.9089 * 1e8)
    s = (_le_bits(12345678, 56) + _le_bits(87654321, 56) + _le_bits(3, 3) + _le_bits(0, 5)
         + _le_bits(18, 8) + _le_bits(lon_raw, 37) + _le_bits(lat_raw, 37)
         + _le_bits(43210, 27) + _le_bits(50000, 27)      # ellipsoid 43.21m, msl 50.0m
         + _le_bits(0x3E00, 16) + _le_bits(0xC000, 16) + _le_bits(0x3400, 16)   # 1.5, -2.0, 0.25
         + _le_bits(12, 6) + _le_bits(3, 2) + _le_bits(0x3F33, 16))             # 12 sats, 3D, pdop 1.8
    fx = decode_fix(_bits_to_bytes(s))
    assert fx["lon_deg"] == 116.3975 and fx["lat_deg"] == 39.9089, fx
    assert fx["height_msl_m"] == 50.0 and fx["height_ellipsoid_m"] == 43.21, fx
    assert fx["sats_used"] == 12 and fx["status"] == "3D_FIX" and abs(fx["pdop"] - 1.8) < 1e-3, fx
    assert abs(fx["vel_n_mps"] - 1.5) < 1e-3 and abs(fx["vel_e_mps"] + 2.0) < 1e-2

    print("self-test: all checks PASSED (spec example log reproduced byte-for-byte)")
    return 0


# ---------------- main test ----------------
def fmt_fix(fx, src):
    return ("node %d %s: %s lat=%.7f lon=%.7f alt_msl=%.1fm alt_ell=%.1fm "
            "vel=(N%.2f E%.2f D%.2f)m/s sats=%d pdop=%.1f gnss_ts=%dus"
            % (src, fx["msg"], fx["status"], fx["lat_deg"], fx["lon_deg"],
               fx["height_msl_m"], fx["height_ellipsoid_m"],
               fx["vel_n_mps"], fx["vel_e_mps"], fx["vel_d_mps"],
               fx["sats_used"], fx.get("pdop", float("nan")), fx["gnss_timestamp_us"]))


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
    rawlog_path = REPORTS / ("gps_alloc_test_%s_frames.log" % ts_tag)
    REPORTS.mkdir(exist_ok=True)
    rawlog = rawlog_path.open("w", encoding="ascii")

    rx = Reassembler()
    alloc = Allocator(sl, my_id, print)
    stats = {"msg_counts": {}, "svc_frames": 0, "nonframe_lines": 0, "frames": 0}
    node_status = {}          # src -> latest decode
    first_nodestatus = [None]
    fixes = []                # decoded Fix/Fix2 payloads
    fix_sources = []          # src per decoded fix
    node_info = {}
    getinfo = {"pending": None, "sent_ts": 0.0, "tries": 0, "done": False}

    t0 = time.time()
    alloc_deadline = t0 + args.alloc_timeout
    monitor_end = None
    _ltp = [0.0]

    def throttled_nodestatus(src, st, now):
        node_status[src] = st
        if first_nodestatus[0] is None:
            first_nodestatus[0] = now
            print("[t+%5.2f] *** NODE ONLINE: NodeStatus from node %d (uptime=%ds health=%s mode=%s)"
                  % (now, src, st["uptime_sec"], st["health"], st["mode"]))
        elif now - _ltp[0] >= 5.0:
            _ltp[0] = now
            print("[t+%5.2f] NodeStatus node %d: uptime=%ds health=%s mode=%s vssc=%d"
                  % (now, src, st["uptime_sec"], st["health"], st["mode"], st["vssc"]))

    while True:
        now = time.time()
        rel = now - t0
        if monitor_end is None and (alloc.granted_node is not None or first_nodestatus[0] is not None):
            monitor_end = now + args.monitor
            print("[t+%5.2f] allocation phase finished -> monitoring %ds for GNSS data"
                  % (rel, args.monitor))
        if monitor_end is not None and now >= monitor_end:
            break
        if monitor_end is None and now >= alloc_deadline:
            print("[t+%5.2f] allocation timeout reached without the node coming online" % rel)
            break

        # fire GetNodeInfo once the node is alive (retry once)
        target = alloc.granted_node if alloc.granted_node is not None else (
            min(node_status) if node_status else None)
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
                if (stype == SVC_GET_NODE_INFO and not req and dest == my_id
                        and src == getinfo["pending"]):
                    pl = rx.feed(src, SVC_GET_NODE_INFO, 1, 0, data, SIG_GET_NODE_INFO)
                    if pl is not None:
                        info = parse_get_node_info(pl)
                        if info:
                            node_info = info
                            getinfo["done"] = True
                            print("[t+%5.2f] GetNodeInfo node %d: name=%r sw=%s hw=%s uid=%s"
                                  % (rel, src, info["name"], info["sw"], info["hw"],
                                     info["unique_id"]))
                            if alloc.granted_uid_hex and info["unique_id"] == alloc.granted_uid_hex:
                                print("[t+%5.2f] unique_id from GetNodeInfo MATCHES the granted one"
                                      % rel)
                continue

            src = arb & 0x7F
            if src == 0:                                     # anonymous message
                if (arb >> 8) & 3 == ALLOC_DTID:             # Allocation request (dtid low 2 bits)
                    disc = (arb >> 10) & 0x3FFF
                    alloc.handle_request(rel, data, disc)
                continue

            dtype = (arb >> 8) & 0xFFFF
            key = "node%d/dtid%d" % (src, dtype)
            stats["msg_counts"][key] = stats["msg_counts"].get(key, 0) + 1
            pl = rx.feed(src, dtype, 0, 0, data, DTYPE_SIGNATURE.get(dtype))
            if pl is None:
                continue
            if dtype == DT_NODE_STATUS:
                st = decode_node_status(pl)
                if st:
                    throttled_nodestatus(src, st, rel)
            elif dtype in (DT_GNSS_FIX, DT_GNSS_FIX2):
                fx = decode_fix(pl) if dtype == DT_GNSS_FIX else decode_fix2(pl)
                if fx:
                    fixes.append(fx)
                    fix_sources.append(src)
                    print("[t+%5.2f] GNSS %s" % (rel, fmt_fix(fx, src)))
                else:
                    print("[t+%5.2f] GNSS dtid=%d from node %d: payload too short (%dB)"
                          % (rel, dtype, src, len(pl)))
            elif dtype == DT_GNSS_AUX:
                ax = decode_auxiliary(pl)
                if ax:
                    print("[t+%5.2f] GNSS Auxiliary node %d: vdop=%.1f hdop=%.1f"
                          % (rel, src, ax["vdop"], ax["hdop"]))
        time.sleep(0.01)

    rawlog.close()
    sl.stop()
    sl.sp.close()

    # ---------------- summarize ----------------
    online = bool(node_status)
    gnss_ok = bool(fixes)
    last_fx = fixes[-1] if fixes else None
    pos_lock = bool(last_fx and last_fx.get("status") in ("2D_FIX", "3D_FIX")
                    and (last_fx["lat_deg"] != 0 or last_fx["lon_deg"] != 0))
    if online and gnss_ok:
        verdict, code = "PASS", 0
    elif online:
        verdict, code = "PASS (partial: node online but no GNSS messages)", 3
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
    if node_status:
        for src, st in sorted(node_status.items()):
            print("  node %d ........... uptime=%ds health=%s mode=%s vssc=%d"
                  % (src, st["uptime_sec"], st["health"], st["mode"], st["vssc"]))
    if node_info:
        print("  device ........... name=%r sw=%s hw=%s vcs=%s"
              % (node_info.get("name"), node_info.get("sw"), node_info.get("hw"),
                 node_info.get("vcs_commit")))
        print("  device uid ....... %s" % node_info.get("unique_id"))
    if last_fx:
        print("  gnss messages .... %d decoded (last: %s)"
              % (len(fixes), fmt_fix(last_fx, fix_sources[-1] if fix_sources else 0)))
        print("  position lock .... %s" % ("YES" if pos_lock else
                                           "no (status=%s - antenna sky view?)" % last_fx["status"]))
    print("  message rates .... %s" % ", ".join(
        "%s=%d" % (k, v) for k, v in sorted(stats["msg_counts"].items())))
    print("  crc failures ..... %d" % rx.crc_fail_count)
    print("  raw frame log .... %s" % rawlog_path)
    print("=" * 74)

    result = dict(
        verdict=verdict, exit_code=code, port=args.port, bitrate=args.bitrate,
        allocator_node_id=my_id,
        allocation=dict(requests=alloc.requests, followups=alloc.followups,
                        grants=alloc.grants, granted_node=alloc.granted_node,
                        granted_unique_id=alloc.granted_uid_hex, table=dict(alloc.table)),
        node_status={str(k): v for k, v in node_status.items()},
        node_info=node_info,
        gnss=dict(count=len(fixes), first=fixes[0] if fixes else None,
                  last=last_fx, position_lock=pos_lock),
        message_counts=stats["msg_counts"], frames=stats["frames"],
        crc_failures=rx.crc_fail_count,
        raw_frame_log=str(rawlog_path),
    )
    jf = REPORTS / ("gps_alloc_test_%s.json" % ts_tag)
    jf.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    md = REPORTS / ("gps_alloc_test_%s.md" % ts_tag)
    md.write_text(report_md(result, fixes), encoding="utf-8")
    print("[REPORT] %s" % jf)
    print("[REPORT] %s" % md)
    return code


DTID_NAMES = {
    DT_NODE_STATUS: "uavcan.protocol.NodeStatus",
    DT_GNSS_AUX: "uavcan.equipment.gnss.Auxiliary",
    DT_GNSS_FIX: "uavcan.equipment.gnss.Fix",
    DT_GNSS_FIX2: "uavcan.equipment.gnss.Fix2",
    1002: "uavcan.equipment.ahrs.MagneticFieldStrength2",
    20003: "ardupilot.gnss.Status",
}


def report_md(r, fixes):
    lines = ["# GPS DroneCAN 分配与定位测试报告", "",
             "- 时间：%s" % time.strftime("%Y-%m-%d %H:%M:%S"),
             "- 适配器：%s（slcan，%d bit/s）" % (r["port"], r["bitrate"]),
             "- 结论：**%s**" % r["verdict"], ""]
    a = r["allocation"]
    lines += ["## 动态节点 ID 分配", "",
              "- 分配器节点 ID：%d" % r["allocator_node_id"],
              "- 收到分配请求：%d 次；发送 follow-up：%d 次；授权（grant）：%d 次"
              % (a["requests"], a["followups"], a["grants"]),
              "- 被分配节点 ID：%s" % a["granted_node"],
              "- 模块 unique ID（16 字节）：`%s`" % (a["granted_unique_id"] or "未取得"), ""]
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
                  "- 设备 unique ID：`%s`" % ni.get("unique_id"), ""]
    g = r["gnss"]
    lines += ["## GNSS 定位信息", "",
              "- 解码消息数：%d（Fix/Fix2，dtid 1060/1063）" % g["count"],
              "- 位置锁定：%s" % ("是" if g["position_lock"] else "否（状态：%s，可能无天空视野）"
                                  % (g["last"] or {}).get("status", "N/A")), ""]
    if g["last"]:
        fx = g["last"]
        lines += ["最新定位解：", "",
                  "| 字段 | 值 |", "|---|---|",
                  "| 状态 | %s |" % fx["status"],
                  "| 纬度 | %.7f° |" % fx["lat_deg"],
                  "| 经度 | %.7f° |" % fx["lon_deg"],
                  "| 海拔（MSL） | %.1f m |" % fx["height_msl_m"],
                  "| 海拔（椭球高） | %.1f m |" % fx["height_ellipsoid_m"],
                  "| NED 速度 | N %.2f / E %.2f / D %.2f m/s |"
                  % (fx["vel_n_mps"], fx["vel_e_mps"], fx["vel_d_mps"]),
                  "| 使用卫星数 | %d |" % fx["sats_used"],
                  "| PDOP | %s |" % fx.get("pdop"),
                  "| GNSS 时间戳 | %d µs（%s） |" % (fx["gnss_timestamp_us"],
                                                    fx.get("gnss_time_standard")), ""]
    lines += ["## 消息统计", ""]
    for k, v in sorted(r["message_counts"].items()):
        try:
            dtid = int(k.split("dtid")[1])
            name = DTID_NAMES.get(dtid, "")
        except (ValueError, IndexError):
            name = ""
        lines.append("- `%s`：%d 帧%s" % (k, v, "（%s）" % name if name else ""))
    lines += ["", "- CRC 校验失败：%d" % r["crc_failures"],
              "- 原始帧日志：`%s`" % r["raw_frame_log"], ""]
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--bitrate", type=int, default=1000000)
    ap.add_argument("--node-id", type=int, default=100, help="allocator node ID (default 100)")
    ap.add_argument("--alloc-timeout", type=float, default=45.0,
                    help="seconds to wait for the allocation exchange to complete")
    ap.add_argument("--monitor", type=float, default=30.0,
                    help="seconds to monitor GNSS data after the node is online")
    ap.add_argument("--no-getinfo", dest="getinfo", action="store_false",
                    help="skip the GetNodeInfo query")
    ap.add_argument("--self-test", action="store_true",
                    help="validate framing/decoding against the UAVCAN spec example and exit")
    args = ap.parse_args()
    if args.self_test:
        return self_test()
    try:
        return run(args)
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 1


if __name__ == "__main__":
    sys.exit(main())
