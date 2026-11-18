"""校验 FDAULOG v1 二进制日志，并将通过 CRC 的完整记录流式导出为 CSV。"""

import argparse
import csv
import struct
import sys
import zlib
from pathlib import Path


MAGIC = b"FDAULOG\0"
VERSION = 1
PERIOD_US = 200_000
# 小端文件头固定 16 字节，记录固定 80 字节，x 表示跳过保留字节。
HEADER = struct.Struct("<8sHHI")
RECORD = struct.Struct("<QI fBB2x Iqqi fff QBBBBB3x II")
FIELDS = (
    "frame_time_us", "air_offset_us", "diff_pressure_pa", "air_node_id", "air_flags",
    "gps_offset_us", "latitude_e8", "longitude_e8", "alt_msl_mm", "vel_n", "vel_e",
    "vel_d", "gnss_time_us", "gps_node_id", "fix_status", "sats_used",
    "gnss_time_standard", "gps_flags", "sequence", "crc32",
)


def read_header(source):
    """检查 magic、格式版本、记录大小和采样周期，成功后文件位置停在第一帧。"""
    raw = source.read(HEADER.size)
    if len(raw) != HEADER.size:
        raise ValueError(f"incomplete file header ({len(raw)} bytes)")
    magic, version, size, period = HEADER.unpack(raw)
    if magic != MAGIC:
        raise ValueError(f"unexpected magic {magic!r}")
    if (version, size, period) != (VERSION, RECORD.size, PERIOD_US):
        raise ValueError(f"unsupported header (version={version}, size={size}, period_us={period})")


def export_records(source, output, diagnostics):
    """逐帧校验并导出，异常诊断与 CSV 分别写入独立输出流。"""
    writer = csv.DictWriter(output, fieldnames=FIELDS, lineterminator="\n")
    writer.writeheader()
    offset = HEADER.size
    previous = None
    pending_bad = 0
    while raw := source.read(RECORD.size):
        if len(raw) != RECORD.size:
            print(f"offset {offset}: ignored {len(raw)} trailing bytes", file=diagnostics)
            break
        # 按固定帧边界前进，损坏整帧跳过，末尾不完整记录忽略并报告长度。
        values = RECORD.unpack(raw)
        crc = zlib.crc32(raw[:-4])
        if crc != values[-1]:
            print(f"offset {offset}: CRC mismatch (expected {crc:08X}, found {values[-1]:08X})",
                  file=diagnostics)
            pending_bad += 1
            offset += RECORD.size
            continue

        row = dict(zip(FIELDS, values))
        sequence = row["sequence"]
        time_us = row["frame_time_us"]
        if previous is not None:
            prev_seq, prev_time_us = previous
            # 序号按 uint32 回绕计算，同时核对时间差是否与窗口数量一致。
            delta = (sequence - prev_seq) & 0xFFFFFFFF
            if delta == 0 or time_us <= prev_time_us:
                print(f"offset {offset}: non-increasing time or repeated sequence ({prev_seq} -> {sequence})",
                      file=diagnostics)
            elif time_us - prev_time_us != delta * PERIOD_US:
                print(f"offset {offset}: timestamp discontinuity for sequence {prev_seq} -> {sequence}",
                      file=diagnostics)
            if 1 < delta < 0x80000000:
                print(f"offset {offset}: sequence gap {prev_seq} -> {sequence} ({delta - 1} records)",
                      file=diagnostics)
        elif pending_bad:
            print(f"offset {offset}: first valid sequence {sequence} follows {pending_bad} corrupt records",
                  file=diagnostics)
        previous = sequence, time_us
        offset += RECORD.size

        # 有效位决定 CSV 是否留空，合法的 0 偏移、0 压差和负压差均保留。
        air_flags = row["air_flags"]
        gps_flags = row["gps_flags"]
        if not air_flags & 1:
            row["air_offset_us"] = row["air_node_id"] = None
        if air_flags & 3 != 3:
            row["diff_pressure_pa"] = None
        if not gps_flags & 1:
            for field in ("gps_offset_us", "gps_node_id", "fix_status", "sats_used", "gnss_time_standard"):
                row[field] = None
        if gps_flags & 3 != 3:
            row["latitude_e8"] = row["longitude_e8"] = None
        if gps_flags & 3 != 3 or row["fix_status"] != 3:
            row["alt_msl_mm"] = None
        if gps_flags & 5 != 5:
            row["gnss_time_us"] = None
        if gps_flags & 9 != 9:
            row["vel_n"] = row["vel_e"] = row["vel_d"] = None
        row["crc32"] = f"{row['crc32']:08X}"
        writer.writerow(row)


def main(argv=None):
    """命令行入口，先验证输入文件头，再创建 CSV 输出，诊断信息写到 stderr。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="FDAULOG v1 .BIN file")
    parser.add_argument("-o", "--output", type=Path, help="CSV path (defaults to stdout)")
    args = parser.parse_args(argv)
    try:
        with args.input.open("rb") as source:
            read_header(source)
            if args.output is None:
                export_records(source, sys.stdout, sys.stderr)
            else:
                if args.output.exists() and args.input.samefile(args.output):
                    raise ValueError("input and output refer to the same file")
                with args.output.open("w", encoding="utf-8", newline="") as output:
                    export_records(source, output, sys.stderr)
    except (OSError, ValueError) as error:
        parser.exit(1, f"{parser.prog}: {error}\n")


if __name__ == "__main__":
    main()
