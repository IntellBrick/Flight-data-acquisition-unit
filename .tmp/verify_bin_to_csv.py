import csv
import io
import locale
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(root / "tools"))
from decode_sd_log import HEADER, RECORD, export_records, read_header

test_dir = Path(tempfile.mkdtemp(prefix="bin_to_csv_验收 ", dir=root / ".tmp"))
exe = test_dir / "bin_to_csv.exe"
shutil.copy2(root / "bin_to_csv.exe", exe)
other_cwd = test_dir / "different working directory"
other_cwd.mkdir()
env = os.environ.copy()
env.pop("PYTHONPATH", None)
env.pop("PYTHONHOME", None)
env["PATH"] = str(Path(os.environ["SystemRoot"]) / "System32")
env["PYTHONIOENCODING"] = "utf-8"


def run(*args, user_input=None):
    return subprocess.run([str(exe), *map(str, args)], input=user_input,
                          capture_output=True, text=True, encoding=locale.getencoding(),
                          cwd=other_cwd, env=env, timeout=30)


def reference(path):
    output = io.StringIO(newline="")
    diagnostics = io.StringIO()
    with path.open("rb") as source:
        read_header(source)
        export_records(source, output, diagnostics)
    return output.getvalue()


for name in ("LOG0001.BIN", "LOG0003.BIN"):
    path = test_dir / name
    shutil.copy2(root / name, path)
    # Interactive mode: extension omitted; cwd intentionally differs from exe dir.
    result = run(user_input=path.stem + "\n\n")
    assert result.returncode == 0, (result.stdout, result.stderr)
    assert path.with_suffix(".csv").read_text(encoding="utf-8") == reference(path)
    assert "转换完成" in result.stdout and "按回车键退出" in result.stdout
    rows = list(csv.DictReader(io.StringIO(reference(path))))
    print(f"PASS interactive/default directory/{name}: {len(rows)} rows")

path = test_dir / "中文 空格.BIN"
shutil.copy2(root / "LOG0001.BIN", path)
result = run(user_input=f'"{path}"\n\n')
assert result.returncode == 0, (result.stdout, result.stderr)
assert path.with_suffix(".csv").read_text(encoding="utf-8") == reference(path)
print("PASS quoted absolute path with Chinese/spaces")

# Command-line mode and replacement of an existing CSV.
path.with_suffix(".csv").write_text("old CSV", encoding="utf-8")
result = run(path)
assert result.returncode == 0, (result.stdout, result.stderr)
assert "按回车键退出" not in result.stdout
assert path.with_suffix(".csv").read_text(encoding="utf-8") == reference(path)
print("PASS CLI mode/overwrite")

result = run(user_input="missing.BIN\n\n")
assert result.returncode == 1 and "找不到文件" in result.stderr
assert not (test_dir / "missing.csv").exists()
print("PASS missing file")

bad = test_dir / "invalid.BIN"
bad.write_bytes(b"not a valid BIN file")
bad.with_suffix(".csv").write_bytes(b"preserve existing output")
result = run(bad)
assert result.returncode == 1 and "转换失败" in result.stderr
assert bad.with_suffix(".csv").read_bytes() == b"preserve existing output"
print("PASS invalid header preserves existing CSV")

corrupt = test_dir / "corrupt.BIN"
raw = bytearray((root / "LOG0001.BIN").read_bytes())
raw[HEADER.size + 4] ^= 1
corrupt.write_bytes(raw + b"partial")
result = run(corrupt)
assert result.returncode == 0, (result.stdout, result.stderr)
assert "CRC mismatch" in result.stderr and "trailing bytes" in result.stderr
assert corrupt.with_suffix(".csv").read_text(encoding="utf-8") == reference(corrupt)
print("PASS CRC and partial record diagnostics")

result = run(user_input="\n\n")
assert result.returncode == 1 and "未输入文件名" in result.stderr
print("PASS empty input")
print(f"All 8 checks passed; isolated artifacts: {test_dir}")
