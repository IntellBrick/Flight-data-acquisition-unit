"""供双击运行的 BIN 转 CSV 入口；复用 decode_sd_log 的校验和导出逻辑。"""

import argparse
import sys
from pathlib import Path

from decode_sd_log import export_records, read_header


def default_directory():
    """打包后使用 exe 所在目录，源码运行时使用项目根目录。"""
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent
    return Path(__file__).resolve().parent.parent


def input_path(filename, directory):
    filename = filename.strip()
    if len(filename) >= 2 and filename[0] == filename[-1] == '"':
        filename = filename[1:-1]
    if not filename:
        raise ValueError("未输入文件名。")
    path = Path(filename)
    if not path.suffix:
        path = path.with_suffix(".BIN")
    if path.suffix.lower() != ".bin":
        raise ValueError("请输入 .BIN 文件名。")
    if not path.is_absolute():
        path = directory / path
    return path.resolve()


def convert(path):
    output_path = path.with_suffix(".csv")
    with path.open("rb") as source:
        # 文件头合法后才创建或覆盖 CSV。
        read_header(source)
        with output_path.open("w", encoding="utf-8", newline="") as output:
            export_records(source, output, sys.stderr)
    return output_path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", nargs="?", help="BIN 文件名或完整路径；省略时交互输入")
    args = parser.parse_args(argv)
    interactive = args.input is None
    exit_code = 0
    try:
        directory = default_directory()
        if interactive:
            print("飞行数据 BIN 转 CSV")
            print(f"默认查找目录：{directory}")
            print("CSV 保存到 BIN 所在目录，同名 CSV 会被覆盖。")
            print("CRC 错误或不完整记录会跳过，诊断信息显示在窗口中。\n")
            filename = input("请输入 BIN 文件名（例如 LOG0001.BIN，也可输入完整路径）：")
        else:
            filename = args.input
        path = input_path(filename, directory)
        print(f"输入文件：{path}", flush=True)
        output_path = convert(path)
        print(f"转换完成：{output_path}", flush=True)
    except FileNotFoundError as error:
        print(f"转换失败：找不到文件 {error.filename}", file=sys.stderr)
        exit_code = 1
    except PermissionError as error:
        print(f"转换失败：无法读写文件，请检查权限或关闭正在占用 CSV 的程序。\n{error}",
              file=sys.stderr)
        exit_code = 1
    except (OSError, ValueError) as error:
        print(f"转换失败：{error}", file=sys.stderr)
        exit_code = 1
    except (EOFError, KeyboardInterrupt):
        print("\n已取消转换。")
        exit_code = 1
    if interactive:
        try:
            input("\n按回车键退出……")
        except (EOFError, KeyboardInterrupt):
            pass
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
