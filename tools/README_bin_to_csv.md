# BIN 转 CSV 可执行程序

`bin_to_csv.exe` 是 Windows 64 位独立程序，使用时无需安装 Python。

1. 将 exe 和待转换的 BIN 文件放在同一文件夹。
2. 双击 exe，输入文件名，例如 `LOG0003.BIN`（也可以只输入 `LOG0003`）。
3. 按回车后，在 BIN 所在文件夹生成 `LOG0003.csv`。
4. 窗口显示结果后，按回车退出。

默认查找目录始终是 exe 所在目录。也支持输入带空格的完整路径，或拖入 BIN 文件后按回车；CSV 会输出到该 BIN 所在目录。同名 CSV 会覆盖，请提前保留需要的旧文件。

文件头、CRC、序号和时间连续性的检查复用 `tools/decode_sd_log.py`。CRC 错误的整帧和末尾不完整记录会跳过，诊断信息显示在窗口中；“转换完成”表示导出结束，具体数据问题请查看诊断信息。

从终端运行时也可以直接传入文件名，此时不等待按回车退出：

```powershell
.\bin_to_csv.exe LOG0003.BIN
```

## 重新打包

在项目根目录用 Windows 64 位 Python 运行以下命令。打包依赖和中间文件放在 `.tmp`，最终 exe 放在项目根目录。

```powershell
python -m pip install --target .tmp\bin_to_csv_build_deps pyinstaller==6.22.3
$env:PYTHONPATH = (Resolve-Path .tmp\bin_to_csv_build_deps).Path
python -m PyInstaller --noconfirm --clean --onefile --console --name bin_to_csv --distpath . --workpath .tmp\bin_to_csv_build --specpath .tmp tools\bin_to_csv.py
```

交互入口源码是 `tools/bin_to_csv.py`，原有 `tools/decode_sd_log.py` 命令行用法保持不变。
