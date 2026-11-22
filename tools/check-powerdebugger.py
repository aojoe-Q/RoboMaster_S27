#!/usr/bin/env python3
"""Check PowerDebugger visibility and permissions without connecting to the MCU."""

import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--pyocd',
        default=str(Path(os.environ.get('XDG_DATA_HOME', Path.home() / '.local/share'))
                    / 'robomaster-s27/pyocd-venv/bin/pyocd'),
    )
    args = parser.parse_args()
    devices = []
    for device in Path('/sys/bus/usb/devices').glob('*'):
        try:
            if ((device / 'idVendor').read_text().strip(),
                    (device / 'idProduct').read_text().strip()) != ('303a', '40ff'):
                continue
            bus = int((device / 'busnum').read_text())
            address = int((device / 'devnum').read_text())
            serial_path = device / 'serial'
            serial = serial_path.read_text().strip() if serial_path.exists() else ''
            devices.append((serial, Path(f'/dev/bus/usb/{bus:03d}/{address:03d}')))
        except (OSError, ValueError):
            continue

    if not devices:
        print('系统未发现 PowerDebugger USB 设备（303a:40ff）。请检查电脑端 USB 接收器。')
        return 1

    rules = Path(__file__).resolve().parent / 'udev/60-powerdebugger.rules'
    accessible = []
    for serial, node in devices:
        print(f'PowerDebugger：{serial or "无序列号"}，USB 节点：{node}', flush=True)
        if not node.exists():
            print('USB 设备节点在当前环境不可见。请在本机终端运行；容器/远程环境需先提供 USB 设备访问。')
        elif not os.access(node, os.R_OK | os.W_OK):
            print('当前用户没有 USB 读写权限。安装权限规则后拔插 PowerDebugger：')
            print(f'  sudo install -m 644 {shlex.quote(str(rules))} /etc/udev/rules.d/60-powerdebugger.rules')
            print('  sudo udevadm control --reload-rules')
        else:
            accessible.append(serial.lower())
    if not accessible:
        return 1

    try:
        result = subprocess.run(
            [args.pyocd, 'json', '--probes'], capture_output=True,
            text=True, timeout=15,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        print(f'pyOCD 探针检测失败：{error}')
        return 1
    try:
        data = json.loads(result.stdout)
    except ValueError:
        print('pyOCD 没有返回可用的探针信息：')
        print(result.stdout + result.stderr)
        return 1
    if result.returncode != 0 or data.get('status') != 0:
        print('pyOCD 探针检测失败：', data.get('error', result.stderr))
        return 1
    for probe in data.get('boards', []):
        serial = probe.get('unique_id', '')
        if serial.lower() in accessible or (
                not serial and 'PowerDebugger' in probe.get('product_name', '')):
            print(f'pyOCD 已识别：{probe.get("info", "PowerDebugger")}，序列号：{serial}')
            return 0

    print('USB 节点可读写，但 pyOCD 仍未识别 PowerDebugger。请将以下命令的输出用于排查 USB 后端：')
    print(f'  {shlex.quote(args.pyocd)} list -vv')
    return 1


if __name__ == '__main__':
    sys.exit(main())
