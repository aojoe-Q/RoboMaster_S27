#!/usr/bin/env bash
# F5 的前置任务：在同一个终端按顺序检查、编译、烧录。
set -euo pipefail

flash_project_root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$flash_project_root"
flash_pyocd="${1:-${XDG_DATA_HOME:-${HOME}/.local/share}/robomaster-s27/pyocd-venv/bin/pyocd}"
flash_log="${flash_project_root}/build/Linux-Debug/flash.log"

echo "[1/3] 检查 GDB、pyOCD 和 Cortex-Debug..."
if timeout 20 bash tools/setup-linux-debug.sh --check; then
    :
else
    flash_status=$?
    if (( flash_status == 124 )); then
        echo "调试工具检查超过 20 秒，已停止。请单独运行 Linux: Check Debug Tools 查看输出。" >&2
    fi
    exit "$flash_status"
fi
if [[ ! -x "$flash_pyocd" ]]; then
    echo "配置中的 pyOCD 不可执行：${flash_pyocd}" >&2
    echo "请运行 bash tools/setup-linux-debug.sh 更新本机路径。" >&2
    exit 1
fi
python3 tools/check-powerdebugger.py --pyocd "$flash_pyocd"

echo "[2/3] 编译 Linux-Debug 固件..."
bash cmake/build.sh Debug

echo "[3/3] 连接 PowerDebugger 并烧录 STM32H723..."
echo "pyOCD：${flash_pyocd}"
echo "固件：${flash_project_root}/build/Linux-Debug/S27_TEST_2.elf"
echo "烧录日志：${flash_log}"
flash_status=0
PYTHONUNBUFFERED=1 "$flash_pyocd" load --no-wait --color never \
    -t stm32h723xx -O connect_mode=attach -O reset_type=sysresetreq \
    --format elf "${flash_project_root}/build/Linux-Debug/S27_TEST_2.elf" \
    2>&1 | tee "$flash_log" || flash_status=$?

if (( flash_status != 0 )); then
    echo "烧录失败（退出码 ${flash_status}），已停止调试启动。" >&2
    echo "请运行 Linux: Probe (PowerDebugger) 查看探针是否可访问；检查 USB、供电、无线链路和 USB 权限。" >&2
    exit "$flash_status"
fi

echo "[完成] 烧录命令已成功结束，VS Code 接下来启动 GDB 调试并停在 main()。"
