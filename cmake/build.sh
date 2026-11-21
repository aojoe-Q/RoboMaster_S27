#!/usr/bin/env bash
# =============================================================================
# 配置并编译 S27_TEST_2 的 CMake 工程（Linux / macOS / WSL / Git-Bash）
#
#   依赖：
#     sudo apt install cmake ninja-build gcc-arm-none-eabi
#     # 或：sudo apt install gcc-arm-none-eabi-multiarch gdb-multiarch
#
#   用法：
#     ./cmake/build.sh              # Debug
#     ./cmake/build.sh Release      # Release
#     ARM_TOOLCHAIN_PATH=/opt/arm-gcc/bin ./cmake/build.sh
# =============================================================================
set -euo pipefail

cd "$(dirname "$0")/.."
PRESET="${1:-Debug}"

if ! command -v arm-none-eabi-gcc >/dev/null 2>&1; then
    if [ -n "${ARM_TOOLCHAIN_PATH:-}" ]; then
        export PATH="${ARM_TOOLCHAIN_PATH}:${PATH}"
    else
        echo "arm-none-eabi-gcc 不在 PATH 中。请安装 gcc-arm-none-eabi，"
        echo "或用 ARM_TOOLCHAIN_PATH=/path/to/bin 指定。"
        exit 1
    fi
fi

EXTRA_ARGS=()
if [ -n "${ARM_TOOLCHAIN_PATH:-}" ]; then
    EXTRA_ARGS+=("-DARM_TOOLCHAIN_PATH=${ARM_TOOLCHAIN_PATH}")
fi

cmake --preset "${PRESET}" "${EXTRA_ARGS[@]}"
cmake --build --preset "${PRESET}"

echo
echo "== 产物 =="
ls -l "build/${PRESET}/S27_TEST_2.elf" "build/${PRESET}/S27_TEST_2.hex" "build/${PRESET}/S27_TEST_2.bin" 2>/dev/null || true
echo
echo "烧录示例（PowerDebugger / CMSIS-DAP）："
echo "  pyocd flash -t stm32h723xx -O connect_mode=attach --format elf build/${PRESET}/S27_TEST_2.elf"
