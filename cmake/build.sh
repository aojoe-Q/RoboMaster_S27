#!/usr/bin/env bash
# =============================================================================
# 配置并编译 S27_TEST_2 的 CMake 工程（Linux / macOS / WSL / Git-Bash）
#
#   依赖：
#     sudo apt install cmake make gcc-arm-none-eabi libnewlib-arm-none-eabi
#     Linux 使用 Make；其他系统的 Debug / Release 预设使用 Ninja。
#
#   用法：
#     bash cmake/build.sh              # Debug
#     bash cmake/build.sh Release      # Release
#     bash cmake/build.sh Debug --clean-first  # 全量重编译
#     ARM_TOOLCHAIN_PATH=/opt/arm-gcc/bin bash cmake/build.sh
# =============================================================================
set -euo pipefail
# 固定编译器诊断语言，便于 VS Code 和汇总脚本识别 warning / error。
export LC_ALL=C

cd "$(dirname "$0")/.."
build_config="${1:-Debug}"
if (( $# > 0 )); then shift; fi
build_clean_args=()
if (( $# == 1 )) && [[ "$1" == "--clean-first" ]]; then
    build_clean_args=(--clean-first)
    shift
fi
case "$build_config" in
    Debug|Release) ;;
    *) echo "用法：bash cmake/build.sh [Debug|Release] [--clean-first]" >&2; exit 1 ;;
esac
if (( $# > 0 )); then
    echo "用法：bash cmake/build.sh [Debug|Release] [--clean-first]" >&2
    exit 1
fi

build_preset="$build_config"
if [[ "$(uname -s)" == "Linux" ]]; then
    build_preset="Linux-${build_config}"
fi

if [ -n "${ARM_TOOLCHAIN_PATH:-}" ]; then
    export PATH="${ARM_TOOLCHAIN_PATH}:${PATH}"
fi
if ! command -v arm-none-eabi-gcc >/dev/null 2>&1; then
    echo "arm-none-eabi-gcc 不在 PATH 中。请安装 gcc-arm-none-eabi，"
    echo "或用 ARM_TOOLCHAIN_PATH=/path/to/bin 指定。"
    exit 1
fi

build_extra_args=()
if [ -n "${ARM_TOOLCHAIN_PATH:-}" ]; then
    build_extra_args+=("-DARM_TOOLCHAIN_PATH=${ARM_TOOLCHAIN_PATH}")
fi

build_start_seconds=$SECONDS
build_dir="build/${build_preset}"
build_log="${build_dir}/build.log"
mkdir -p "$build_dir"

build_print_summary() {
    local build_exit_status="$1"
    local build_errors build_warnings build_units
    read -r build_errors build_warnings build_units < <(
        awk '
            /^[^[:space:]].*:[[:space:]]+(warning|fatal error|error|Error):([[:space:]]|$)/ {
                if ($0 ~ /:[[:space:]]+warning:/) warnings++
                else errors++
            }
            /^CMake Error([ :]|$)/ { errors++ }
            /^\[[ 0-9%\/]+\] Building (C|CXX|ASM) object / { units++ }
            END { print errors+0, warnings+0, units+0 }
        ' "$build_log"
    )
    {
        echo
        if (( build_exit_status == 0 )); then
            echo "== 编译成功，用时 $(( SECONDS - build_start_seconds )) 秒 =="
        else
            echo "== 构建失败，退出码 ${build_exit_status}，用时 $(( SECONDS - build_start_seconds )) 秒 =="
        fi
        echo "错误：${build_errors}，警告：${build_warnings}（本次编译，${build_units} 个编译单元）"
        if (( ${#build_clean_args[@]} == 0 )); then
            echo "增量编译只统计本次输出的诊断；整个工程请运行 Build: Rebuild (自动选择系统)。"
        fi
        echo "诊断详情：VS Code 问题面板（Ctrl+Shift+M）或 ${PWD}/${build_log}"
    } 2>&1 | tee -a "$build_log"
}

build_status=0
{
    echo "== 编译配置 =="
    arm-none-eabi-gcc --version | head -n 1
    echo "配置：${build_preset}"
    echo "日志：${PWD}/${build_log}"
    if (( ${#build_clean_args[@]} > 0 )); then
        echo "方式：清理后全量编译"
    else
        echo "方式：增量编译（未修改的文件复用已有目标文件）"
    fi
    cmake --preset "${build_preset}" "${build_extra_args[@]}"
} 2>&1 | tee "$build_log" || build_status=$?
if (( build_status != 0 )); then
    build_print_summary "$build_status"
    exit "$build_status"
fi

cmake --build --preset "${build_preset}" "${build_clean_args[@]}" \
    --parallel "${CMAKE_BUILD_PARALLEL_LEVEL:-4}" --verbose 2>&1 | tee -a "$build_log" || build_status=$?
if (( build_status != 0 )); then
    build_print_summary "$build_status"
    exit "$build_status"
fi

{
    echo
    echo "== 代码、初始化数据、未初始化数据占用（字节） =="
    arm-none-eabi-size "${build_dir}/S27_TEST_2.elf"
    echo
    echo "== 固件产物 =="
    ls -lh "${build_dir}/S27_TEST_2.elf" "${build_dir}/S27_TEST_2.hex" "${build_dir}/S27_TEST_2.bin"
    echo
    echo "查看所有源文件编译：bash cmake/build.sh ${build_config} --clean-first"
    echo "编译、烧录并调试：VS Code 中选择 PowerDebugger (自动选择系统)，按 F5"
} 2>&1 | tee -a "$build_log"
build_print_summary 0
