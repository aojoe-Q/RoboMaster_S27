#!/usr/bin/env bash
# 安装 pyOCD / Cortex-Debug，探测 GDB，并合并本机 VS Code 配置。
# --configure-only 只配置本机路径；--check 只检查依赖。
set -euo pipefail

debug_project_root="$(cd "$(dirname "$0")/.." && pwd)"
debug_mode="${1:-install}"
case "$debug_mode" in
    install|--configure-only|--check) ;;
    *) echo "用法：bash tools/setup-linux-debug.sh [--configure-only|--check]" >&2; exit 1 ;;
esac

debug_gdb="${S27_GDB_PATH:-}"
if [[ -z "$debug_gdb" ]]; then
    for debug_candidate in arm-none-eabi-gdb gdb-multiarch; do
        if command -v "$debug_candidate" >/dev/null 2>&1; then
            debug_gdb="$(command -v "$debug_candidate")"
            break
        fi
    done
fi
if [[ -z "$debug_gdb" ]]; then
    for debug_candidate in "${HOME}"/.local/share/stm32cube/bundles/gnu-gdb-for-stm32/*/bin/arm-none-eabi-gdb; do
        if [[ -x "$debug_candidate" ]]; then debug_gdb="$debug_candidate"; fi
    done
fi
if [[ -z "$debug_gdb" ]] || ! "$debug_gdb" --version >/dev/null 2>&1; then
    echo "未找到可运行的 ARM GDB。请安装 gdb-multiarch，或设置 S27_GDB_PATH。" >&2
    exit 1
fi

debug_gcc="$(command -v arm-none-eabi-gcc)"
debug_venv="${XDG_DATA_HOME:-${HOME}/.local/share}/robomaster-s27/pyocd-venv"
debug_pyocd="${debug_venv}/bin/pyocd"
if command -v pyocd >/dev/null 2>&1 && pyocd --version >/dev/null 2>&1; then
    debug_pyocd="$(command -v pyocd)"
fi

if [[ "$debug_mode" != "--check" ]]; then
    python3 - "$debug_project_root" "$debug_gdb" "$debug_pyocd" "$debug_gcc" <<'PY'
from pathlib import Path
import json
import sys

project, gdb, pyocd, gcc = sys.argv[1:]
project = Path(project)
roots = [project]
parent_settings = project.parent / '.vscode/settings.json'
if parent_settings.is_file():
    parent = json.loads(parent_settings.read_text())
    source = parent.get('cmake.sourceDirectory', '').replace('${workspaceFolder}', str(project.parent))
    if source and Path(source).resolve() == project:
        roots.append(project.parent)
for root in roots:
    path = root / '.vscode/settings.json'
    settings = json.loads(path.read_text()) if path.is_file() else {}
    settings.update({
        'cortex-debug.gdbPath': gdb,
        'cortex-debug.pyocdPath': pyocd,
        'cortex-debug.armToolchainPath': str(Path(gcc).parent),
        'cortex-debug.armToolchainPrefix': 'arm-none-eabi',
        'cortex-debug.liveWatchRefreshRate': 300,
        'debug.openDebug': 'openOnSessionStart',
        'debug.internalConsoleOptions': 'openOnSessionStart',
        'debug.console.closeOnEnd': False,
        'terminal.integrated.scrollback': 20000,
    })
    path.parent.mkdir(exist_ok=True)
    path.write_text(json.dumps(settings, indent=4, ensure_ascii=False) + '\n')
    print(f'已更新：{path}')
PY
fi

if [[ "$debug_mode" == "install" ]]; then
    if ! "$debug_pyocd" --version >/dev/null 2>&1; then
        # 通过已有的 pip 给虚拟环境安装，兼容系统未安装 ensurepip 的情况。
        python3 -m venv --without-pip "$debug_venv"
        python3 -m pip --python "$debug_venv" install pyocd
    fi
    if ! command -v code >/dev/null 2>&1; then
        echo "请在 VS Code 中安装 Cortex-Debug（marus25.cortex-debug），然后重跑检查。" >&2
        exit 1
    fi
    if ! code --list-extensions | grep -Fxiq 'marus25.cortex-debug'; then
        code --install-extension marus25.cortex-debug
    fi
fi

debug_missing=0
echo "GDB：${debug_gdb}"
if "$debug_pyocd" --version 2>/dev/null; then
    echo "pyOCD：${debug_pyocd}"
else
    echo "缺少 pyOCD：请运行 bash tools/setup-linux-debug.sh"
    debug_missing=1
fi
if command -v code >/dev/null 2>&1 && timeout 10 code --list-extensions | grep -Fxiq 'marus25.cortex-debug'; then
    echo "Cortex-Debug：已安装"
else
    echo "缺少 Cortex-Debug：请运行 code --install-extension marus25.cortex-debug"
    debug_missing=1
fi

if (( debug_missing > 0 )); then exit 1; fi
echo "依赖检查通过。重载 VS Code，选择 PowerDebugger (自动选择系统)，按 F5。"
