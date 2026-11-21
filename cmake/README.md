# CMake 构建说明（arm-none-eabi-gcc）

本目录把原来的 Keil MDK 工程（`MDK-ARM/project.uvprojx`）翻译成 CMake 工程，
不依赖 Keil、可在 Windows / Linux / macOS 上编译，便于以后在 Linux 上开发。

```
S27_TEST_2/
├── CMakeLists.txt                    # 主构建脚本（源文件/头文件/宏/链接选项）
├── CMakePresets.json                 # 预设：Debug / Release（Ninja）
├── STM32H723VGTX_FLASH.ld            # 链接脚本（由 COD_H7_Template.sct 翻译）
├── Core/Startup/startup_stm32h723xx.s# GCC 语法启动文件（比官方模板多了 .axi_data 拷贝）
└── cmake/
    ├── gcc-arm-none-eabi.cmake       # 工具链文件（自动定位 arm-none-eabi-gcc）
    ├── build.ps1                     # Windows 一键构建
    └── build.sh                      # Linux / Git-Bash 一键构建
```

## 一、构建

**Windows（推荐）**

```powershell
.\cmake\build.ps1              # Debug（-Og -g3）
.\cmake\build.ps1 Release      # Release（-Os）
.\cmake\build.ps1 -Clean       # 删掉 build 目录重新来
```

脚本会依次在 PATH、`ARM_TOOLCHAIN_PATH`、STM32CubeIDE 安装目录（`E:\STM32 IDE` 等）
里找 `arm-none-eabi-gcc.exe` / `ninja.exe` / `cmake.exe`，找到后自动设好环境变量。
CubeIDE 自带的那套 GCC 14.3.1 就够用，不需要另装 ARM GCC。

**Linux / macOS**

```bash
sudo apt install cmake ninja-build gcc-arm-none-eabi
./cmake/build.sh              # 或 ./cmake/build.sh Release
```

**手动**

```bash
cmake --preset Debug -DARM_TOOLCHAIN_PATH=/path/to/gcc/bin
cmake --build --preset Debug
```

产物在 `build/Debug/`：`S27_TEST_2.elf` / `.hex` / `.bin` / `.map`（已被 .gitignore 忽略）。

## 二、烧录与调试

在 VSCode 里按 `Ctrl+Shift+P` → `Tasks: Run Task`：

| 任务 | 作用 |
| --- | --- |
| `CMake: Build (Debug)` | GCC 编译（不烧录） |
| `CMake: Build (Release)` | Release 编译 |
| `Flash: CMake 产物 (pyOCD)` | 先 GCC 编译，再用 PowerDebugger 烧录 `build/Debug/S27_TEST_2.elf` |

调试：`Ctrl+Shift+D` 选 **Debug: CMake 产物 (GCC 编译+烧录+调试)** 后按 F5，
会先烧录再停在 `main()`，`Cortex Live Watch` 面板照旧可用。

命令行烧录：

```bash
pyocd flash -t stm32h723xx -O connect_mode=attach --format elf build/Debug/S27_TEST_2.elf
pyocd reset  -t stm32h723xx -O connect_mode=attach -O reset_type=sysresetreq
```

（`-O connect_mode=attach` 不能省，PowerDebugger 无线链路不回报 nRESET 状态。）

## 三、和 Keil 构建的差异（重要）

| 项目 | Keil（ARMCC6） | 本 CMake（GCC 14.3.1） |
| --- | --- | --- |
| 优化 | `-O0` | Debug `-Og`、Release `-Os` |
| 代码大小 | Code 157216 B | FLASH 91956 B（含常量） |
| `.data` / `.bss` | 分摊在 DTCM(0x20000000) + AXI(0x24000000) | 全部在 **DTCM**（97.5 KB / 128 KB） |
| DMA 缓冲（`.AXI_SRAM_DMA`） | AXI SRAM | AXI SRAM `0x24000000`（224 B，每个变量 32 字节对齐） |
| `ArmData` 地址 | `0x24004340` | **`0x200004F4`**（随编译变化，务必用符号名） |
| 数学库 | `arm_cortexM7lfdp_math.lib` | 不链接；工程里没有 `arm_*` 函数调用，仅 `#include "arm_math.h"` |

因此：

* 用 `live_watch.py` 看运行数据时**必须带 `--elf build/Debug/S27_TEST_2.elf`**，
  否则它会退回硬编码的 `0x24004340`（那是 Keil 版固件的地址）。
* GCC 版把 `ArmData` 放在 DTCM，调试器可直接读，`set mem inaccessible-by-default off`
  这条其实可以省（保留也无害，`.AXI_SRAM` 段仍在 AXI）。

## 四、扩展工程

新增源文件：加到 `CMakeLists.txt` 的 `PROJECT_SOURCES`（按模块注释分组）。
新增头文件目录/宏：分别加到 `PROJECT_INCLUDE_DIRS` / `target_compile_definitions`。

当前配置全部来自 `project.uvprojx`：

* 宏：`USE_HAL_DRIVER, STM32H723xx, ARM_MATH_CM7, __FPU_PRESENT=1U, ARM_MATH_CHECK`
* MCU：`-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb`（对应 Keil 的 FPU3=DFPU）
* 共 106 个 `.c` + 1 个启动 `.s`

## 五、已知提示（不影响使用）

* 编译有 21 条 `-Wunused-function` / `-Wunused-variable` / `M_PI redefined` 之类的警告，
  都是原有代码风格问题，Keil 不显示而已；如需安静可把 `-Wall` 去掉。
* FreeRTOS 的 `portable/RVDS/ARM_CM4F/port.c` 其实是 GNU 语法（与 CubeMX 生成的
  `GCC/ARM_CM4F/port.c` 内容一致），所以 GCC 直接用这份，没有另加文件。
* `-Wl,--no-warn-rwx-segments` 会自动探测是否支持（binutils ≥ 2.39），旧工具链自动跳过。

## 六、本机验证记录（2026-11-22，Windows）

用 CubeIDE 自带工具链（GCC 14.3.1 / CMake 3.30.8 / Ninja 1.11.1）实测：

| 指标 | Debug | Release |
| --- | --- | --- |
| 编译单元 | 108 / 108 通过 | 108 / 108 通过 |
| FLASH | 91 956 B (8.77%) | 73 284 B (6.99%) |
| DTCMRAM | 97 512 B (74.40%) | 97 512 B (74.40%) |
| RAM_D1（AXI 的 .axi_sram） | 224 B (0.17%) | 224 B (0.17%) |
| `S27_TEST_2.elf` | 2 947 328 B | 157 676 B |
| `S27_TEST_2.hex` | 258 735 B | 206 220 B |
| `S27_TEST_2.bin` | 91 956 B | 73 284 B |

镜像自检：入口 `0x08015C7D`，`.bin` 前 8 字节 `00 00 02 20 7D 5C 01 08`
→ 初始 SP `0x20020000`（DTCM 顶端）、Reset 向量带 thumb 位，向量表正常。

`powershell -File cmake\build.ps1 -Clean` 从零到出 hex/bin 约 6 秒。

> 尚未在真机上烧录验证。首次烧录会复位并让机械臂按新固件动作，建议确认周边安全后执行
> `Flash: CMake 产物 (pyOCD)`，需回滚时用 `Flash: PowerDebugger (pyOCD)` 重烧 Keil 版 hex。