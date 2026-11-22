# RoboMaster S27 电控

辽宁科技大学 COD 战队 S27 赛季工程机器人机械臂电控代码。

- 通过 USB 串口把机械臂电机数据交给上位机，由 ROS 2 + MoveIt2 做路径规划
- 丝杆夹爪采用恒力矩堵转标零
- 控制器：STM32H723VGT6，FreeRTOS，CAN 总线驱动 DJI / LK / HT 三类电机

## 目录结构

| 目录 | 内容 |
|---|---|
| `Core/` | CubeMX 生成的初始化代码（`S27.ioc` 为工程源） |
| `APP/` | 业务层：机械臂任务、电机任务、USB 协议、多圈存储、模式解析 |
| `Components/` | 算法（CRC、卡尔曼、低通、四元数、RLS、斜坡）与设备驱动（BMI088、电机、遥控、裁判系统、图传） |
| `BSP/` | 板级外设（CAN、UART、SPI、ADC、PWM、DWT、GPIO、LED） |
| `Application/Task/` | CAN / Control / Detect / INS 四个任务与 `Config.h` |
| `SystemView/` | SEGGER RTT 调试组件 |
| `Middlewares/` | FreeRTOS、STM32 USB Device、ARM DSP |
| `Drivers/` | ST HAL 与 CMSIS |
| `MDK-ARM/` | Keil 工程 `project.uvprojx`、armasm 启动文件、散列文件 `COD_H7_Template.sct` |
| `Core/Startup/` | GCC 版启动文件（给 CMake 构建用，与 `MDK-ARM/` 那份并存） |
| `cmake/` | 交叉编译工具链文件、`build.ps1` / `build.sh` 一键脚本、构建说明 |
| `tools/selftest/` | 主机侧回归自测（不需要硬件） |
| `docs/` | 配置与说明文档 |

## 编译与烧录

VS Code 使用同一套默认入口，按操作系统自动选择工具链：

| 操作 | Linux | Windows |
| --- | --- | --- |
| `Ctrl+Shift+B`：`Build: Rebuild (自动选择系统)` | ARM GCC + Make 全量编译 | Keil ARMCLANG 全量编译 |
| `F5`：`PowerDebugger (自动选择系统)` | 编译、烧录 `build/Linux-Debug/S27_TEST_2.elf`，进入调试 | 编译、烧录 Keil HEX，以 `project.axf` 进入调试 |
| IntelliSense | 自动选择 `Linux` 配置 | 自动选择 `Win32` 配置 |

每台电脑首次使用仍需安装工具并运行配置脚本：Linux 使用 `bash tools/setup-linux-debug.sh`，
Windows 使用 `powershell -ExecutionPolicy Bypass -File tools\setup-vscode.ps1`。
重载 VS Code 后使用自动入口；旧工作区如保留了手动选择的配置，首次选择一次自动入口。
这些入口保存在项目文件中，可以随 Git 提交；本机路径保存在忽略提交的 `.vscode/settings.json`。

两套构建并存，产物等价，任选一种：

**A. CMake + arm-none-eabi-gcc（不需要 Keil）**

Linux 首次安装依赖并编译 Debug：

```bash
sudo apt install cmake make gcc-arm-none-eabi libnewlib-arm-none-eabi
bash cmake/build.sh
```

Linux Release 使用 `bash cmake/build.sh Release`。生成文件位于 `build/Linux-Debug/` 或
`build/Linux-Release/`。VS Code 按 `Ctrl+Shift+B` 执行 `Build: Rebuild (自动选择系统)`，
在 Linux 上全量编译并显示错误、警告总数。

详细编译输出保存在 `build/Linux-Debug/build.log`；查看所有源文件的编译过程可运行任务
`Linux: Rebuild (Debug)`。烧录并进入调试：先执行 `bash tools/setup-linux-debug.sh`，
重载 VS Code 后选 **PowerDebugger (自动选择系统)**，按 F5。
运行中查看数据使用 **Cortex Live Watch**，添加 `ArmData`。
详细步骤见 [Linux 调试说明](docs/LINUX_DEBUG.md)。

Windows 使用：

```powershell
.\cmake\build.ps1            # 配置 + 编译 Debug，产物在 build/Debug/
.\cmake\build.ps1 Release    # Release 配置（-Os）
```

脚本会自动找 `arm-none-eabi-gcc` / `ninja` / `cmake`（先看 PATH，再看 `ARM_TOOLCHAIN_PATH`，最后找 STM32CubeIDE 自带的那套）。也可以直接用 CMake：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

Windows 产物在 `build/Debug/`；Linux 产物在 `build/Linux-Debug/`。两者都会生成
`S27_TEST_2.elf` / `.hex` / `.bin` / `.map`。`build/` 已在 `.gitignore` 里。

**B. Keil + ARMCLANG**

1. **VS Code（Windows）**：运行任务 `Keil: Build`，走 `MDK-ARM/build.cmd` → `UV4.exe -b`，只编译不烧录。
2. **Keil 界面**：打开 `MDK-ARM/project.uvprojx`，F7 编译。

> 两套构建各自独立：CMake 用 `Core/Startup/startup_stm32h723xx.s`（GCC 语法）+ `STM32H723VGTX_FLASH.ld`；Keil 用 `MDK-ARM/startup_stm32h723xx.s`（armasm）+ `COD_H7_Template.sct`。**加/删源文件时两边都要改**。
> 改了 `.uvprojx`、启动文件或散列文件后，Keil 要**关掉重开**并 Rebuild 全量，增量编译会用到旧 `.o`。

烧录用 pyOCD 或 STM32CubeProgrammer，配置见 `.vscode/tasks.json`（`Flash: CMake 产物` 那条走 pyOCD 烧 `build/Debug/S27_TEST_2.elf`）。

主机自测（gcc 编译真实函数体，不碰硬件）：

```powershell
python tools/selftest/run_regression_selftest.py
```

## 换电脑 / 新环境配置

仓库里不含任何本机绝对路径，`.vscode` 里的工具位置全部走环境变量。新机器：

1. 装好 Keil（含 ARMCLANG）、Python 3.13、`pip install pyocd`、STM32CubeIDE（取其中的 `arm-none-eabi-gcc`）、STM32CubeProgrammer。
2. 在仓库根目录执行 `powershell -File tools/setup-vscode.ps1 -Check` 看缺什么。
3. 去掉 `-Check` 再执行一遍，它会写 7 个用户环境变量并生成本机的 `.vscode/settings.json`。
4. **完全退出 VS Code 再重开**（环境变量不会热加载）。
5. `Ctrl+Shift+B` 验证能编译。

细节见 [docs/VSCODE_SETUP.md](docs/VSCODE_SETUP.md)。注意 `.vscode/settings.json` 是每台机器各自生成、不入库的。

## 分支说明

| 分支 | 指向 | 用途 |
|---|---|---|
| `main` | `09352b8` | 主线，日常开发与上车都用它 |
| `pre-vscode-config` | `e8358c0` | `.vscode` 参数化改造之前的版本，留作退路 |
| `多圈掉电存储与恢复` | `d9497e3` | 多圈掉电存储 / 恢复的独立开发线（本地名 `wip-2026-11-05`） |
| `usb-comm` | `d013e70` | USB 通信线并入主线前的状态 |
| `wip-2026-11-04` | `021f898` | 机械臂控制重构、tests 框架的存档 |
| `fix/20261117-code-review`、`fix/p0-uart-dma-buffer` | `5f524cb`、`4ea35c0` | 已并入 main，仅存档 |

ROS 2 侧代码已独立到 `aojoe-Q/ARM_ROS` 仓库，本仓库的 `ROS2.md` 是指路牌。

## 更新记录

每次改动都在这里加一行（新的在最上面），说明改了什么、要不要重新编译、上车有什么注意。

| 日期 | 提交 | 改了什么 |
|---|---|---|
| 2026-11-22 | - | **按系统选择默认入口**：统一 Ctrl+Shift+B 和 F5，使用 VS Code 系统覆盖配置在 Linux 上调用 GCC/Make、在 Windows 上调用 Keil；IntelliSense 使用原生 Linux/Win32 自动配置名。保留各工具链的手动入口，本机路径继续由对应系统的配置脚本生成。 |
| 2026-11-22 | - | **PowerDebugger 访问诊断**：补充 `303a:40ff` 的 USB/HID udev 权限规则和探针检查脚本；F5 在编译、烧录前检查设备节点及用户权限，区分未连接、节点不可见、权限不足和 pyOCD 未识别。规则需在本机安装后拔插 USB 生效。 |
| 2026-11-22 | - | **F5 前置任务等待修正**：检查、编译、烧录改为一个前台脚本并显示阶段进度；烧录和 GDB Server 增加 `--no-wait`，无探针时立即失败；检查命令增加时限，烧录输出保存到 `flash.log`。已验证失败退出，实际板端调试待验证。 |
| 2026-11-22 | - | **编译诊断统计**：构建成功和失败都打印本次错误、警告总数；`Ctrl+Shift+B` 默认全量重编译，便于每次查看整个工程的诊断；问题面板自动显示 GCC 诊断，F5 烧录前仍执行增量编译。 |
| 2026-11-22 | - | **Linux 编译日志与调试入口**：编译显示完整 GCC 命令并保存 `build.log`；新增全量重编译、调试依赖安装/检查、pyOCD 烧录任务和 Linux Cortex-Debug 配置。F5 串联编译、烧录、停在 `main()`，启用 Cortex Live Watch 查看 `ArmData`。已验证全量编译和 GDB 调试符号，硬件连接待本机验证。 |
| 2026-11-22 | - | **Linux GCC 构建入口**：新增 Make 后端的 Linux Debug/Release CMake 预设，`cmake/build.sh` 在 Linux 上选择 Make，两个 VS Code 工作区都将 `Linux: Build (Debug)` 设为默认任务。修正 Linux 大小写敏感文件系统下的头文件引用。Debug/Release 均通过本机全量编译。 |
| 2026-11-22 | `afca975` | **新增 CMake + arm-none-eabi-gcc 构建**：`CMakeLists.txt`（按 `project.uvprojx` 翻译，含全部 180 余个源文件与头文件路径）、`CMakePresets.json`（Debug `-Og` / Release `-Os`，Ninja 生成器）、`cmake/gcc-arm-none-eabi.cmake`、GCC 语法启动文件 `Core/Startup/startup_stm32h723xx.s`、链接脚本 `STM32H723VGTX_FLASH.ld`、`cmake/build.ps1` 与 `build.sh` 一键脚本、`cmake/README.md`。`.vscode` 同步加三条任务：CMake Build (Debug/Release)、pyOCD 烧录 CMake 产物。**实测通过**：gcc 14.3.1 编译链接无错误，FLASH 91,956 B / DTCM 97,512 B，产出 `build/Debug/S27_TEST_2.elf`、`.hex`、`.bin`。Keil 构建未改动，两套并存。 |
| 2026-11-21 | `09352b8` | **VS Code 配置参数化**：`tasks.json` / `launch.json` / `c_cpp_properties.json` 里的本机绝对路径全换成 `${env:...}`；`build.cmd` 首次入库并支持 `KEIL_UV4` 探测；新增 `tools/setup-vscode.ps1`（探测 7 个工具、写环境变量、生成 settings.json）与 `docs/VSCODE_SETUP.md`；`.gitignore` 重写（原来那份是坏 UTF-8）。换电脑 clone 后跑一次脚本即可。 |
| 2026-11-21 | `e8358c0` | 新增 `PC_MODE.md`：记录 PC 模式三阶段接管的动机、行为对照、自测方法与回滚方式。纯文档。 |
| 2026-11-21 | `1905d94` | 恢复 `84ec9ee` + `f304a09` 的内容（撤销 `be7dc7d` 的回退）。代码内容与 `f304a09` 一致。 |
| 2026-11-21 | `be7dc7d` | 回退 `84ec9ee` 与 `f304a09`（PC 模式回到旧行为）。保留在历史里，实际已被 `1905d94` 恢复。 |
| 2026-11-21 | `f304a09` | 修复回归脚本：同步 `5f524cb` 的接口改名（`TAKE_POSE_1` → `TAKE_pose_1`、`Arm_L_Movement_J1_J4` → `ARM_rc`，补 `TAKE_pose_2` / `ARM_reset`），全量回归重新可跑。 |
| 2026-11-21 | `84ec9ee` | **PC 模式接管改为三阶段状态机**：`PC_READ_ONLY`（首次目标前无力、清零力矩）/ `PC_TRACKING`（首拍重置插补器）/ `PC_HOLDING`（丢目标锁存位置）；新增摇杆映射“右下 + 左上” → `ARM_RC`；PC 模式下夹爪不再叠加力矩；新增 `pc_mode_selftest.c`。**需要 Rebuild，上车注意夹爪改由位置指令驱动。** |
| 2026-11-20 | `b83ff16` | `ROS2.md` 从 149 行改成 23 行指路牌，指向独立仓库 `aojoe-Q/ARM_ROS`；同时从本仓库删除 `ros/ros_test_8` 分支。 |
| 2026-11-18 | `c19a7e3` | 新增 `ROS2.md`：说明两条开发线的分支划分、ROS 侧包结构与编译运行方式。 |
| 2026-11-18 | `5f524cb` | **机械臂动作接口重整**：`TAKE_pose_1` / `TAKE_pose_2` / `ARM_reset` / `ARM_rc` 取代旧命名，模式解析加消抖；删除 40 余份历史文档。**改的是函数名，Keil 必须全量 Rebuild。** |
| 2026-11-17 | `cd64749` | 代码审查修复：CAN、遥控、UART 细节修正；清理历史文档；补模式 / 标零 / 回零 / 回归四套 selftest。 |
| 2026-11-17 | `62fce24` | **P0/P1/P2 修复**：UART DMA 双缓冲长度去掉多余的 `*2`（SBUS 18 / 图传 39 / 裁判帧长）；`Image_Transmission_Info_Update` 增加 Size 参数并补帧长校验；修遥控超时竞态；DMA 段由 `.AXI_SRAM` 改为 `.AXI_SRAM_DMA`（散列文件同步）；注释掉 `Control_Task` / `INS_Task` 释放 4096 字节栈；`Arm_R_task` 移入 `archive/`。 |
| 2026-11-10 | `84f7efc` | 网页上新建 README（5 行简介）。 |
| 2026-11-08 | `2b6bcbf` | **有注释电机使能校验**：新增输出层 `APP/arm_output.h`（250 Hz 发布、`EN_WAIT_BOOT`/`EN_SEND`/`EN_OK`/`EN_FAIL` 状态机、反馈 200 ms 超时），控制层只发布快照不再直接使能；夹爪标零增加“动过 + 顶死 + 仍使能”校验。 |
| 2026-11-07 | `d013e70` | 机械臂夹爪：新增堵转标零（纯力矩顶死点后 `save_motor_zero`）与力矩控制，`Arm_L_task` 流程调整；新增散列文件 `COD_H7_Template.sct`（Sector 6/7 留给掉电存储）。 |
| 2026-11-04 | `0aaf33e` | 新增 USB 通信开发线：`arm_usb_protocol`、`Arm_USB_task`、`ring_buffer`（原在独立分支 `usb-comm`）。 |
| 2026-11-04 | `df6801c` | 同步桌面最新代码：机械臂任务、电机任务、遥控器、`gpio.c` 更新；Keil 工程改名 `S27.uvprojx` → `project.uvprojx`。 |
| 2026-11-04 | `9d77343` | 初始提交：STM32H723VGT6 电控工程，301 个文件，`.gitignore` 排除全部 Keil 编译产物。 |

## 已知问题

- `STM32H723VGTX_FLASH.ld` 目前 FLASH 区按整 1 MB 分配。若日后把「多圈掉电存储与恢复」分支合进 `main`，必须把 Sector 6/7（从 `0x080C0000` 起的 256 KB）从 FLASH 区排除，否则掉电保存的数据会被固件覆盖；Keil 那边的 `COD_H7_Template.sct` 已经是这么做的。
- `APP/Arm_L_task.c:1202` 有一行 GBK 双重编码乱码注释（`d013e70` 引入），全工程仅此一处，不影响编译。
- `LICENSE` 在 `5f524cb` 清理文档时被删，需要时用 `git checkout 62fce24 -- LICENSE` 取回。
