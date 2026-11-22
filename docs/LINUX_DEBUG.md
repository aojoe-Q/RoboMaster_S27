# Linux：显示编译信息、烧录并进入调试

## 1. 首次安装调试工具

默认构建和调试入口会自动选择系统。Linux 使用 ARM GCC + Make 和 Linux-Debug ELF，
Windows 使用 Keil ARMCLANG 和 Keil HEX/AXF。Windows 首次配置使用 `tools/setup-vscode.ps1`，
Linux 按下面步骤配置。

在工程目录执行：

```bash
bash tools/setup-linux-debug.sh
```

脚本安装独立虚拟环境中的 pyOCD、VS Code 的 Cortex-Debug 扩展，探测现有 ARM GDB，
并把本机路径合并到 `.vscode/settings.json`。已有 STM32Cube 扩展提供的 ARM GDB 可以直接用；
没有 ARM GDB 时，先安装 `gdb-multiarch`，或通过 `S27_GDB_PATH` 指定。

此步骤需要网络。安装完运行 **Developer: Reload Window** 重载 VS Code。
“任务: 运行任务”中的 `Linux: Setup Debug Tools` 也会执行同一个安装脚本。

只检查依赖：

```bash
bash tools/setup-linux-debug.sh --check
```

## 2. 查看编译输出

`Ctrl+Shift+B` 执行 `Build: Rebuild (自动选择系统)`，在 Linux 上全量重编译整个工程，终端显示每个源文件的
编译命令、警告和代码/数据大小，末尾显示 `错误：N，警告：M`。失败时也会显示统计并返回失败。
输出同时保存在 `build/Linux-Debug/build.log`。警告按本次实际输出的诊断条数统计，
同一个头文件的警告可能随多个编译单元重复出现。

有诊断时“问题”面板会自动显示，也可以按 `Ctrl+Shift+M` 打开并点击条目定位代码。
该面板按文件和位置合并诊断，条目数可能与终端中的输出条数不同。

需要增量构建时，运行任务 `Linux: Build (Debug)`，它只编译改动过的文件、统计本次诊断。
没有改动时不会重新输出旧警告；F5 烧录前也使用增量编译。命令行全量重编译：

```bash
bash cmake/build.sh Debug --clean-first
```

## 3. F5 编译、烧录、调试

1. 连接 PowerDebugger 和 STM32H723 开发板。
2. 打开“运行和调试”（`Ctrl+Shift+D`），选择 **PowerDebugger (自动选择系统)**。
3. 按 `F5`。VS Code 按顺序检查工具、编译 Debug、用 pyOCD 烧录，随后打开调试页面并停在 `main()`。
4. 再按 `F5` 继续运行。

烧录和 GDB 使用同一个 `build/Linux-Debug/S27_TEST_2.elf`，符号地址与固件对应。
烧录或编译失败时任务会返回失败，排查终端里的错误后再启动调试。

前置任务在一个终端中依次显示 `[1/3] 检查`、`[2/3] 编译`、`[3/3] 烧录`。
工具检查最多等待 20 秒，烧录使用 `--no-wait`：找不到探针时立即报告失败，
不再无限等待。烧录输出保存到 `build/Linux-Debug/flash.log`。

如果旧配置一直显示“正在等待 preLaunchTask”，先取消调试启动，再运行
**Tasks: Terminate Task** 结束旧的烧录及其前置检查/编译任务，
然后执行 **Developer: Reload Window**。再次按 F5，查看同名任务终端中的阶段标记和错误。

如果开发板已经烧录过同一份 ELF，可选 **PowerDebugger Attach (自动选择系统)**，
该入口连接现有程序，不执行编译和烧录。

## 4. 查看机械臂数据

暂停时，在“监视”里添加 `ArmData`，展开 `J1` 到 `J6`、`Grip`，查看实测位置、
目标位置、Kp、Kd 和力矩。也可以添加以下表达式：

| 表达式 | 内容 |
| --- | --- |
| `ArmData.J1.position` | J1 实测位置 |
| `ArmData.J1.send_set` | J1 下发目标位置 |
| `ArmData.ArmMode` | 当前工作模式 |
| `Arm_L_Ready` | 机械臂初始化完成标志 |
| `arm_home_diag` | 回零诊断 |
| `arm_grip_zero_diag` | 夹爪标零诊断 |

程序运行时，在 **Cortex Live Watch** 面板点击 `+`，添加 `ArmData`。
该面板按配置以 4 Hz 采样；普通“监视”窗口在暂停时求值。
在 `main()` 暂停时数据还未初始化，继续运行后才能观察后续更新。

## 5. 检查探针

运行任务 **Linux: Probe (PowerDebugger)**。正常时应能列出 PowerDebugger。
该任务先检查 USB 设备节点和当前用户的读写权限，再让 pyOCD 列出探针。
它不连接、暂停或复位目标 MCU。F5 在编译和烧录前也会执行这个检查。

PowerDebugger 的 USB ID 是 `303a:40ff`。首次使用时，在本机终端安装设备权限规则：

```bash
sudo install -m 644 tools/udev/60-powerdebugger.rules /etc/udev/rules.d/60-powerdebugger.rules
sudo udevadm control --reload-rules
```

然后拔插电脑端 PowerDebugger USB 接收器，再检查：

```bash
python3 tools/check-powerdebugger.py
```

看到 `pyOCD 已识别` 后重新按 F5。规则仅匹配这款设备的 USB 和 HID 接口，
让当前桌面用户及 `plugdev` 组可访问，不需要以 root 启动 VS Code。
如果检测结果是“USB 设备节点在当前环境不可见”，需要在本机终端执行；
容器或远程环境必须先提供 USB 设备访问，权限规则无法补出不可见的设备节点。

本配置沿用 `stm32h723xx`、`connect_mode=attach`、`reset_type=sysresetreq`。

## 验证范围

已在 Linux 上验证全量 Debug 构建、详细输出、日志保存，以及 GDB 读取 ELF 中的
`ArmData` 和诊断结构体。已检测到 pyOCD 和 Cortex-Debug 安装完成。前置脚本的阶段输出、
编译/烧录失败退出及日志保存已验证；USB 访问受限，尚未在此环境验证开发板烧录、
调试连接和实时采样。
