# VS Code 开发环境配置（换电脑照着做）

> 适用于 `RoboMaster_S27` 仓库的固件工程。
> 目标：**仓库里不存任何本机路径**，换电脑 `git clone` + 跑一遍脚本就能编译、烧录、调试。

---

## 1. 为什么以前换电脑会坏

`.vscode/` 里的四个文件原来写死了本机绝对路径，例如：

```json
"serverpath": "C:/Users/lenovo/AppData/Local/Programs/Python/Python313/Scripts/pyocd.exe"
"armToolchainPath": "E:/STM32 IDE/STM32CubeIDE_2.1.1/STM32CubeIDE/plugins/.../tools/bin"
```

换台电脑，Python 装在别的版本、CubeIDE 装在别的盘，这些路径全废。更麻烦的是 `.vscode/` 整个被 `.gitignore` 忽略，`git clone` 下来根本没有这些文件，`Ctrl+Shift+B` 直接没有任务。

现在的做法分两层：

| 层 | 内容 | 是否入库 |
|---|---|---|
| 结构（已参数化） | `.vscode/tasks.json`、`launch.json`、`c_cpp_properties.json`、`live_watch.py`、`MDK-ARM/build.cmd` | **入库**，路径全部写成 `${env:变量名}` 或读环境变量 |
| 本机路径 | 7 个环境变量 + `.vscode/settings.json` | **不入库**，由 `tools/setup-vscode.ps1` 生成 |

---

## 2. 依赖清单

| 组件 | 必需 | 用途 | 缺失时的现象 |
|---|---|---|---|
| Keil MDK（含 ARMCLANG） | 是 | 编译、生成 `.hex/.axf` | `Keil: Build` 报 UV4.exe not found |
| Python 3 | 是 | 跑 `live_watch.py`、pyOCD | 烧录任务失败 |
| pyOCD（`pip install pyocd`） | 是 | 通过 PowerDebugger 烧录/调试 | `Probe: 列出调试器` 失败 |
| STM32CubeIDE | 是 | 提供 `arm-none-eabi-gdb.exe`（cortex-debug 用） | 调试启动即退出 |
| STM32CubeProgrammer | 否 | ST-Link 烧录任务 | 只用 PowerDebugger 可忽略 |
| VS Code 扩展：Cortex-Debug、STM32Cube for Visual Studio Code、C/C++ | 是 | 调试、补全 | 没有调试配置可选 |
| clangd | 否 | STM32Cube 扩展的索引/补全 | 扩展提示，代码跳转弱一些 |

---

## 3. 新电脑五步

```powershell
# 1) 装好上表的软件（Keil / Python / CubeIDE，Python 记得 pip install pyocd）

# 2) 拉代码
git clone git@github.com:aojoe-Q/RoboMaster_S27.git
cd RoboMaster_S27

# 3) 跑配置脚本（会自动探测路径、写环境变量、生成 .vscode/settings.json）
powershell -ExecutionPolicy Bypass -File tools\setup-vscode.ps1

# 4) 完全退出 VS Code（不是关窗口），再重新打开这个文件夹
#    环境变量只在进程启动时读取，不重启不生效

# 5) 验证
#    Ctrl+Shift+B            -> Keil: Build，应出现 BUILD OK
#    任务: Probe: 列出调试器  -> 应列出 PowerDebugger
```

只想先看看缺什么，加 `-Check`，不会写入任何东西：

```powershell
powershell -ExecutionPolicy Bypass -File tools\setup-vscode.ps1 -Check
```

---

## 4. 脚本做了什么

### 4.1 写 7 个用户级环境变量

| 变量 | 指向 | 谁用 |
|---|---|---|
| `KEIL_UV4` | `...\UV4\UV4.exe` | `MDK-ARM/build.cmd` |
| `ARMCLANG` | `...\ARM\ARMCLANG\bin\armclang.exe` | `c_cpp_properties.json` 的 `compilerPath` |
| `PYTHON` | `...\python.exe` | Live Watch 任务 |
| `PYOCD` | `...\Scripts\pyocd.exe` | 所有烧录/调试任务、`launch.json` |
| `ARM_TOOLCHAIN_PATH` | `...\tools\bin`（含 `arm-none-eabi-gdb.exe`） | `launch.json`、Live Watch |
| `STLINK_GDB` | `...\ST-LINK_gdbserver.exe` | ST-Link 调试配置 |
| `CUBE_PROG` | `STM32_Programmer_CLI.exe` | ST-Link 烧录任务 |

探测顺序是「已有环境变量 → 注册到 PATH 的命令 → 常见安装目录 → 各盘根目录扫描」。
它认这些位置：`C:\Keil_v5`、`D:\keil5`、`E:\keil5`、`E:\STM32 IDE\STM32CubeIDE_*`、`<盘>:\ST\STM32CubeIDE_*`、`Program Files\STMicroelectronics\...`、`<盘>:\STM32CubeProgrammer`。

### 4.2 生成 `.vscode/settings.json`

只放 cortex-debug 和 clangd 的绝对路径（VS Code 的 settings 不做变量替换，只能写实路径），因此这个文件**不入库**。每次跑脚本会重新生成；如果检测不到 clangd，会沿用上一份 `settings.json` 里的路径。

---

## 5. 不想跑脚本：手动配

在 PowerShell 里执行（把路径换成你自己的），然后重启 VS Code：

```powershell
[Environment]::SetEnvironmentVariable('KEIL_UV4','C:\Keil_v5\UV4\UV4.exe','User')
[Environment]::SetEnvironmentVariable('ARMCLANG','C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe','User')
[Environment]::SetEnvironmentVariable('PYTHON','C:\Users\你\AppData\Local\Programs\Python\Python313\python.exe','User')
[Environment]::SetEnvironmentVariable('PYOCD','C:\Users\你\AppData\Local\Programs\Python\Python313\Scripts\pyocd.exe','User')
[Environment]::SetEnvironmentVariable('ARM_TOOLCHAIN_PATH','C:\ST\STM32CubeIDE_1.15.0\STM32CubeIDE\plugins\...\tools\bin','User')
```

再照抄一份 `.vscode/settings.json`，把 `cortex-debug.armToolchainPath` / `gdbPath` / `pyocdPath` 写成实路径即可。

**只编译不烧录的话**，最小集是 `KEIL_UV4`（外加 Keil 装好）。

---

## 6. 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| `Keil: Build` 报 `UV4.exe not found` | 跑一遍脚本；或 `echo $env:KEIL_UV4` 看变量是否为空 |
| 环境变量明明设了，VS Code 里还是空 | VS Code 是旧进程，**完全退出**（任务管理器里没有 `Code.exe`）再开 |
| `UV4 exit code = 11` | 工程已在 Keil GUI 里打开，UV4 命令行无法占用；先关掉 Keil |
| `Probe: 列出调试器` 什么都没有 | PowerDebugger 没插好；或调试会话正占用探针（同时只能一个进程用） |
| 调试启动即退出 | `ARM_TOOLCHAIN_PATH` 里没有 `arm-none-eabi-gdb.exe`，检查 CubeIDE 是否装全 |
| IntelliSense 全是红波浪线 | `ARMCLANG` 路径不对；`c_cpp_properties.json` 里是 `${env:ARMCLANG}`，变量没生效就找不到编译器 |
| Live Watch 报 pyOCD 版本不匹配 | `pip install -U pyocd`，脚本里假定 0.4x |

---

## 7. 相关文件

| 文件 | 作用 |
|---|---|
| `.vscode/tasks.json` | Keil 编译、pyOCD 烧录/擦除/复位、Live Watch、ST-Link 烧录 |
| `.vscode/launch.json` | Cortex-Debug 的 PowerDebugger / ST-Link 调试配置 |
| `.vscode/c_cpp_properties.json` | include 路径、宏定义、编译器（armclang） |
| `.vscode/live_watch.py` | 运行中不停机采样 `ArmData`，可存 CSV |
| `MDK-ARM/build.cmd` | 包装 UV4 命令行编译，屏蔽 GUI 句柄、归一化退出码 |
| `tools/setup-vscode.ps1` | 探测并写环境变量、生成 settings.json |
