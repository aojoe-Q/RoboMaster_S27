<#
.SYNOPSIS
    为 RoboMaster S27 固件仓库配置本机工具链路径：写用户环境变量 + 生成 .vscode/settings.json。

.DESCRIPTION
    仓库里的 .vscode/tasks.json、launch.json、c_cpp_properties.json 已经不写死任何本机路径，
    全部通过环境变量引用。换电脑后跑一遍本脚本即可：

        powershell -ExecutionPolicy Bypass -File tools\setup-vscode.ps1

    脚本会自动探测 Keil / ARMCLANG / Python / pyOCD / STM32CubeIDE 工具链 /
    ST-LINK GDB Server / STM32CubeProgrammer，写进当前用户的持久环境变量，
    并据此生成不入库的 .vscode/settings.json（cortex-debug 的路径配置）。

    参数：
        -Check   只检测并打印结果，不写任何东西（用于先看看缺什么）
        -Quiet   少打印细节

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File tools\setup-vscode.ps1 -Check
#>
[CmdletBinding()]
param(
    [switch]$Check,
    [switch]$Quiet
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot

function Write-Head([string]$text) {
    if (-not $Quiet) {
        Write-Host ''
        Write-Host "== $text ==" -ForegroundColor Cyan
    }
}

function Find-FirstFile([string[]]$patterns) {
    foreach ($p in $patterns) {
        if ([string]::IsNullOrWhiteSpace($p)) { continue }
        foreach ($hit in (Get-ChildItem -Path $p -ErrorAction SilentlyContinue)) {
            if ($hit -and -not $hit.PSIsContainer) { return $hit.FullName }
            if ($hit -and $hit.PSIsContainer) {
                $inner = Get-ChildItem -Path $hit.FullName -ErrorAction SilentlyContinue | Select-Object -First 1
                if ($inner) { return $inner.FullName }
            }
        }
    }
    return $null
}

function Get-DriveRoots {
    $roots = @()
    foreach ($d in [char[]]'CDEFGH') {
        if (Test-Path "${d}:\") { $roots += "${d}:\" }
    }
    return $roots
}

# ---------- 1. Keil MDK (UV4.exe) ----------
Write-Head '探测 Keil MDK'
$keilUv4 = $null
if ($env:KEIL_UV4 -and (Test-Path $env:KEIL_UV4)) { $keilUv4 = $env:KEIL_UV4 }
if (-not $keilUv4) {
    $patterns = @()
    foreach ($root in Get-DriveRoots) {
        $patterns += "$root" + 'Keil_v5\UV4\UV4.exe'
        $patterns += "$root" + 'keil5\UV4\UV4.exe'
        $patterns += "$root" + 'Keil\UV4\UV4.exe'
        $patterns += "$root" + 'Program Files\Keil_v5\UV4\UV4.exe'
        $patterns += "$root" + 'Program Files (x86)\Keil_v5\UV4\UV4.exe'
    }
    foreach ($p in $patterns) { if (Test-Path $p) { $keilUv4 = $p; break } }
}
if ($keilUv4) { if (-not $Quiet) { Write-Host "  UV4.exe        : $keilUv4" -ForegroundColor Green } }
else { Write-Host "  UV4.exe        : 未找到（需要装 Keil MDK）" -ForegroundColor Yellow }

# ---------- 2. ARMCLANG（IntelliSense 用） ----------
Write-Head '探测 ARMCLANG'
$armclang = $null
if ($keilUv4) {
    $keilRoot = Split-Path -Parent (Split-Path -Parent $keilUv4)   # ...\<KeilRoot>
    $cand = Join-Path $keilRoot 'ARM\ARMCLANG\bin\armclang.exe'
    if (Test-Path $cand) { $armclang = $cand }
}
if ($armclang) { if (-not $Quiet) { Write-Host "  armclang.exe   : $armclang" -ForegroundColor Green } }
else { Write-Host "  armclang.exe   : 未找到（Keil 里 ARMCLANG 组件是否安装？）" -ForegroundColor Yellow }

# ---------- 3. Python / pyOCD ----------
Write-Head '探测 Python 与 pyOCD'
$python = $null
if ($env:PYTHON -and (Test-Path $env:PYTHON)) { $python = $env:PYTHON }
if (-not $python) {
    $cmd = Get-Command python -ErrorAction SilentlyContinue
    if ($cmd) { $python = $cmd.Source }
}
if (-not $python) {
    $patterns = @()
    $local = Join-Path $env:LOCALAPPDATA 'Programs\Python'
    if (Test-Path $local) {
        $patterns += (Get-ChildItem $local -Directory -Filter 'Python3*' | Sort-Object Name -Descending | ForEach-Object { Join-Path $_.FullName 'python.exe' })
    }
    foreach ($root in Get-DriveRoots) { $patterns += "$root" + 'Python3*\python.exe' }
    foreach ($p in $patterns) { if (Test-Path $p) { $python = $p; break } }
}

$pyocd = $null
if ($python) {
    $pyocdCand = Join-Path (Split-Path -Parent $python) 'Scripts\pyocd.exe'
    if (Test-Path $pyocdCand) { $pyocd = $pyocdCand }
}
if (-not $pyocd) {
    $cmd = Get-Command pyocd -ErrorAction SilentlyContinue
    if ($cmd) { $pyocd = $cmd.Source }
}
if ($python) { if (-not $Quiet) { Write-Host "  python.exe     : $python" -ForegroundColor Green } }
else { Write-Host "  python.exe     : 未找到（需要装 Python 3）" -ForegroundColor Yellow }
if ($pyocd) { if (-not $Quiet) { Write-Host "  pyocd.exe      : $pyocd" -ForegroundColor Green } }
else { Write-Host "  pyocd.exe      : 未找到（pip install pyocd）" -ForegroundColor Yellow }

# ---------- 4. STM32CubeIDE 工具链（gdb / ST-LINK GDB Server） ----------
Write-Head '探测 STM32CubeIDE 工具链'
$ideDirs = @()
foreach ($root in Get-DriveRoots) {
    # 直接放在盘根的 STM32CubeIDE_2.x
    $ideDirs += (Get-ChildItem $root -Directory -Filter 'STM32CubeIDE*' -ErrorAction SilentlyContinue | ForEach-Object { $_.FullName })
    # <盘根>\ST\STM32CubeIDE_2.x
    $stDir = Join-Path $root 'ST'
    if (Test-Path $stDir) {
        $ideDirs += (Get-ChildItem $stDir -Directory -Filter 'STM32CubeIDE*' -ErrorAction SilentlyContinue | ForEach-Object { $_.FullName })
    }
    # <盘根>\<含 STM32/STMicro 字样的目录>\STM32CubeIDE_2.x（例如 E:\STM32 IDE\STM32CubeIDE_2.1.1）
    foreach ($parent in (Get-ChildItem $root -Directory -ErrorAction SilentlyContinue)) {
        if ($parent.Name -match 'STM32|STMicro') {
            $ideDirs += (Get-ChildItem $parent.FullName -Directory -Filter 'STM32CubeIDE*' -ErrorAction SilentlyContinue | ForEach-Object { $_.FullName })
        }
    }
    # Program Files 下的标准安装位置
    $pf = Join-Path $root 'Program Files\STMicroelectronics\STM32Cube'
    if (Test-Path $pf) {
        $ideDirs += (Get-ChildItem $pf -Directory -Filter 'STM32CubeIDE*' -ErrorAction SilentlyContinue | ForEach-Object { $_.FullName })
    }
}
$ideDirs = @($ideDirs | Select-Object -Unique)

$armToolchain = $null
$stlinkGdb = $null
foreach ($ide in $ideDirs) {
    if (-not $armToolchain) {
        $gdb = Get-ChildItem -Path (Join-Path $ide 'STM32CubeIDE\plugins') -Filter 'arm-none-eabi-gdb.exe' -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1
        if (-not $gdb) {
            $gdb = Get-ChildItem -Path (Join-Path $ide 'plugins') -Filter 'arm-none-eabi-gdb.exe' -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1
        }
        if ($gdb) { $armToolchain = Split-Path -Parent $gdb.FullName }
    }
    if (-not $stlinkGdb) {
        $srv = Get-ChildItem -Path $ide -Filter 'ST-LINK_gdbserver.exe' -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($srv) { $stlinkGdb = $srv.FullName }
    }
}
if ($armToolchain) { if (-not $Quiet) { Write-Host "  GNU 工具链 bin : $armToolchain" -ForegroundColor Green } }
else { Write-Host "  GNU 工具链 bin : 未找到（需要 STM32CubeIDE，或用 arm-none-eabi-gdb 所在目录）" -ForegroundColor Yellow }
if ($stlinkGdb) { if (-not $Quiet) { Write-Host "  ST-LINK 服务器 : $stlinkGdb" -ForegroundColor Green } }
else { Write-Host "  ST-LINK 服务器 : 未找到（只有用 ST-Link 才需要）" -ForegroundColor DarkGray }

# ---------- 5. STM32CubeProgrammer ----------
Write-Head '探测 STM32CubeProgrammer'
$cubeProg = $null
$progPatterns = @()
foreach ($root in Get-DriveRoots) {
    $progPatterns += "$root" + 'STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
    $progPatterns += "$root" + 'Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
    $progPatterns += "$root" + 'Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
}
foreach ($p in $progPatterns) { if (Test-Path $p) { $cubeProg = $p; break } }
if ($cubeProg) { if (-not $Quiet) { Write-Host "  Programmer_CLI : $cubeProg" -ForegroundColor Green } }
else { Write-Host "  Programmer_CLI : 未找到（只有用 ST-Link 才需要）" -ForegroundColor DarkGray }

# ---------- 6. clangd（STM32Cube 扩展的补全） ----------
Write-Head '探测 clangd'
$clangd = $null
$cmd = Get-Command clangd -ErrorAction SilentlyContinue
if ($cmd) { $clangd = $cmd.Source }
if (-not $clangd) {
    $guesses = @(
        (Join-Path $env:LOCALAPPDATA 'Programs\Microsoft VS Code\resources\app\node_modules.asar.unpacked\@vscode\ripgrep\bin\clangd.exe')
    )
    foreach ($g in $guesses) { if (Test-Path $g) { $clangd = $g; break } }
}
if ($clangd) { if (-not $Quiet) { Write-Host "  clangd.exe     : $clangd" -ForegroundColor Green } }
else { Write-Host "  clangd.exe     : 未找到（STM32Cube 扩展会提示）" -ForegroundColor DarkGray }

# 如果本机已经有一份 settings.json 且里面的 clangd 路径有效，沿用它，避免每次重跑把已有配置弄丢
$reuseSettingsPath = Join-Path $repoRoot '.vscode\settings.json'
if (-not $clangd -and (Test-Path $reuseSettingsPath)) {
    try {
        $old = Get-Content -LiteralPath $reuseSettingsPath -Raw | ConvertFrom-Json
        $oldClangd = $old.'stm32cube-ide-clangd.path'
        if ($oldClangd -and (Test-Path $oldClangd)) {
            $clangd = $oldClangd
            Write-Host "  clangd.exe     : 沿用已有配置 $clangd" -ForegroundColor Green
        }
    } catch {
        Write-Host '  (旧 settings.json 解析失败，忽略)' -ForegroundColor DarkGray
    }
}

# ---------- 汇总 ----------
$vars = [ordered]@{}
if ($keilUv4)     { $vars['KEIL_UV4']          = $keilUv4 }
if ($python)      { $vars['PYTHON']            = $python }
if ($pyocd)       { $vars['PYOCD']             = $pyocd }
if ($armclang)    { $vars['ARMCLANG']          = $armclang }
if ($armToolchain){ $vars['ARM_TOOLCHAIN_PATH']= $armToolchain }
if ($stlinkGdb)   { $vars['STLINK_GDB']        = $stlinkGdb }
if ($cubeProg)    { $vars['CUBE_PROG']         = $cubeProg }

$missing = @()
if (-not $keilUv4)      { $missing += 'Keil MDK (UV4.exe)' }
if (-not $python)       { $missing += 'Python 3' }
if (-not $pyocd)        { $missing += 'pyOCD (pip install pyocd)' }
if (-not $armToolchain) { $missing += 'GNU 工具链 (STM32CubeIDE)' }

Write-Head '结果'
foreach ($k in $vars.Keys) { Write-Host ("  {0,-18} = {1}" -f $k, $vars[$k]) }
if ($missing.Count -gt 0) {
    Write-Host ''
    Write-Host ('  缺少必需组件: ' + ($missing -join '、')) -ForegroundColor Yellow
    Write-Host '  先装上这些再跑一次；只装了 Keil 的话，编译能用，但烧录/调试不行。' -ForegroundColor Yellow
}

if ($Check) {
    Write-Host ''
    Write-Host '  [-Check 模式] 没有写入任何内容。' -ForegroundColor DarkGray
    if ($missing.Count -gt 0) { exit 1 } else { exit 0 }
}

# ---------- 7. 写用户环境变量 ----------
Write-Head '写入用户环境变量'
foreach ($k in $vars.Keys) {
    [Environment]::SetEnvironmentVariable($k, $vars[$k], 'User')
    Set-Item -Path ("env:" + $k) -Value $vars[$k]
    Write-Host ("  {0,-18} 已写入 (User)" -f $k) -ForegroundColor Green
}

# ---------- 8. 生成 .vscode/settings.json ----------
Write-Head '生成 .vscode/settings.json'
$gdbPath = $null
if ($armToolchain) { $gdbPath = Join-Path $armToolchain 'arm-none-eabi-gdb.exe' }
$settings = [ordered]@{
    'files.encoding'                      = 'utf8'
    'files.associations'                  = @{ '*.h' = 'c' }
    'C_Cpp.intelliSenseEngine'            = 'disabled'
    'stm32cube-ide-clangd.arguments'       = @('--background-index', '--completion-style=detailed', '--header-insertion=never')
    'stm32cube-ide-clangd.detectExtensionConflicts' = $false
    'cortex-debug.armToolchainPrefix'     = 'arm-none-eabi'
    'cortex-debug.variableUseNaturalFormat' = $false
    'cortex-debug.armToolchainPath'       = $armToolchain
    'cortex-debug.gdbPath'                = $gdbPath
    'cortex-debug.pyocdPath'              = $pyocd
    'stm32cube-ide-clangd.path'           = $clangd
}
# 去掉探测不到的项，免得 cortex-debug 报路径不存在
$clean = [ordered]@{}
foreach ($k in $settings.Keys) {
    $v = $settings[$k]
    if ($null -ne $v -and "$v" -ne '') { $clean[$k] = $v }
}

$settingsPath = Join-Path $repoRoot '.vscode\settings.json'
$json = $clean | ConvertTo-Json -Depth 6
[System.IO.File]::WriteAllText($settingsPath, $json + "`n", (New-Object System.Text.UTF8Encoding($false)))
Write-Host "  已写入 $settingsPath" -ForegroundColor Green

Write-Host ''
Write-Host '下一步：' -ForegroundColor Cyan
Write-Host '  1. 完全关掉 VS Code（不是关闭窗口，是退出进程），重新打开本文件夹，环境变量才会生效。'
Write-Host '  2. 快捷键 Ctrl+Shift+B 试一次 "Keil: Build"。'
Write-Host '  3. 任务 "Probe: 列出调试器" 应该能列出 PowerDebugger。'
Write-Host '  4. 换电脑时：git clone 之后跑一遍本脚本即可。'
if ($missing.Count -gt 0) { exit 1 } else { exit 0 }
