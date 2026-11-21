<#
.SYNOPSIS
    配置并编译 S27_TEST_2 的 CMake 工程（Windows / PowerShell）。

.DESCRIPTION
    自动寻找 arm-none-eabi-gcc、ninja、cmake：
      1) 先看 PATH 里有没有；
      2) 再看环境变量 ARM_TOOLCHAIN_PATH 指向的目录；
      3) 最后在 STM32CubeIDE 安装目录里找它自带的插件工具链
         （com.st.stm32cube.ide.mcu.externaltools.*）。

.EXAMPLE
    .\cmake\build.ps1                     # 配置 + 编译 Debug
    .\cmake\build.ps1 Release             # 配置 + 编译 Release
    .\cmake\build.ps1 -Clean              # 删掉 build 目录后重新配置编译
    .\cmake\build.ps1 -ConfigureOnly      # 只生成构建文件，不编译
#>
param(
    [ValidateSet('Debug', 'Release')]
    [string]$Preset = 'Debug',
    [switch]$Clean,
    [switch]$ConfigureOnly
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot          # 工程根目录
Set-Location $root

function Find-Exe {
    param([string]$Name, [string[]]$SearchRoots)
    $cmd = Get-Command $Name -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    foreach ($r in $SearchRoots) {
        if (-not (Test-Path $r)) { continue }
        $hit = Get-ChildItem -Path $r -Filter $Name -Recurse -File -Depth 6 -ErrorAction SilentlyContinue |
               Select-Object -First 1
        if ($hit) { return $hit.FullName }
    }
    return $null
}

$ideRoots = @('E:\STM32 IDE', 'C:\ST', 'D:\ST', 'C:\Program Files\STMicroelectronics')
if ($env:ARM_TOOLCHAIN_PATH) { $ideRoots = @($env:ARM_TOOLCHAIN_PATH) + $ideRoots }

$gcc   = Find-Exe -Name 'arm-none-eabi-gcc.exe' -SearchRoots $ideRoots
$ninja = Find-Exe -Name 'ninja.exe'             -SearchRoots $ideRoots
$cmake = Find-Exe -Name 'cmake.exe'             -SearchRoots $ideRoots

if (-not $gcc)   { throw ' 未找到 arm-none-eabi-gcc.exe，请安装 ARMGCC 或设置 ARM_TOOLCHAIN_PATH' }
if (-not $cmake) { throw ' 未找到 cmake.exe，请安装 CMake 并加入 PATH' }
if (-not $ninja) { throw ' 未找到 ninja.exe，请安装 Ninja 并加入 PATH（或用 CubeIDE 自带的）' }

$gccBin   = Split-Path -Parent $gcc
$ninjaBin = Split-Path -Parent $ninja
$env:PATH = "$gccBin;$ninjaBin;$env:PATH"
$env:ARM_TOOLCHAIN_PATH = $gccBin

Write-Host "== toolchain =="
Write-Host "  gcc   : $gcc"
Write-Host "  ninja : $ninja"
Write-Host "  cmake : $cmake"

if ($Clean) {
    $bd = Join-Path $root "build\$Preset"
    if (Test-Path $bd) { Remove-Item -Recurse -Force $bd; Write-Host "  已删除 $bd" }
}

& $cmake --preset $Preset "-DARM_TOOLCHAIN_PATH=$gccBin"
if ($LASTEXITCODE -ne 0) { throw "cmake 配置失败（exit $LASTEXITCODE）" }

if (-not $ConfigureOnly) {
    & $cmake --build (Join-Path $root "build\$Preset")
    if ($LASTEXITCODE -ne 0) { throw "编译失败（exit $LASTEXITCODE）" }
    $out = Join-Path $root "build\$Preset"
    Write-Host "`n== 产物 =="
    Get-ChildItem $out -File | Where-Object { $_.Extension -in '.elf', '.hex', '.bin' } |
        ForEach-Object { Write-Host ("  {0,-24} {1,10:N0} B" -f $_.Name, $_.Length) }
}
