# Windows prelaunch entry: build with Keil, then flash the matching HEX.
param([string]$PyOcd = $env:PYOCD)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
Set-Location $projectRoot

Write-Host '[1/3] Checking pyOCD...'
if ([string]::IsNullOrWhiteSpace($PyOcd)) {
    $candidate = Get-Command pyocd -ErrorAction SilentlyContinue
    if ($candidate) { $PyOcd = $candidate.Source }
}
if ([string]::IsNullOrWhiteSpace($PyOcd) -or -not (Test-Path -LiteralPath $PyOcd -PathType Leaf)) {
    throw 'pyOCD not found. Run tools\setup-vscode.ps1, then restart VS Code.'
}
& $PyOcd --version
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host '[2/3] Building Keil firmware...'
& (Join-Path $projectRoot 'MDK-ARM\build.cmd') -b
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

$hexFile = Join-Path $projectRoot 'MDK-ARM\COD_H7_Template\project.hex'
if (-not (Test-Path -LiteralPath $hexFile -PathType Leaf)) {
    throw "Build completed without a HEX file: $hexFile"
}
Write-Host '[3/3] Flashing STM32H723 with PowerDebugger...'
& $PyOcd load --no-wait --color never -t stm32h723xx `
    -O connect_mode=attach -O reset_type=sysresetreq --format hex $hexFile
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host '[Done] Flash completed. VS Code will start GDB and stop at main().'
exit 0
