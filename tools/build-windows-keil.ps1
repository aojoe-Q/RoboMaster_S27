# Windows build entry used by the automatic VS Code tasks.
param([switch]$Rebuild)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$buildMode = '-b'
if ($Rebuild) { $buildMode = '-r' }

Write-Host "Windows: Keil ARMCLANG ($buildMode)"
& (Join-Path $projectRoot 'MDK-ARM\build.cmd') $buildMode
exit $LASTEXITCODE
