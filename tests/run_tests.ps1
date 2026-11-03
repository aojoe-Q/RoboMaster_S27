$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent
$buildDir = Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null
Push-Location $repo
try {
    & 'E:\mingw64\bin\gcc.exe' -std=c11 -Wall -Wextra -Werror -O1 -g -Itests/stubs -IAPP -IComponents/Device/Inc -IComponents/Algorithm/Inc tests/test_v8.c APP/arm_gate.c APP/arm_control.c APP/multi_turn_store.c Components/Device/Src/Image_Transmission.c Components/Algorithm/Src/CRC.c -lm -o tests/build/test_v8.exe
    if ($LASTEXITCODE -ne 0) { throw 'Host compilation failed' }
    & tests/build/test_v8.exe
    if ($LASTEXITCODE -ne 0) { throw 'Host tests failed' }
} finally { Pop-Location }
