@echo off
rem Keil MDK command-line build wrapper for VSCode tasks.
rem Usage: build.cmd [-b ^| -r]
rem   -b = incremental build (default), -r = rebuild all
rem Exit codes seen from UV4: 0 ok, 1 warnings, 2 errors, 3 fatal, 11 cannot open project.
rem This wrapper hides UV4's stdio (it is a GUI app and can leave the VSCode
rem task waiting on inherited handles) and normalizes warnings to exit 0 so that
rem preLaunchTask / F5 does not abort on a harmless warning.
rem
rem UV4 location: taken from the KEIL_UV4 environment variable, otherwise probed
rem in the usual install locations. Run tools\setup-vscode.ps1 once per machine
rem to set KEIL_UV4 (and the other toolchain variables) automatically.
setlocal
set "UV=%KEIL_UV4%"
if not defined UV call :probe_uv
if not defined UV (
  echo [build.cmd] ERROR: UV4.exe not found.
  echo [build.cmd]   Run once:  powershell -ExecutionPolicy Bypass -File tools\setup-vscode.ps1
  echo [build.cmd]   Or set:    KEIL_UV4=C:\Keil_v5\UV4\UV4.exe
  exit /b 1
)
set "PROJ=%~dp0project.uvprojx"
set "LOG=%~dp0build.log"
set "MODE=%~1"
if "%MODE%"=="" set "MODE=-b"

if not exist "%UV%" (
  echo [build.cmd] ERROR: UV4.exe not found at %UV%
  echo [build.cmd]   KEIL_UV4 points to a missing file - fix it or run tools\setup-vscode.ps1
  exit /b 1
)
if not exist "%PROJ%" (
  echo [build.cmd] ERROR: project not found at %PROJ%
  exit /b 1
)
if exist "%LOG%" del /q "%LOG%" >nul 2>&1

"%UV%" %MODE% "%PROJ%" -j0 -o "%LOG%" >nul 2>&1
set "RC=%ERRORLEVEL%"

echo [build.cmd] UV4 = %UV%
echo [build.cmd] UV4 exit code = %RC%   ^(0=ok 1=warnings 2=errors 3=fatal 11=cannot-open-project^)
if exist "%LOG%" (
  type "%LOG%"
) else (
  echo [build.cmd] WARNING: no build log produced - is the project already open in the Keil GUI?
)

if %RC% GEQ 2 (
  echo [build.cmd] BUILD FAILED
  exit /b 1
)
echo [build.cmd] BUILD OK
exit /b 0

:probe_uv
for %%D in (C D E F G) do (
  if not defined UV if exist "%%D:\Keil_v5\UV4\UV4.exe" set "UV=%%D:\Keil_v5\UV4\UV4.exe"
  if not defined UV if exist "%%D:\keil5\UV4\UV4.exe" set "UV=%%D:\keil5\UV4\UV4.exe"
  if not defined UV if exist "%%D:\Keil\UV4\UV4.exe" set "UV=%%D:\Keil\UV4\UV4.exe"
  if not defined UV if exist "%%D:\Program Files\Keil_v5\UV4\UV4.exe" set "UV=%%D:\Program Files\Keil_v5\UV4\UV4.exe"
  if not defined UV if exist "%%D:\Program Files (x86)\Keil_v5\UV4\UV4.exe" set "UV=%%D:\Program Files (x86)\Keil_v5\UV4\UV4.exe"
)
goto :eof
