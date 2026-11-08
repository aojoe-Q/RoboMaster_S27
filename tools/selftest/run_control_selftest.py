"""Compile the actual output/DM/BSP sources against deterministic HAL fakes."""
from pathlib import Path
import shutil
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
BUILD = HERE / "build" / "control"
BUILD.mkdir(parents=True, exist_ok=True)
for name in ("Moter_task.c", "Moter_task.h", "DM_motor.c", "DM_motor.h",
             "arm_output.h", "can_health.h", "motor_encoding.h", "arm_motion.h"):
    shutil.copyfile(ROOT / "APP" / name, BUILD / name)
shutil.copyfile(ROOT / "BSP/Src/bsp_can.c", BUILD / "bsp_can.c")
shutil.copyfile(ROOT / "BSP/Inc/bsp_can.h", BUILD / "bsp_can.h")
for name in ("fdcan.h", "can_bsp.h", "FreeRTOS.h", "task.h", "cmsis_os.h",
             "Arm_L_task.h", "Motor.h", "struct_typedef.h", "arm_math.h", "stm32h7xx.h"):
    (BUILD / name).write_text('#include "control_hal.h"\n')
cc = shutil.which("gcc") or r"E:\mingw64\bin\gcc.exe"
exe = BUILD / "control_selftest.exe"
subprocess.run([cc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                "-I", str(BUILD), "-I", str(HERE), str(BUILD / "DM_motor.c"),
                str(BUILD / "Moter_task.c"), str(BUILD / "bsp_can.c"),
                str(HERE / "control_selftest.c"), "-lm", "-o", str(exe)], check=True)
for scenario in ("encoding", "motion", "enable", "retry", "fifo", "bus", "queue"):
    subprocess.run([str(exe), scenario], check=True)
print("CONTROL SELFTEST OK")
