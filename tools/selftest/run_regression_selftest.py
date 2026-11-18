"""Compile real mode/UART functions with host fakes; no hardware access.

Generated files live in build/regression. Calibration macros and types are read
from the production headers on every run, including the gripper limits.
"""
from pathlib import Path
import re
import shutil
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OUT = HERE / "build/regression"
OUT.mkdir(parents=True, exist_ok=True)
CC = shutil.which("gcc") or r"E:\mingw64\bin\gcc.exe"


def read(path):
    return re.sub(r"/\*.*?\*/|//[^\n]*", "", (ROOT / path).read_text(encoding="utf-8-sig"), flags=re.S)


def function(text, name):
    match = re.search(r"^(?:static\s+)?(?:void|uint8_t)\s+" + name + r"\s*\([^;]*?\)\s*\{", text, re.M)
    assert match, name
    end, depth = match.end(), 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return text[match.start():end]


def typedef(text, name):
    return re.search(r"typedef\s+(?:struct|enum)\s*\{[^}]*\}\s*" + name + r"\s*;", text).group()


def run(name, parts, scenarios):
    source = OUT / (name + ".c")
    source.write_text("\n".join(parts), encoding="utf-8")
    exe = OUT / (name + ".exe")
    subprocess.run([CC, "-std=c99", "-O0", "-Wall", "-Wextra", "-Werror",
                    "-Wno-pointer-to-int-cast", "-I", str(ROOT / "APP"),
                    str(source), "-lm", "-o", str(exe)], check=True)
    for scenario in scenarios:
        subprocess.run([str(exe), scenario], check=True, timeout=20)


arm = read("APP/Arm_L_task.c")
header = read("APP/Arm_L_task.h")
lib = read("APP/Compat/user_lib.c")
lib_header = read("APP/Compat/user_lib.h")
pre, post = (HERE / "arm_mode_selftest.c").read_text(encoding="utf-8").split("/* REAL_FUNCTIONS */")
types = ["#include <stdint.h>\ntypedef float fp32;"]
types += re.findall(r"^#define[ \t]+\w+[ \t]+[^\n]+", header, re.M)
types += [typedef(header, name) for name in ("Arm_mode_t", "DM_contorl_t", "ArmData_t", "Custom_controller_t")]
types += [typedef(lib_header, "ramp_function_source_t")]
parts = types + [pre, function(lib, "arm_data_ramp_init"), function(lib, "arm_ramp_calc")]
parts += [function(arm, name) for name in ("limitPosition", "Arm_Powerless", "Arm_PrepareAndPublish",
          "TAKE_POSE_1", "GetArmMode", "Arm_L_Movement_J1_J4", "Arm_L_Task")]
run("arm_mode", parts + [post], ("bus_hold", "explicit", "loss", "reconnect", "grip_retry",
                                "invalid", "debounce", "unready", "grip_limits"))

uart = read("BSP/Src/bsp_uart.c")
remote_header = read("Components/Device/Inc/Remote_Control.h")
pre, post = (HERE / "uart_rx_selftest.c").read_text(encoding="utf-8").split("/* REAL_FUNCTIONS */")
parts = re.findall(r"^#define[ \t]+SBUS_RX_\w+[ \t]+[^\n]+", remote_header, re.M)
parts += [pre, function(uart, "USART_RxDMA_MultiBuffer_Init"), function(uart, "USER_USART5_RxHandler"), post]
run("uart_rx", parts, ("uart",))
print("REGRESSION SELFTEST OK")
