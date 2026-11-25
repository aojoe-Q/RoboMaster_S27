#!/usr/bin/env python3
"""Live watch for the global `ArmData` (STM32H723 + PowerDebugger / CMSIS-DAP).

Reads the struct straight out of AXI SRAM through the debug port *while the
firmware keeps running* -- no halt, no reset, no breakpoint.  The probe is
opened in pyOCD "attach" mode, so the core is never stopped.

Usage (normally started by the VSCode task "Live Watch: ArmData (running)"):
    python live_watch.py --elf MDK-ARM/ATS_H7_Template/project.axf
    python live_watch.py --elf ... --rate 50 --csv armdata.csv
    python live_watch.py --elf ... --once          # single snapshot

Ctrl+C stops it; the target keeps running.
"""

import argparse
import csv
import os
import re
import struct
import subprocess
import sys
import time

# ---------------------------------------------------------------- struct layout
# typedef struct { uint8_t ArmMode; DM_contorl_t J1..J6,Grip; fp32 Total_Angle;
#                  fp32 pitch_position; fp32 roll_position; } ArmData_t;
# DM_contorl_t = 12 x fp32 (48 bytes)
JOINTS = ["J1", "J2", "J3", "J4", "J5", "J6", "Grip"]
FIELDS = [
    ("Initial_Total_position", 0),
    ("position", 4),
    ("send_set", 8),
    ("Error", 12),
    ("Now_set_position", 16),
    ("Now_theta", 20),
    ("Angle_Sum_Value", 24),
    ("RC_Sum_Value", 28),
    ("Key_Control_Value", 32),
    ("Kp", 36),
    ("Kd", 40),
    ("torque", 44),
]
JOINT_OFFSET = 4            # ArmMode(1) + 3 padding
JOINT_SIZE = 48
ARM_SIZE = 352             # 0x160, matches `nm -S`
TOTAL_ANGLE_OFF = 340
PITCH_OFF = 344
ROLL_OFF = 348
DEFAULT_ADDR = 0x24004340

ARM_MODES = {
    0: "IDLE", 1: "RESET", 2: "PC", 3: "RC", 4: "KEY", 5: "PC_CTRL",
}


def resolve_symbol(elf_path, symbol="ArmData"):
    """Ask arm-none-eabi-nm for the address; fall back to the default."""
    nm = None
    for cand in ("arm-none-eabi-nm",):
        try:
            subprocess.run([cand, "--version"], capture_output=True, timeout=5)
            nm = cand
            break
        except Exception:
            pass
    if nm is None:
        tc = os.environ.get("ARM_TOOLCHAIN_PATH")
        if tc:
            p = os.path.join(tc, "arm-none-eabi-nm.exe")
            if os.path.exists(p):
                nm = p
    if nm is None or not elf_path or not os.path.exists(elf_path):
        return DEFAULT_ADDR, None
    try:
        out = subprocess.run([nm, "--defined-only", "-S", elf_path],
                             capture_output=True, text=True, timeout=30).stdout
    except Exception:
        return DEFAULT_ADDR, None
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 4 and parts[3] == symbol:
            try:
                return int(parts[0], 16), int(parts[1], 16)
            except ValueError:
                continue
    return DEFAULT_ADDR, None


def unpack_arm(data):
    """Decode one 352-byte ArmData blob."""
    arm_mode = data[0]
    joints = []
    for i in range(len(JOINTS)):
        base = JOINT_OFFSET + i * JOINT_SIZE
        vals = {}
        for name, off in FIELDS:
            vals[name] = struct.unpack_from("<f", data, base + off)[0]
        joints.append(vals)
    total_angle = struct.unpack_from("<f", data, TOTAL_ANGLE_OFF)[0]
    pitch = struct.unpack_from("<f", data, PITCH_OFF)[0]
    roll = struct.unpack_from("<f", data, ROLL_OFF)[0]
    return arm_mode, joints, total_angle, pitch, roll


COL_WIDTH = {"position": 10, "send_set": 10, "RC_Sum_Value": 14,
             "Kp": 8, "Kd": 8, "torque": 8}
FLOAT_COLS = ("position", "send_set", "RC_Sum_Value")


def make_header(cols):
    return "%-6s" % "joint" + "".join(("%*s" % (COL_WIDTH[c], c)) for c in cols)


def fmt_row(label, j, cols):
    cells = []
    for c in cols:
        w = COL_WIDTH[c]
        v = j[c]
        if c in FLOAT_COLS:
            cells.append("%*.*f" % (w, 4, v))
        else:
            cells.append("%*.*f" % (w, 2, v))
    return "%-6s" % label + "".join(cells)


def main():
    ap = argparse.ArgumentParser(description="ArmData live watch (non-intrusive)")
    ap.add_argument("--elf", default=None, help="path to project.axf (for symbol lookup)")
    ap.add_argument("--addr", default=None, help="override ArmData address (hex ok)")
    ap.add_argument("--rate", type=float, default=20.0, help="samples per second (default 20)")
    ap.add_argument("--duration", type=float, default=0.0, help="stop after N seconds (0 = forever)")
    ap.add_argument("--csv", default=None, help="append samples to this CSV file")
    ap.add_argument("--once", action="store_true", help="print one snapshot and exit")
    ap.add_argument("--halt-sample", action="store_true",
                    help="halt/resume around each read (only if non-intrusive read fails)")
    ap.add_argument("--no-color", action="store_true")
    ap.add_argument("--verbose", action="store_true", help="show pyOCD warnings")
    args = ap.parse_args()

    if not args.verbose:
        import logging
        logging.getLogger().setLevel(logging.ERROR)
        logging.getLogger("pyocd").setLevel(logging.ERROR)

    if args.addr:
        addr = int(args.addr, 0)
        size = ARM_SIZE
    else:
        addr, size = resolve_symbol(args.elf)
        size = size or ARM_SIZE

    try:
        from pyocd.core.helpers import ConnectHelper
    except ImportError:
        print("[error] pyocd is not installed for this python: %s" % sys.executable)
        return 2

    try:
        session = ConnectHelper.session_with_chosen_probe(
            target_override="stm32h723xx",
            options={"connect_mode": "attach", "reset_type": "sysresetreq"},
            blocking=False,
        )
    except Exception as exc:  # probe busy / not found
        print("[error] cannot open the debug probe: %s" % exc)
        print("        hints: close any active VSCode debug session first; "
              "the probe can only be owned by one process.")
        return 3
    if session is None:
        print("[error] no CMSIS-DAP probe found. Is PowerDebugger connected? (`pyocd list`)")
        return 3

    cols = ["position", "send_set", "RC_Sum_Value", "Kp", "Kd", "torque"]
    header = make_header(cols)

    csv_file = csv_writer = None
    if args.csv:
        new_file = not os.path.exists(args.csv)
        csv_file = open(args.csv, "a", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_file)
        if new_file:
            cols_csv = ["t_s", "mode"]
            for jn in JOINTS:
                for c in cols:
                    cols_csv.append("%s_%s" % (jn, c))
            csv_writer.writerow(cols_csv)
            csv_file.flush()

    with session:
        target = session.target
        period = 1.0 / max(args.rate, 0.5)
        t0 = time.time()
        last_print = 0.0
        samples = 0
        errors = 0
        try:
            while True:
                try:
                    if args.halt_sample:
                        target.halt()
                        try:
                            data = target.read_memory_block8(addr, size)
                        finally:
                            target.resume()
                    else:
                        data = target.read_memory_block8(addr, size)
                    arm_mode, joints, total_angle, pitch, roll = unpack_arm(bytes(data))
                except Exception as exc:
                    errors += 1
                    if errors == 1:
                        print("[warn] read failed (%s); retrying..." % exc)
                    if errors > 20:
                        print("[error] too many read errors, aborting.")
                        break
                    time.sleep(period)
                    continue
                samples += 1
                now = time.time()
                elapsed = now - t0

                if csv_writer is not None:
                    row = ["%.3f" % elapsed, arm_mode]
                    for j in joints:
                        for c in cols:
                            row.append("%.6f" % j[c])
                    csv_writer.writerow(row)
                    csv_file.flush()

                if args.once:
                    print("ArmData @ 0x%08X  mode=%d" % (addr, arm_mode))
                    print(header)
                    for name, j in zip(JOINTS, joints):
                        print(fmt_row(name, j, cols))
                    print("Total_Angle=%.4f pitch=%.4f roll=%.4f" % (total_angle, pitch, roll))
                    break

                # refresh the terminal table ~20x/s regardless of sample rate
                if now - last_print >= 0.05:
                    last_print = now
                    out = []
                    out.append("\x1b[H\x1b[2J" if not args.no_color else "\n" * 2)
                    mode_name = ARM_MODES.get(arm_mode, "?")
                    out.append("ArmData @ 0x%08X   mode=%d (%s)   %.1f Hz   t=%.1fs   samples=%d"
                               % (addr, arm_mode, mode_name, args.rate, elapsed, samples))
                    out.append(header)
                    for name, j in zip(JOINTS, joints):
                        out.append(fmt_row(name, j, cols))
                    out.append("Total_Angle=%.4f  pitch=%.4f  roll=%.4f   [Ctrl+C to stop]"
                               % (total_angle, pitch, roll))
                    sys.stdout.write("\n".join(out) + "\n")
                    sys.stdout.flush()

                if args.duration and elapsed >= args.duration:
                    break
                time.sleep(period)
        except KeyboardInterrupt:
            pass
    if csv_file is not None:
        csv_file.close()
    print("[done] %d samples, target left running." % samples)
    return 0


if __name__ == "__main__":
    sys.exit(main())
