"""Host self-test of the firmware USB link against tools/arm_usb_protocol.py.

1. Compiles APP/Arm_USB_task.c, arm_usb_protocol.c and ring_buffer.c with
   gcc against stub/ (fake clock, fake USB) and runs usb_selftest.c.
2. Decodes the STATE frames the firmware produced and re-encodes them in
   Python: both sides must give the same bytes.
3. Feeds frames built by Python into the firmware and checks its answers.
4. Checks the joint limits in stub/Arm_L_task.h still match APP/Arm_L_task.h.

Nothing runs on the STM32; this does not replace a Keil build.

Usage:  python tools/selftest/run_selftest.py
"""

from __future__ import annotations

import os
import re
import shutil
import struct
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
PROJECT = os.path.dirname(TOOLS)
APP = os.path.join(PROJECT, "APP")
MINIPC_INC = os.path.join(PROJECT, "Components", "Device", "Inc")
STUB = os.path.join(HERE, "stub")
BUILD = os.environ.get("ARM_SELFTEST_BUILD") or os.path.join(HERE, "build")
FIRMWARE = ("Arm_USB_task.c", "Arm_USB_task.h", "arm_usb_protocol.c",
            "arm_usb_protocol.h", "ring_buffer.c", "ring_buffer.h")

sys.path.insert(0, TOOLS)
import arm_usb_protocol as proto  # noqa: E402


class Failure(Exception):
    pass


def check(cond, message):
    if not cond:
        raise Failure(message)


def find_compiler():
    for cand in (os.environ.get("CC"), "gcc", r"E:\mingw64\bin\gcc.exe", "cc", "clang"):
        if not cand:
            continue
        path = shutil.which(cand) or (cand if os.path.isfile(cand) else None)
        if path:
            return path
    raise Failure("no C compiler found (set CC or install gcc)")


def check_limits():
    pattern = re.compile(r"#define\s+((?:J\d|Grip)_(?:Max|Min)_position)\s+(-?[0-9.]+)f?")

    def read(path):
        with open(path, "rb") as f:
            text = f.read().decode("latin-1")
        return {name: float(value) for name, value in pattern.findall(text)}

    real = read(os.path.join(APP, "Arm_L_task.h"))
    stub = read(os.path.join(STUB, "Arm_L_task.h"))
    check(len(real) == 14, "expected 14 joint limits in APP/Arm_L_task.h, found %d" % len(real))
    check(real == stub, "stub joint limits differ from APP/Arm_L_task.h: %s"
          % sorted(set(real.items()) ^ set(stub.items())))
    print("joint limits in the stub match APP/Arm_L_task.h")

    # ARM_MODE_PC_CONTROL position in the real Arm_mode_t enum
    with open(os.path.join(APP, "Arm_L_task.h"), "rb") as f:
        text = f.read().decode("latin-1")
    body = text[text.index("typedef enum"):text.index("}Arm_mode_t")]
    names = re.findall(r"^\s*([A-Z][A-Za-z0-9_]+)\s*,", body, re.M)
    check("ARM_MODE_PC_CONTROL" in names and names.index("ARM_MODE_PC_CONTROL") == 10,
          "ARM_MODE_PC_CONTROL is not entry 10 of Arm_mode_t: %s" % names)
    check(len(proto.ARM_MODE_NAMES) == len(names),
          "python ARM_MODE_NAMES has %d entries, Arm_mode_t %d"
          % (len(proto.ARM_MODE_NAMES), len(names)))
    print("Arm_mode_t: PC_CONTROL = 10, python mode table has the same length")
    return real


def build(compiler):
    os.makedirs(BUILD, exist_ok=True)
    for name in FIRMWARE:
        shutil.copy(os.path.join(APP, name), BUILD)
    exe = os.path.join(BUILD, "usb_selftest.exe")
    cmd = [compiler, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-DARM_USB_SELFTEST",
           "-I", STUB, "-I", BUILD, "-I", MINIPC_INC,
           os.path.join(BUILD, "Arm_USB_task.c"), os.path.join(BUILD, "arm_usb_protocol.c"),
           os.path.join(BUILD, "ring_buffer.c"), os.path.join(HERE, "usb_selftest.c"),
           "-o", exe, "-lm"]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise Failure("compile failed:\n" + result.stdout + result.stderr)
    print("compiled firmware sources with -Wall -Wextra -Werror")
    return exe


def write_host_frames():
    clock = [100.0]
    enc = proto.HostEncoder(clock=lambda: clock[0])
    ok = [0.10, 0.50, 0.60, -1.0, 0.20, -0.30, 0.35]
    bad = list(ok)
    bad[4] = 1.30                                   # J5 max is 1.23497334

    data = enc.enable(True)                         # seq 0 -> ACK OK
    data += b"\x00\x13\xa5\xa5\x37"                  # junk with false SOFs
    first = enc.position(ok, valid_time_ms=300)     # seq 1 -> accepted, no ACK
    data += first
    data += enc.position(bad, valid_time_ms=300)    # seq 2 -> OUT_OF_RANGE
    # seq 3: the first command again (same host timestamp), as a resend would.
    resent = proto.build_frame(proto.MSG_POSITION_CMD, proto.decode_frames(first)[0].payload,
                               enc._next(), 0)
    data += resent                                  # seq 3 -> STALE
    with open(os.path.join(BUILD, "host_frames.bin"), "wb") as f:
        f.write(data)

    # The Python parser must survive the same junk and arbitrary chunking.
    stream = proto.FrameStream()
    frames = []
    for i in range(0, len(data), 3):
        frames += stream.feed(data[i:i + 3])
    check([f.type for f in frames] == [proto.MSG_ENABLE] + [proto.MSG_POSITION_CMD] * 3,
          "python parser did not recover the host frames: %s" % [f.name for f in frames])
    cmds = [proto.decode_position_cmd(f.payload) for f in frames[1:]]
    check(all(abs(a - b) < 1e-6 for a, b in zip(cmds[0].position, ok)), "POSITION_CMD round trip")
    check(cmds[1].command_timestamp_ms > cmds[0].command_timestamp_ms,
          "HostEncoder timestamps increase even with a frozen clock")
    return ok


def run(exe):
    result = subprocess.run([exe, BUILD], capture_output=True, text=True)
    sys.stdout.write(result.stdout)
    if result.returncode != 0:
        raise Failure("C checks failed:\n" + result.stderr)


def f32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


def verify_state_frames():
    with open(os.path.join(BUILD, "dev_state.bin"), "rb") as f:
        raw = f.read()
    frames = proto.decode_frames(raw)
    check(len(frames) == 2 and all(fr.type == proto.MSG_STATE for fr in frames),
          "expected 2 STATE frames, got %s" % [fr.name for fr in frames])
    check(frames[1].sequence == (frames[0].sequence + 1) & 0xFF, "STATE sequence increments")

    rebuilt = b""
    for fr in frames:
        s = proto.decode_state(fr.payload)
        check(s.arm_mode == 8 and s.system_state == proto.SYS_IDLE and s.fault_code == 0,
              "STATE header fields: mode %d state %d fault %d"
              % (s.arm_mode, s.system_state, s.fault_code))
        check(s.arm_ready and s.all_feedback_valid and s.feedback_age_ms == [0] * 7,
              "ready / feedback fields: flags %d valid %s age %s"
              % (s.flags, bin(s.feedback_valid), s.feedback_age_ms))
        check(s.timestamp_ms == fr.timestamp_ms, "payload and header timestamps agree")
        for i in range(proto.JOINT_NUM):
            # position comes from the CAN feedback, not ArmData (set to 99 there)
            exp_pos = f32(f32(0.1 * (i + 1)) - 0.25)
            exp_set = f32(-0.05 * (i + 1))
            exp_vel = f32(1.5 - 0.5 * i)
            exp_tor = -48.5 if i == 6 else f32(0.25 * i - 0.8)
            check(abs(s.position[i] - exp_pos) <= 2e-6, "position J%d %.7f" % (i + 1, s.position[i]))
            check(abs(s.target[i] - exp_set) <= 2e-6, "target J%d" % (i + 1))
            check(abs(s.velocity[i] - exp_vel) <= 2e-6, "velocity J%d" % (i + 1))
            check(abs(s.torque[i] - exp_tor) <= 1e-3, "torque J%d %.3f" % (i + 1, s.torque[i]))
        v = s.raw
        payload = proto.encode_state(v[0], v[1], v[2], v[3], v[4], v[5], v[7:14], v[14:21],
                                     v[21:28], v[28:35], v[35:42])
        rebuilt += proto.build_frame(proto.MSG_STATE, payload, fr.sequence, fr.timestamp_ms)
    check(rebuilt == raw, "python encoder does not reproduce the firmware STATE bytes")
    print("STATE frames decode to the expected values (incl. -48.5 Nm torque); "
          "python re-encodes them byte for byte")


def verify_cross(ok_targets):
    with open(os.path.join(BUILD, "cross_out.bin"), "rb") as f:
        raw = f.read()
    stream = proto.FrameStream()
    frames = stream.feed(raw)
    check(stream.crc_errors == 0 and stream.resync_bytes == 0, "firmware output is clean")

    acks = [proto.decode_ack(fr.payload) for fr in frames if fr.type == proto.MSG_ACK]
    got = [(a.acked_type, a.acked_sequence, a.result) for a in acks]
    want = [(proto.MSG_ENABLE, 0, proto.ACK_OK),
            (proto.MSG_POSITION_CMD, 2, proto.ACK_OUT_OF_RANGE),
            (proto.MSG_POSITION_CMD, 3, proto.ACK_STALE)]
    check(got == want, "ACKs %s, expected %s" % (got, want))

    states = [proto.decode_state(fr.payload) for fr in frames if fr.type == proto.MSG_STATE]
    check(states and states[-1].system_state == proto.SYS_RUNNING,
          "last STATE should be RUNNING, got %s" % (states[-1].system_name if states else None))

    with open(os.path.join(BUILD, "cross_target.txt")) as f:
        fields = f.read().split()
    check(fields[0] == "1", "firmware reports no fresh PC target")
    target = [float(x) for x in fields[1:]]
    check(all(abs(a - b) <= 2e-6 for a, b in zip(target, ok_targets)),
          "PC target %s != %s" % (target, ok_targets))
    print("python-built frames: ENABLE ok, valid command applied, out-of-range and "
          "resent commands rejected")


def main():
    try:
        check(proto.crc16(b"123456789") == 0x29B1, "python crc16 check value")
        compiler = find_compiler()
        print("compiler: %s" % compiler)
        check_limits()
        exe = build(compiler)
        ok = write_host_frames()
        run(exe)
        verify_state_frames()
        verify_cross(ok)
    except Failure as exc:
        print("SELFTEST FAILED: %s" % exc)
        return 1
    print("SELFTEST OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
