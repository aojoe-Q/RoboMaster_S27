"""Send ENABLE / POSITION_CMD / STOP to the arm over USB, for bench tests.

    python tools/arm_host_control.py --stop                       # send STOP and exit
    python tools/arm_host_control.py --dry-run --joint 1=0.1
    python tools/arm_host_control.py --port COM7 --joint 3=0.3 --duration 5

The tool reads the current STATE first; joints you do not name keep their
current position. It refuses targets more than --max-step rad away from the
measured position unless --force is given, and always sends STOP on exit
(including Ctrl-C). Commands stream at --rate Hz, each valid for --valid-ms.

Note: the firmware only moves the arm once Arm_L_Task consumes
Arm_USB_Get_PC_Target(); before that the STATE frames show pc=RUNNING but the
arm does not follow.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import arm_usb_protocol as proto  # noqa: E402
from arm_host_monitor import open_port, read_available  # noqa: E402


def parse_joint_args(items):
    targets = {}
    for item in items:
        try:
            idx, value = item.split("=", 1)
            idx = int(idx)
            if not 1 <= idx <= proto.JOINT_NUM:
                raise ValueError
            targets[idx - 1] = float(value)
        except ValueError:
            raise SystemExit("bad --joint %r, use N=VALUE with N in 1..%d" % (item, proto.JOINT_NUM))
    return targets


def wait_state(port, stream, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for frame in stream.feed(read_available(port)):
            if frame.type == proto.MSG_STATE:
                return proto.decode_state(frame.payload)
    raise SystemExit("no STATE frame within %.1f s - is the firmware running?" % timeout)


def report(port, stream, last_state, acks=None):
    for frame in stream.feed(read_available(port, idle_s=0)):
        if frame.type == proto.MSG_STATE:
            last_state[0] = proto.decode_state(frame.payload)
        elif frame.type == proto.MSG_ACK:
            a = proto.decode_ack(frame.payload)
            if acks is not None:
                acks.append(a)
            print("ACK  %s seq=%d -> %s" % (proto.MSG_NAMES.get(a.acked_type, a.acked_type),
                                           a.acked_sequence, a.result_name))
        elif frame.type == proto.MSG_FAULT:
            f = proto.decode_fault(frame.payload)
            print("FAULT %s" % proto.fault_names(f.fault_code))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", help="serial port, e.g. COM7")
    ap.add_argument("--pos", help="all 7 targets in rad, comma separated")
    ap.add_argument("--joint", action="append", default=[], metavar="N=RAD",
                    help="target for joint N (1..7), may repeat")
    ap.add_argument("--rate", type=float, default=50.0, help="command rate in Hz (default 50)")
    ap.add_argument("--valid-ms", type=int, default=100, help="validity of each command")
    ap.add_argument("--duration", type=float, default=3.0, help="seconds to stream (default 3)")
    ap.add_argument("--max-step", type=float, default=0.2,
                    help="refuse targets further than this from the measured position (rad)")
    ap.add_argument("--force", action="store_true", help="skip the --max-step check")
    ap.add_argument("--stop", action="store_true", help="send STOP and exit")
    ap.add_argument("--dry-run", action="store_true", help="print the frames, open no port")
    args = ap.parse_args(argv)

    enc = proto.HostEncoder()

    if args.dry_run:
        targets = [0.0] * proto.JOINT_NUM
        if args.pos:
            targets = [float(x) for x in args.pos.split(",")]
        for idx, value in parse_joint_args(args.joint).items():
            targets[idx] = value
        for name, frame in (("ENABLE", enc.enable(True)),
                            ("POSITION_CMD", enc.position(targets, args.valid_ms)),
                            ("STOP", enc.stop())):
            print("%-13s %s" % (name, frame.hex(" ")))
        return 0

    port = open_port(args.port)
    stream = proto.FrameStream()
    try:
        if args.stop:
            port.write(enc.stop())
            time.sleep(0.05)
            report(port, stream, [None])
            return 0

        state = wait_state(port, stream)
        if state.arm_mode != proto.ARM_MODE_PC_CONTROL:
            raise SystemExit("arm_mode is %s, not PC_CONTROL: set both DT7 switches up first"
                             % proto.mode_name(state.arm_mode))
        if not state.arm_ready or not state.all_feedback_valid:
            raise SystemExit("arm not ready (ready=%s, feedback %s) - ENABLE would be refused"
                             % (state.arm_ready, bin(state.feedback_valid)))
        targets = list(state.position)
        if args.pos:
            targets = [float(x) for x in args.pos.split(",")]
            if len(targets) != proto.JOINT_NUM:
                raise SystemExit("--pos needs %d values" % proto.JOINT_NUM)
        for idx, value in parse_joint_args(args.joint).items():
            targets[idx] = value

        for name, now, tgt in zip(proto.JOINT_NAMES, state.position, targets):
            step = abs(tgt - now)
            flag = ""
            if step > args.max_step:
                flag = "  <-- exceeds --max-step %.3f" % args.max_step
            print("%-4s now %+.4f  target %+.4f%s" % (name, now, tgt, flag))
            if step > args.max_step and not args.force:
                raise SystemExit("refusing a %.3f rad step on %s, use --force if intended"
                                 % (step, name))

        enable_ok = False
        for _ in range(3):                      # ENABLE is acked; retry if lost
            acks = []
            port.write(enc.enable(True))
            deadline = time.monotonic() + 0.1
            while time.monotonic() < deadline and not acks:
                report(port, stream, [state], acks)
                time.sleep(0.002)
            if acks and acks[-1].acked_type == proto.MSG_ENABLE:
                enable_ok = acks[-1].result == proto.ACK_OK
                if not enable_ok:
                    raise SystemExit("ENABLE refused: %s" % acks[-1].result_name)
                break
        if not enable_ok:
            raise SystemExit("no ACK for ENABLE")

        period = 1.0 / args.rate
        last = [state]
        sent = 0
        t0 = time.monotonic()
        end_t = t0 + args.duration
        next_send = t0
        while True:
            now = time.monotonic()
            if now >= end_t:
                break
            if now >= next_send:
                port.write(enc.position(targets, args.valid_ms))
                sent += 1
                next_send += period
                if next_send < now:             # fell behind: do not burst
                    next_send = now + period
            report(port, stream, last)
            time.sleep(min(0.001, max(0.0, next_send - time.monotonic())))
        elapsed = time.monotonic() - t0
        print("sent %d POSITION_CMD in %.2f s = %.1f Hz (asked %.1f Hz)"
              % (sent, elapsed, sent / elapsed, args.rate))
        print("last STATE: pc=%s fault=%s" % (last[0].system_name,
                                              proto.fault_names(last[0].fault_code)))
    except KeyboardInterrupt:
        print("interrupted")
    finally:
        try:
            port.write(enc.stop())
            time.sleep(0.05)
        finally:
            port.close()
        print("STOP sent")
    return 0


if __name__ == "__main__":
    sys.exit(main())
