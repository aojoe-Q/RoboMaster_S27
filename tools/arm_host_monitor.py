"""Live monitor for the arm USB link (STATE / ACK / FAULT frames).

    python tools/arm_host_monitor.py --list
    python tools/arm_host_monitor.py                    # auto-detect the port
    python tools/arm_host_monitor.py --port COM7 --csv arm_log.csv
    python tools/arm_host_monitor.py --once

Needs pyserial (pip install pyserial). Output is ASCII for cmd.exe.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import arm_usb_protocol as proto  # noqa: E402

try:
    import serial  # type: ignore
    from serial.tools import list_ports  # type: ignore
except ImportError:  # pragma: no cover
    serial = None
    list_ports = None


def list_serial_ports():
    if list_ports is None:
        raise SystemExit("pyserial is missing: pip install pyserial")
    ports = list(list_ports.comports())
    if not ports:
        print("no serial port found")
    for port in ports:
        print("%-10s %-45s %s" % (port.device, port.description or "", port.hwid or ""))
    return ports


def is_mcu_port(port) -> bool:
    text = " ".join(str(x).upper() for x in (port.device, port.description, port.hwid) if x)
    return any(m in text for m in ("STM32", "STMICRO", "0483:", "VIRTUAL COM", "CDC"))


def pick_port(explicit=None) -> str:
    if explicit:
        return explicit
    if list_ports is None:
        raise SystemExit("pyserial is missing: pip install pyserial")
    ports = list(list_ports.comports())
    candidates = [p.device for p in ports if is_mcu_port(p)]
    if len(candidates) == 1:
        return candidates[0]
    if len(ports) == 1:
        return ports[0].device
    if not ports:
        raise SystemExit("no serial port found - is the board plugged in?")
    raise SystemExit("cannot choose a port, use --port. Candidates: "
                     + ", ".join(p.device for p in ports))


def open_port(explicit=None):
    if serial is None:
        raise SystemExit("pyserial is missing: pip install pyserial")
    device = pick_port(explicit)
    try:
        # timeout=0: read() returns at once with whatever has arrived. With a
        # timeout, read(4096) waits for 4096 bytes or the timeout, which caps
        # any loop that also sends at 1 / timeout Hz.
        port = serial.Serial(device, 115200, timeout=0)      # CDC ignores the baud
    except Exception as exc:  # noqa: BLE001
        raise SystemExit("cannot open %s: %s" % (device, exc))
    time.sleep(0.2)
    port.reset_input_buffer()
    print("port %s open" % device)
    return port


def read_available(port, idle_s: float = 0.002) -> bytes:
    """Return the bytes already received; sleep idle_s when there are none."""
    n = port.in_waiting
    if n:
        return port.read(n)
    if idle_s > 0:
        time.sleep(idle_s)
    return b""


def fmt(values, scale=1.0, width=7, prec=3) -> str:
    return " ".join("%s=%+*.*f" % (n, width, prec, v * scale)
                    for n, v in zip(proto.JOINT_NAMES, values))


def fmt_feedback(s: proto.ArmState) -> str:
    parts = []
    for i, name in enumerate(proto.JOINT_NAMES):
        age = s.feedback_age_ms[i]
        if age == proto.FEEDBACK_AGE_NEVER:
            parts.append("%s=never" % name)
        else:
            parts.append("%s=%s%dms" % (name, "" if s.joint_valid(i) else "STALE:", age))
    return " ".join(parts)


def describe_state(s: proto.ArmState, verbose: bool) -> str:
    lines = ["t=%8d ms  mode=%-16s pc=%-8s fault=%s ready=%s"
             % (s.timestamp_ms, proto.mode_name(s.arm_mode), s.system_name,
                proto.fault_names(s.fault_code), "yes" if s.arm_ready else "NO"),
             "  pos  " + fmt(s.position)]
    if not s.all_feedback_valid or verbose:
        lines.append("  fb   " + fmt_feedback(s))
    if verbose:
        lines += ["  tgt  " + fmt(s.target),
                  "  vel  " + fmt(s.velocity),
                  "  tor  " + fmt(s.torque)]
    return "\n".join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", help="serial port, e.g. COM7")
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    ap.add_argument("--csv", metavar="PATH", help="write every STATE frame to CSV")
    ap.add_argument("--health-jsonl", metavar="PATH", help="write diagnostic pages as JSON lines")
    ap.add_argument("--duration", type=float, help="stop after N seconds")
    ap.add_argument("--display-hz", type=float, default=5.0, help="console refresh rate")
    ap.add_argument("--verbose", action="store_true", help="also print target, velocity, torque")
    ap.add_argument("--once", action="store_true", help="print the first STATE and exit")
    args = ap.parse_args(argv)

    if args.list:
        list_serial_ports()
        return 0

    port = open_port(args.port)
    stream = proto.FrameStream()
    writer = None
    csv_file = None
    health_file = open(args.health_jsonl, "w", encoding="utf-8") if args.health_jsonl else None
    if args.csv:
        csv_file = open(args.csv, "w", newline="")
        writer = csv.writer(csv_file)
        writer.writerow(["host_time", "mcu_ms", "arm_mode", "system_state", "fault",
                         "flags", "feedback_valid"]
                        + ["age_" + n for n in proto.JOINT_NAMES]
                        + ["pos_" + n for n in proto.JOINT_NAMES]
                        + ["tgt_" + n for n in proto.JOINT_NAMES]
                        + ["vel_" + n for n in proto.JOINT_NAMES]
                        + ["tor_" + n for n in proto.JOINT_NAMES])

    start = time.monotonic()
    last_print = 0.0
    count = 0
    last_mcu_ms = None
    try:
        while args.duration is None or time.monotonic() - start < args.duration:
            for frame in stream.feed(read_available(port)):
                if frame.type == proto.MSG_STATE:
                    s = proto.decode_state(frame.payload)
                    count += 1
                    if writer:
                        writer.writerow([time.time(), s.timestamp_ms, s.arm_mode, s.system_state,
                                         s.fault_code, s.flags, s.feedback_valid]
                                        + s.feedback_age_ms + s.position + s.target
                                        + s.velocity + s.torque)
                    now = time.monotonic()
                    if args.once or now - last_print >= 1.0 / args.display_hz:
                        rate = ""
                        if last_mcu_ms is not None and now > start:
                            rate = "  (%.1f STATE/s, crc err %d)" % (count / (now - start),
                                                                    stream.crc_errors)
                        print(describe_state(s, args.verbose) + rate)
                        last_print = now
                    last_mcu_ms = s.timestamp_ms
                    if args.once:
                        return 0
                elif frame.type == proto.MSG_ACK:
                    a = proto.decode_ack(frame.payload)
                    print("ACK  %s seq=%d -> %s" % (proto.MSG_NAMES.get(a.acked_type, a.acked_type),
                                                   a.acked_sequence, a.result_name))
                elif frame.type == proto.MSG_DIAGNOSTIC:
                    health = proto.decode_diagnostic(frame.payload)
                    print("CAN HEALTH", health)
                    if health_file:
                        health_file.write(json.dumps({"host_time": time.time(), **health}) + "\n")
                        health_file.flush()
                elif frame.type == proto.MSG_FAULT:
                    f = proto.decode_fault(frame.payload)
                    print("FAULT %s, pc=%s" % (proto.fault_names(f.fault_code),
                                               proto.SYS_NAMES.get(f.system_state)))
    except KeyboardInterrupt:
        pass
    finally:
        port.close()
        if csv_file:
            csv_file.close()
        if health_file:
            health_file.close()

    elapsed = max(time.monotonic() - start, 1e-6)
    print("%d STATE frames in %.1f s (%.1f/s), crc errors %d, resync bytes %d"
          % (count, elapsed, count / elapsed, stream.crc_errors, stream.resync_bytes))
    return 0


if __name__ == "__main__":
    sys.exit(main())
