"""Measure the POSITION_CMD rate of arm_host_control.py without hardware.

FakePort behaves like pyserial for read()/in_waiting/timeout: bytes arrive
at the 100 Hz STATE rate, and read(n) with a timeout blocks until n bytes or
the timeout, as pyserial does. The real control loop runs against it.

Usage:  python tools/selftest/host_rate_check.py
"""

from __future__ import annotations

import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import arm_usb_protocol as proto  # noqa: E402
import arm_host_control  # noqa: E402


def state_frame(seq, system_state=proto.SYS_IDLE):
    payload = proto.encode_state(seq * 10, proto.ARM_MODE_PC_CONTROL, system_state, 0, proto.STATE_FLAG_ARM_READY,
                                 0x7F, [1] * 7, [100000] * 7, [0] * 7, [0] * 7, [100000] * 7)
    return proto.build_frame(proto.MSG_STATE, payload, seq & 0xFF, seq * 10)


class FakePort:
    def __init__(self, timeout):
        self.timeout = timeout
        self._buf = bytearray()
        self._cv = threading.Condition()
        self._stop = False
        self.commands = 0
        self._stream = proto.FrameStream()
        threading.Thread(target=self._mcu, daemon=True).start()

    def _mcu(self):
        seq = 0
        enabled = False
        while not self._stop:
            time.sleep(0.01)
            with self._cv:
                self._buf += state_frame(seq, proto.SYS_RUNNING if enabled else proto.SYS_IDLE)
                seq += 1
                self._cv.notify_all()

    @property
    def in_waiting(self):
        with self._cv:
            return len(self._buf)

    def read(self, n=1):
        deadline = None if self.timeout is None else time.monotonic() + self.timeout
        with self._cv:
            while len(self._buf) < n and self.timeout != 0:
                left = deadline - time.monotonic()
                if left <= 0:
                    break
                self._cv.wait(left)
            out = bytes(self._buf[:n])
            del self._buf[:n]
            return out

    def write(self, data):
        for f in self._stream.feed(data):
            if f.type == proto.MSG_POSITION_CMD:
                self.commands += 1
            elif f.type == proto.MSG_ENABLE:
                ack = proto.build_frame(proto.MSG_ACK, bytes([proto.MSG_ENABLE, f.sequence, 0, 0]))
                with self._cv:
                    self._buf += ack
                    self._cv.notify_all()
        return len(data)

    def reset_input_buffer(self):
        with self._cv:
            self._buf.clear()

    def close(self):
        self._stop = True


def run(timeout, rate, duration=2.0):
    port = FakePort(timeout)
    arm_host_control.open_port = lambda _p: port
    t0 = time.monotonic()
    arm_host_control.main(["--rate", str(rate), "--duration", str(duration), "--force"])
    return port.commands / (time.monotonic() - t0)


def old_style_rate(rate, duration=2.0):
    """The previous loop: read(4096) on a timeout=0.05 port, then sleep."""
    port = FakePort(0.05)
    n = 0
    t0 = time.monotonic()
    next_send = t0
    while time.monotonic() - t0 < duration:
        if time.monotonic() >= next_send:
            port.write(proto.HostEncoder().position([0.1] * 7))
            n += 1
            next_send += 1.0 / rate
        port.read(4096)
        time.sleep(0.002)
    port.close()
    return n / (time.monotonic() - t0)


def main():
    import contextlib
    import io

    print("old loop (read(4096), timeout 0.05): %.1f Hz at --rate 50" % old_style_rate(50))
    ok = True
    for rate in (50, 100):
        with contextlib.redirect_stdout(io.StringIO()):
            got = run(0, rate)
        good = got >= 0.9 * rate
        ok &= good
        print("new loop --rate %3d: %.1f Hz %s" % (rate, got, "ok" if good else "TOO SLOW"))
    print("HOST RATE OK" if ok else "HOST RATE FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
