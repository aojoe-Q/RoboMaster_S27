"""Host side of the arm USB CDC protocol v2 (mirror of APP/arm_usb_protocol.h).

Frame, all little endian:

    sof u8 (0xA5) | version u8 | type u8 | sequence u8 | payload_len u16 |
    timestamp_ms u32 | payload | crc16 u16

CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF) over everything before the CRC.
Angles travel as int32 micro-radians, torque as int32 milli-newton-metres.
POSITION_CMD.command_timestamp_ms must increase strictly within a USB session:
the MCU rejects older or equal values as STALE. Use a monotonic clock.

No third-party dependency: the ROS 2 hardware interface can reuse this file
or port it to C++ field by field.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

SOF = 0xA5
VERSION = 0x02
HEADER_SIZE = 10
CRC_SIZE = 2
OVERHEAD = HEADER_SIZE + CRC_SIZE
MAX_PAYLOAD = 160
JOINT_NUM = 7
JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6", "Grip")

MSG_STATE = 0x01
MSG_POSITION_CMD = 0x02
MSG_ENABLE = 0x03
MSG_STOP = 0x04
MSG_HEARTBEAT = 0x05
MSG_ACK = 0x06
MSG_FAULT = 0x07
MSG_DIAGNOSTIC = 0x08

MSG_NAMES = {
    MSG_DIAGNOSTIC: "DIAGNOSTIC", MSG_STATE: "STATE", MSG_POSITION_CMD: "POSITION_CMD", MSG_ENABLE: "ENABLE",
    MSG_STOP: "STOP", MSG_HEARTBEAT: "HEARTBEAT", MSG_ACK: "ACK", MSG_FAULT: "FAULT",
}

SYS_IDLE, SYS_ENABLED, SYS_RUNNING, SYS_FAULT = 0, 1, 2, 3
SYS_NAMES = {SYS_IDLE: "IDLE", SYS_ENABLED: "ENABLED", SYS_RUNNING: "RUNNING", SYS_FAULT: "FAULT"}

FAULT_LINK_TIMEOUT = 0x0001
FAULT_USB_DISCONNECT = 0x0002
FAULT_RX_OVERFLOW = 0x0004
FAULT_FEEDBACK_LOST = 0x0008
FAULT_CAN_BUS = 0x0010
FAULT_DM_ENABLE = 0x0020

ACK_OK = 0x00
ACK_BAD_LENGTH = 0x01
ACK_BAD_VERSION = 0x02
ACK_NOT_ENABLED = 0x03
ACK_OUT_OF_RANGE = 0x04
ACK_UNKNOWN_TYPE = 0x05
ACK_BAD_VALUE = 0x06
ACK_STALE = 0x07
ACK_NOT_READY = 0x08
ACK_NO_SESSION = 0x09
ACK_WRONG_MODE = 0x0A
ACK_NAMES = {
    ACK_OK: "OK", ACK_BAD_LENGTH: "BAD_LENGTH", ACK_BAD_VERSION: "BAD_VERSION",
    ACK_NOT_ENABLED: "NOT_ENABLED", ACK_OUT_OF_RANGE: "OUT_OF_RANGE",
    ACK_UNKNOWN_TYPE: "UNKNOWN_TYPE", ACK_BAD_VALUE: "BAD_VALUE",
    ACK_STALE: "STALE", ACK_NOT_READY: "NOT_READY", ACK_NO_SESSION: "NO_SESSION",
    ACK_WRONG_MODE: "WRONG_MODE",
}

STATE_FLAG_ARM_READY = 0x01
FEEDBACK_AGE_NEVER = 0xFFFF

# Arm_mode_t order in APP/Arm_L_task.h
ARM_MODE_NAMES = (
    "MOVEMENT_J1_J4", "MOVEMENT_J5_GRIP", "KEY_CONTROL", "RETURN_INITIAL",
    "CHASSIS_KEEP", "POWERLESS", "TAKE_WATER", "PASS_WATER", "RESET", "TAKE_GRIP",
    "PC_CONTROL",
)
ARM_MODE_PC_CONTROL = 10

_STATE_FMT = "<IBBHBBH7H7i7i7i7i"
_POS_CMD_FMT = "<IHBB7i7i"
STATE_PAYLOAD_SIZE = struct.calcsize(_STATE_FMT)       # 138
POS_CMD_PAYLOAD_SIZE = struct.calcsize(_POS_CMD_FMT)   # 64
assert STATE_PAYLOAD_SIZE == 138 and POS_CMD_PAYLOAD_SIZE == 64

URAD_PER_RAD = 1_000_000.0
MNM_PER_NM = 1000.0


class ProtocolError(ValueError):
    pass


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def mode_name(arm_mode: int) -> str:
    if 0 <= arm_mode < len(ARM_MODE_NAMES):
        return ARM_MODE_NAMES[arm_mode]
    return "MODE_%d" % arm_mode


def fault_names(code: int) -> str:
    names = [n for bit, n in ((FAULT_LINK_TIMEOUT, "LINK_TIMEOUT"),
                              (FAULT_USB_DISCONNECT, "USB_DISCONNECT"),
                              (FAULT_RX_OVERFLOW, "RX_OVERFLOW"),
                              (FAULT_FEEDBACK_LOST, "FEEDBACK_LOST"),
                              (FAULT_CAN_BUS, "CAN_BUS"), (FAULT_DM_ENABLE, "DM_ENABLE")) if code & bit]
    return "|".join(names) if names else "none"


def rad_to_urad(rad: float) -> int:
    value = int(round(rad * URAD_PER_RAD))
    return max(-2**31, min(2**31 - 1, value))


def urad_to_rad(urad: int) -> float:
    return urad / URAD_PER_RAD


# ----------------------------------------------------------------- frames

@dataclass
class Frame:
    type: int
    sequence: int
    timestamp_ms: int
    payload: bytes
    version: int = VERSION

    @property
    def name(self) -> str:
        return MSG_NAMES.get(self.type, "0x%02X" % self.type)


def build_frame(msg_type: int, payload: bytes = b"", sequence: int = 0,
                timestamp_ms: int = 0, version: int = VERSION) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ProtocolError("payload too long: %d" % len(payload))
    body = struct.pack("<BBBBHI", SOF, version, msg_type, sequence & 0xFF,
                       len(payload), timestamp_ms & 0xFFFFFFFF) + payload
    return body + struct.pack("<H", crc16(body))


class FrameStream:
    """Incremental parser: feed arbitrary chunks, iterate complete frames.

    Same recovery rule as the firmware: drop one byte on a bad SOF, an
    impossible length or a CRC mismatch.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.crc_errors = 0
        self.resync_bytes = 0

    def feed(self, data: bytes) -> List[Frame]:
        self._buf.extend(data)
        frames = []
        while True:
            start = self._buf.find(SOF)
            if start < 0:
                self.resync_bytes += len(self._buf)
                self._buf.clear()
                break
            if start:
                self.resync_bytes += start
                del self._buf[:start]
            if len(self._buf) < HEADER_SIZE:
                break
            _, version, msg_type, seq, length, tick = struct.unpack_from("<BBBBHI", self._buf)
            if length > MAX_PAYLOAD:
                del self._buf[0]
                self.resync_bytes += 1
                continue
            total = OVERHEAD + length
            if len(self._buf) < total:
                break
            body = bytes(self._buf[:HEADER_SIZE + length])
            (crc_rx,) = struct.unpack_from("<H", self._buf, HEADER_SIZE + length)
            if crc16(body) != crc_rx:
                del self._buf[0]
                self.crc_errors += 1
                continue
            frames.append(Frame(msg_type, seq, tick, body[HEADER_SIZE:], version))
            del self._buf[:total]
        return frames


def decode_frames(data: bytes) -> List[Frame]:
    return FrameStream().feed(data)


# --------------------------------------------------------------- payloads

@dataclass
class ArmState:
    timestamp_ms: int
    arm_mode: int
    system_state: int
    fault_code: int
    flags: int
    feedback_valid: int             # bit i = joint i fed back recently
    feedback_age_ms: List[int]      # FEEDBACK_AGE_NEVER = never
    position: List[float]           # rad, motor feedback
    velocity: List[float]           # rad/s, same sample as position
    torque: List[float]             # Nm, same sample as position
    target: List[float]             # rad, controller output
    raw: tuple = field(default=(), repr=False)

    @property
    def system_name(self) -> str:
        return SYS_NAMES.get(self.system_state, str(self.system_state))

    @property
    def arm_ready(self) -> bool:
        return bool(self.flags & STATE_FLAG_ARM_READY)

    @property
    def all_feedback_valid(self) -> bool:
        return self.feedback_valid == (1 << JOINT_NUM) - 1

    def joint_valid(self, i: int) -> bool:
        return bool(self.feedback_valid & (1 << i))


def decode_state(payload: bytes) -> ArmState:
    if len(payload) != STATE_PAYLOAD_SIZE:
        raise ProtocolError("STATE payload is %d bytes, expected %d"
                            % (len(payload), STATE_PAYLOAD_SIZE))
    v = struct.unpack(_STATE_FMT, payload)
    age, pos, vel, tor, tgt = v[7:14], v[14:21], v[21:28], v[28:35], v[35:42]
    return ArmState(v[0], v[1], v[2], v[3], v[4], v[5], list(age),
                    [urad_to_rad(x) for x in pos], [urad_to_rad(x) for x in vel],
                    [x / MNM_PER_NM for x in tor], [urad_to_rad(x) for x in tgt], v)


def encode_state(timestamp_ms: int, arm_mode: int, system_state: int, fault_code: int,
                 flags: int, feedback_valid: int, feedback_age_ms: Sequence[int],
                 position_urad: Sequence[int], velocity_urad_s: Sequence[int],
                 torque_mnm: Sequence[int], target_urad: Sequence[int]) -> bytes:
    return struct.pack(_STATE_FMT, timestamp_ms, arm_mode, system_state, fault_code,
                       flags, feedback_valid, 0, *feedback_age_ms, *position_urad,
                       *velocity_urad_s, *torque_mnm, *target_urad)


@dataclass
class PositionCommand:
    command_timestamp_ms: int
    valid_time_ms: int
    enable: int
    position: List[float]
    velocity_limit: List[float]


def encode_position_cmd(positions_rad: Sequence[float], valid_time_ms: int = 100,
                        enable: bool = True, command_timestamp_ms: int = 0,
                        velocity_limit_rad_s: Optional[Sequence[float]] = None) -> bytes:
    if len(positions_rad) != JOINT_NUM:
        raise ProtocolError("need %d joint positions, got %d" % (JOINT_NUM, len(positions_rad)))
    limits = [0.0] * JOINT_NUM if velocity_limit_rad_s is None else velocity_limit_rad_s
    if len(limits) != JOINT_NUM or any(v < 0 for v in limits):
        raise ProtocolError("need seven nonnegative velocity limits; 0 uses firmware default")
    return struct.pack(_POS_CMD_FMT, command_timestamp_ms & 0xFFFFFFFF, valid_time_ms,
                       1 if enable else 0, 0,
                       *[rad_to_urad(p) for p in positions_rad],
                       *[rad_to_urad(v) for v in limits])


def decode_position_cmd(payload: bytes) -> PositionCommand:
    if len(payload) != POS_CMD_PAYLOAD_SIZE:
        raise ProtocolError("POSITION_CMD payload is %d bytes" % len(payload))
    v = struct.unpack(_POS_CMD_FMT, payload)
    return PositionCommand(v[0], v[1], v[2], [urad_to_rad(x) for x in v[4:11]],
                           [urad_to_rad(x) for x in v[11:18]])


def encode_enable(enable: bool) -> bytes:
    return struct.pack("<B3x", 1 if enable else 0)


@dataclass
class Ack:
    acked_type: int
    acked_sequence: int
    result: int

    @property
    def result_name(self) -> str:
        return ACK_NAMES.get(self.result, str(self.result))


def decode_ack(payload: bytes) -> Ack:
    if len(payload) != 4:
        raise ProtocolError("ACK payload is %d bytes" % len(payload))
    t, s, r, _ = struct.unpack("<BBBB", payload)
    return Ack(t, s, r)


@dataclass
class Fault:
    fault_code: int
    system_state: int


def decode_fault(payload: bytes) -> Fault:
    if len(payload) != 4:
        raise ProtocolError("FAULT payload is %d bytes" % len(payload))
    code, state, _ = struct.unpack("<HBB", payload)
    return Fault(code, state)


# ------------------------------------------------------------ host helper

class HostEncoder:
    """Keeps the host -> MCU sequence counter and the command timestamps.

    POSITION_CMD timestamps come from time.monotonic() and are forced to
    increase strictly, as the MCU requires. Call new_session() after the
    port was reopened.
    """

    def __init__(self, clock=None) -> None:
        import time as _time
        self._clock = clock or _time.monotonic
        self.sequence = 0
        self._last_cmd_ms = None

    def new_session(self) -> None:
        self._last_cmd_ms = None

    def now_ms(self) -> int:
        return int(self._clock() * 1000.0) & 0xFFFFFFFF

    def _next(self) -> int:
        seq = self.sequence
        self.sequence = (self.sequence + 1) & 0xFF
        return seq

    def _cmd_timestamp(self) -> int:
        ts = self.now_ms()
        if self._last_cmd_ms is not None and ((ts - self._last_cmd_ms) & 0xFFFFFFFF) >= 0x80000000                 or ts == self._last_cmd_ms:
            ts = (self._last_cmd_ms + 1) & 0xFFFFFFFF
        self._last_cmd_ms = ts
        return ts

    def frame(self, msg_type: int, payload: bytes = b"", timestamp_ms: Optional[int] = None) -> bytes:
        return build_frame(msg_type, payload, self._next(),
                           self.now_ms() if timestamp_ms is None else timestamp_ms)

    def enable(self, on: bool = True) -> bytes:
        return self.frame(MSG_ENABLE, encode_enable(on))

    def stop(self) -> bytes:
        return self.frame(MSG_STOP)

    def heartbeat(self) -> bytes:
        return self.frame(MSG_HEARTBEAT)

    def position(self, positions_rad: Sequence[float], valid_time_ms: int = 100,
                 enable: bool = True, velocity_limit_rad_s: Optional[Sequence[float]] = None) -> bytes:
        ts = self._cmd_timestamp()
        return self.frame(MSG_POSITION_CMD,
                          encode_position_cmd(positions_rad, valid_time_ms, enable, ts, velocity_limit_rad_s), ts)


def decode_diagnostic(payload: bytes) -> dict:
    """Health extension v1; counters wrap modulo 2**32 and latch until reset."""
    if len(payload) < 6:
        raise ProtocolError("short diagnostic header")
    schema, page, timestamp_ms = struct.unpack_from("<BBI", payload)
    if schema != 1 or page not in (0, 1):
        raise ProtocolError("unsupported diagnostic schema/page")
    fmt, count = ("<6I4B", 3) if page == 0 else ("<5I2B", 7)
    names = (("tx_drop", "rx_overflow", "rx_invalid", "busoff_count", "flags",
              "recovery_count", "tec", "rec", "tx_free_min", "blocked") if page == 0 else
             ("invalid_command", "tx_drop", "offline_skip", "fifo_defer",
              "first_enable_feedback_ms", "enable_state", "attempts"))
    size = struct.calcsize(fmt)
    if len(payload) != 6 + count * size:
        raise ProtocolError("bad diagnostic page size")
    return {"schema": schema, "page": page, "timestamp_ms": timestamp_ms,
            "buses" if page == 0 else "joints": [
                dict(zip(names, struct.unpack_from(fmt, payload, 6 + i * size)))
                for i in range(count)]}
