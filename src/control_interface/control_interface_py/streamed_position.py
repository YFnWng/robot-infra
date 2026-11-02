"""Encoder-endpoint contract shared by gated ROS and serial dispatch.

Targets are absolute RAW encoder counts in existing physical shaft order.
Speeds are unsigned motor RPM ceilings, not logical joint velocities.
The M1 command remains immutable; the M2 envelope binds session and remaining
transport lifetime without changing the command on heartbeats.
"""
from dataclasses import dataclass
from numbers import Integral
import struct


CAPABILITY = "encoder_target_native_position_v1"
VERSION = 1
AXES = 6
MAX_RPM = 250
MAX_DURATION_US = 250_000
PAYLOAD = struct.Struct("<BIIB6i6H")
OPEN_SESSION = ord("J")
STREAM_COMMAND = ord("Y")
STREAM_STATUS = ord("U")
LIVE_UPDATE = ord("X")
LIVE_CAPABILITY = 'native_live_update_bracket_probe_v1'
LIVE_PAYLOAD = struct.Struct('<IIH')
ENVELOPE = struct.Struct("<II")
STATUS_PAYLOAD = struct.Struct("<IIIIBB6i6h")
STATUS_NAMES = ("IDLE", "RUNNING", "COMPLETE", "EXPIRED", "CANCELLED",
                "INVALID_COMMAND", "INVALID_FEEDBACK", "INHIBITED", "NO_PROGRESS",
                "REJECTED", "SUPERSEDED", "ACCEPTED", "NATIVE_STARTED", "LIVE_UPDATED",
                "LIVE_BEFORE_WRITE", "LIVE_AFTER_ACK")


def live_update_payload(values):
    """One diagnostic axis-0 A4.5 update, or a matched no-A control."""
    if len(values) != 3 or any(not float(x).is_integer() for x in values):
        raise ValueError('live probe requires integer session, command and angle')
    session, command, angle = (int(x) for x in values)
    if not 1 <= session <= 0xffffffff or not 1 <= command <= 0xffffffff or angle not in (0, 45):
        raise ValueError('live probe only permits control/A4.5 on a valid session/command')
    return LIVE_PAYLOAD.pack(session, command, angle)


def _integer(name, value, lower, upper):
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise ValueError(f"{name} must be an integer")
    if not lower <= value <= upper:
        raise ValueError(f"{name} outside [{lower}, {upper}]")
    return int(value)


@dataclass(frozen=True)
class StreamedPositionCommand:
    command_id: int
    duration_us: int
    active_mask: int
    target_counts: tuple[int, ...]
    speed_rpm: tuple[int, ...]

    def __post_init__(self):
        for name, lower, upper in (("command_id", 1, 0xFFFFFFFF),
                                   ("duration_us", 1, MAX_DURATION_US),
                                   ("active_mask", 0, 0x3F)):
            object.__setattr__(self, name, _integer(name, getattr(self, name), lower, upper))
        for name, lower, upper in (("target_counts", -0x80000000, 0x7FFFFFFF),
                                   ("speed_rpm", 0, MAX_RPM)):
            values = tuple(getattr(self, name))
            if len(values) != AXES:
                raise ValueError(f"{name} must contain six values")
            values = tuple(_integer(name, value, lower, upper) for value in values)
            object.__setattr__(self, name, values)
        for axis, speed in enumerate(self.speed_rpm):
            if bool(self.active_mask & (1 << axis)) != bool(speed):
                raise ValueError("active axes require nonzero RPM; inactive axes require zero RPM")

    def encode_payload(self) -> bytes:
        return PAYLOAD.pack(VERSION, self.command_id, self.duration_us,
                            self.active_mask, *self.target_counts, *self.speed_rpm)

    @classmethod
    def decode_payload(cls, payload: bytes):
        if len(payload) != PAYLOAD.size:
            raise ValueError("invalid streamed-position payload length")
        version, command_id, duration, mask, *values = PAYLOAD.unpack(payload)
        if version != VERSION:
            raise ValueError("unsupported streamed-position payload version")
        return cls(command_id, duration, mask, tuple(values[:AXES]), tuple(values[AXES:]))


def newer_command_id(candidate: int, previous: int) -> bool:
    """Serial-number arithmetic; zero is reserved and half-range is ambiguous."""
    candidate = _integer("candidate", candidate, 1, 0xFFFFFFFF)
    previous = _integer("previous", previous, 1, 0xFFFFFFFF)
    distance = (candidate - previous) & 0xFFFFFFFF
    return 0 < distance < 0x80000000


@dataclass(frozen=True)
class PositionEnvelope:
    session_id: int
    remaining_us: int
    command: StreamedPositionCommand

    def __post_init__(self):
        object.__setattr__(self, "session_id", _integer("session_id", self.session_id, 1, 0xFFFFFFFF))
        object.__setattr__(self, "remaining_us", _integer(
            "remaining_us", self.remaining_us, 1, self.command.duration_us))

    def encode_payload(self):
        return ENVELOPE.pack(self.session_id, self.remaining_us) + self.command.encode_payload()

    def as_data(self):
        c = self.command
        return [float(x) for x in (self.session_id, self.remaining_us, c.command_id,
                c.duration_us, c.active_mask, *c.target_counts, *c.speed_rpm)]

    @classmethod
    def from_data(cls, data):
        if len(data) != 17:
            raise ValueError("stream envelope requires 17 exact integer fields")
        if any(isinstance(x, bool) or not isinstance(x, (int, float))
               or not float(x).is_integer() for x in data):
            raise ValueError("stream envelope fields must be exact integers")
        v = [int(x) for x in data]
        return cls(v[0], v[1], StreamedPositionCommand(v[2], v[3], v[4], tuple(v[5:11]), tuple(v[11:17])))

    @classmethod
    def decode_payload(cls, payload):
        if len(payload) != ENVELOPE.size + PAYLOAD.size:
            raise ValueError("invalid stream envelope length")
        session, remaining = ENVELOPE.unpack(payload[:ENVELOPE.size])
        return cls(session, remaining, StreamedPositionCommand.decode_payload(payload[ENVELOPE.size:]))


def parse_session_response(text):
    if not text.startswith("STREAM_SESSION:"):
        raise ValueError("firmware did not acknowledge a position session")
    return _integer("session_id", int(text.split(":", 1)[1]), 1, 0xFFFFFFFF)


def decode_status(payload):
    if len(payload) != STATUS_PAYLOAD.size:
        raise ValueError("invalid streamed-position status length")
    values = STATUS_PAYLOAD.unpack(payload)
    status = values[4]
    if status >= len(STATUS_NAMES):
        raise ValueError("unknown streamed-position status")
    return "POSITION_STREAM:"+STATUS_NAMES[status], [float(x) for x in values]
