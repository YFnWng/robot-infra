"""Fail-closed timestamp checks for the latest-value motion stream."""

from __future__ import annotations


def stamp_nanoseconds(message) -> int:
    """Return a ROS message header stamp as integer nanoseconds."""
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def validate_command_stamp(
        message, *, now_ns: int, last_stamp_ns: int,
        maximum_age_s: float, future_tolerance_s: float):
    """Validate a source timestamp without replacing it at relay boundaries.

    Returns ``(accepted, reason, stamp_ns)``. A zero stamp, an expired/future
    stamp, and a stamp no newer than the last accepted nonzero command all fail
    closed. Callers may deliberately exempt an all-zero velocity command so a
    stale stop can still preempt motion.
    """
    stamp_ns = stamp_nanoseconds(message)
    if stamp_ns <= 0:
        return False, "missing_source_stamp", stamp_ns
    age_ns = int(now_ns) - stamp_ns
    if age_ns < -int(float(future_tolerance_s) * 1e9):
        return False, "source_stamp_from_future", stamp_ns
    if age_ns > int(float(maximum_age_s) * 1e9):
        return False, "source_command_expired", stamp_ns
    if stamp_ns <= int(last_stamp_ns):
        return False, "source_command_nonmonotonic", stamp_ns
    return True, "accepted", stamp_ns
