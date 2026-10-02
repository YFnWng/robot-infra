"""Durable, append-only task observations; no command or ROS dependencies."""
import json
import math
import os
from pathlib import Path
import time


class TrialRecords:
    """Create one journal per invocation, refusing to overwrite prior trials."""

    def __init__(self, path):
        self.stream = Path(path).open("x", encoding="utf-8") if path else None

    def write(self, event, ros_time_ns, **fields):
        if self.stream is None:
            return
        record = dict(event=event, ros_time_ns=int(ros_time_ns),
                      monotonic_ns=time.monotonic_ns(), **fields)
        self.stream.write(json.dumps(record, allow_nan=False) + "\n")
        self.stream.flush()
        os.fsync(self.stream.fileno())

    def close(self):
        if self.stream is not None:
            self.stream.close()


def finite_error(value):
    value = float(value)
    return value if math.isfinite(value) else None
