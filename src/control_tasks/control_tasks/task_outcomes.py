"""Controller failure evidence and expected task failures, without ROS or I/O."""
import json


class TaskFailure(RuntimeError):
    """An unsuccessful task outcome, not an unexpected implementation exception."""

    def __init__(self, message, *, fault_reason="", fault_snapshot_json=""):
        super().__init__(message)
        self.fault_reason = fault_reason
        self.fault_snapshot_json = fault_snapshot_json

    def fields(self):
        return dict(fault_reason=self.fault_reason,
                    fault_snapshot_json=self.fault_snapshot_json)


def _reject_constant(value):
    raise ValueError(f"nonfinite JSON: {value}")


def controller_failure(values):
    """Copy current FAULTED diagnostics; invalid evidence must not abort cleanup."""
    reason = str(values.get("reason", "unknown_controller_fault"))
    try:
        snapshot = json.loads(values.get("fault_snapshot", ""),
                              parse_constant=_reject_constant)
        if not isinstance(snapshot, dict) or snapshot.get("reason") != reason:
            return reason, ""
        return reason, json.dumps(snapshot, allow_nan=False, separators=(",", ":"))
    except (TypeError, ValueError):
        return reason, ""
