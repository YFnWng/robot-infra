"""Language-neutral Phase 4 shadow request and decision validation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


CURRENT_SCHEMA_VERSION = 1


@dataclass(frozen=True)
class ShadowRequestRecord:
    """Immutable request watermark retained by the shadow shell."""

    shell_epoch: int
    request_sequence: int
    target_revision: int
    device_sequence: int
    marker_sequence: int
    manager_sequence: int
    request_steady_ns: int = 0


@dataclass(frozen=True)
class ShadowDecisionRecord:
    """ROS-independent view of a worker decision."""

    schema_version: int
    shell_epoch: int
    request_sequence: int
    target_revision: int
    device_sequence: int
    marker_sequence: int
    manager_sequence: int
    computation_start_steady_ns: int
    computation_end_steady_ns: int
    valid: bool
    logical_velocity: Sequence[float]


def decision_rejection_reason(
        decision: ShadowDecisionRecord,
        request: ShadowRequestRecord | None,
        *,
        current_epoch: int,
        current_target_revision: int,
        last_accepted_sequence: int,
        now_steady_ns: int,
        maximum_result_age_ns: int,
        future_tolerance_ns: int = 1_000_000) -> str:
    """Return ``accepted`` or one stable fail-closed rejection reason."""
    if decision.schema_version != CURRENT_SCHEMA_VERSION:
        return "schema_version_mismatch"
    if request is None:
        return "request_unknown"
    if decision.shell_epoch != current_epoch:
        return "shell_epoch_mismatch"
    if decision.request_sequence != request.request_sequence:
        return "request_sequence_mismatch"
    if decision.request_sequence <= last_accepted_sequence:
        return "request_sequence_stale"
    if request.shell_epoch != current_epoch:
        return "request_epoch_stale"
    if (decision.target_revision != request.target_revision
            or decision.target_revision != current_target_revision):
        return "target_revision_mismatch"
    decision_watermark = (
        decision.device_sequence,
        decision.marker_sequence,
        decision.manager_sequence,
    )
    request_watermark = (
        request.device_sequence,
        request.marker_sequence,
        request.manager_sequence,
    )
    if decision_watermark != request_watermark:
        return "input_watermark_mismatch"
    if not decision.valid:
        return "worker_decision_invalid"
    if len(decision.logical_velocity) != 6:
        return "velocity_dimension_invalid"
    if not all(math.isfinite(float(value))
               for value in decision.logical_velocity):
        return "velocity_nonfinite"
    start = int(decision.computation_start_steady_ns)
    end = int(decision.computation_end_steady_ns)
    now = int(now_steady_ns)
    if start <= 0 or end < start:
        return "computation_time_invalid"
    if end > now+int(future_tolerance_ns):
        return "result_from_future"
    if now-end > int(maximum_result_age_ns):
        return "result_stale"
    return "accepted"
