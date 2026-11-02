"""Diagnostic evidence for the production path-reference commit predicate.

No runtime policy, ROS entities, actuators or alternative compatibility thresholds.
ReferenceHorizon remains the sole owner of reference sampling and compatibility.
"""
from __future__ import annotations

import numpy as np


def _recovered_policy_transition(row, following):
    """Recognize a discarded policy-boundary solve, never a usable plan."""
    if (row.get("valid") is not False
            or row.get("reason") != "worker_state_changed"
            or row.get("command_executed") is not False
            or following is None or following.get("valid") is not True
            or following.get("reason") != "ok"
            or following.get("command_executed") is not False):
        return False
    command = np.asarray(row.get("proposed_command", []), dtype=float)
    if command.shape != (6,) or not np.all(command == 0.0):
        return False
    evidence = row.get("path_commit", {})
    recovery = following.get("path_commit", {})
    required = ("captured_reference", "current_reference", "gate_error",
                "comparison_error", "reference_compatible")
    if any(key not in evidence or key not in recovery for key in required):
        return False
    if (evidence["gate_error"] is not None or evidence["comparison_error"] is not None
            or evidence["reference_compatible"] is not False
            or recovery["gate_error"] is not None or recovery["comparison_error"] is not None
            or recovery["reference_compatible"] is not True):
        return False
    a, b = evidence["captured_reference"], evidence["current_reference"]
    c = recovery["captured_reference"]
    fields = ("path_id", "sequence", "total_length_m", "final_hold", "progress_paused")
    if any(not isinstance(ref, dict) or any(key not in ref for key in fields)
           for ref in (a, b, c)):
        return False
    if (not a["path_id"] or a["path_id"] != b["path_id"] or b["path_id"] != c["path_id"]
            or a["total_length_m"] != b["total_length_m"]
            or b["total_length_m"] != c["total_length_m"]
            or not a["sequence"] < b["sequence"] <= c["sequence"]):
        return False
    policy = ("final_hold", "progress_paused")
    if any(type(ref[key]) is not bool for ref in (a, b, c) for key in policy):
        return False
    return (any(a[key] != b[key] for key in policy)
            and all(b[key] == c[key] for key in policy))


def plan_outcomes(plans, *, allow_path_transitions=False):
    """Qualification outcomes with raw invalid plans preserved for attribution.

    Only path replays may recognize a zero-result policy supersession followed
    immediately by a valid, compatible solve for that same updated policy.
    Generic resets, expiry, missing evidence and unrecovered transitions fail.
    """
    expected, unexpected = [], []
    for index, row in enumerate(plans):
        if row.get("valid", False):
            continue
        following = plans[index+1] if index+1 < len(plans) else None
        if allow_path_transitions and _recovered_policy_transition(row, following):
            expected.append(index)
        else:
            unexpected.append(index)
    return {
        "valid_count": sum(bool(row.get("valid", False)) for row in plans),
        "invalid_count": len(expected)+len(unexpected),
        "expected_transition_supersessions": len(expected),
        "expected_transition_plan_indices": expected,
        "unexpected_invalid_count": len(unexpected),
        "unexpected_invalid_plan_indices": unexpected,
        "passed": any(row.get("valid", False) for row in plans) and not unexpected,
    }


def reference_record(reference):
    if reference is None:
        return None
    return {
        "path_id": reference.path_id, "sequence": reference.sequence,
        "source_timestamp_ns": reference.source_timestamp_ns,
        "received_at_s": reference.received_at_s,
        "progress_m": reference.progress_m,
        "final_hold": reference.final_hold,
        "progress_paused": reference.progress_paused,
        "total_length_m": reference.total_length_m,
    }


def commit_evidence(captured, current, root_ns, steps, step_s, now_s,
                    compatible, gate_error=None):
    """Describe an already evaluated gate without choosing or executing action."""
    row = {
        "captured_reference": reference_record(captured),
        "current_reference": reference_record(current),
        "root_timestamp_ns": root_ns,
        "reference_compatible": bool(compatible), "gate_error": gate_error,
        "comparison_error": None,
        "position_max_coordinate_change_mm": None,
        "tangent_max_component_change": None,
    }
    if captured is None or current is None:
        return row
    try:
        a, ta = captured.sample(root_ns, steps, step_s, now_s)
        b, tb = current.sample(root_ns, steps, step_s, now_s)
        row["captured_targets_m"] = a.tolist()
        row["current_targets_m"] = b.tolist()
        row["position_max_coordinate_change_mm"] = float(
            1000*np.max(np.abs(a-b)))
        if ta is not None and tb is not None:
            row["tangent_max_component_change"] = float(np.max(np.abs(ta-tb)))
    except ValueError as error:
        row["comparison_error"] = str(error)
    return row
