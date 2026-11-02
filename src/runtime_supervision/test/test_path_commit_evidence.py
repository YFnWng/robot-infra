from dataclasses import replace

import numpy as np
import pytest

from catheter_control.planning.path_tracking import ReferenceHorizon
from runtime_supervision.path_commit_evidence import commit_evidence, plan_outcomes


def reference():
    return ReferenceHorizon(
        path_id="circle", sequence=1, source_timestamp_ns=1_000_000_000,
        received_at_s=10., sample_period_s=.04,
        positions_m=np.zeros((8, 3)), expiry_s=.5,
        progress_m=0., total_length_m=.025, final_hold=False,
        progress_paused=False, tangents=np.tile([1., 0., 0.], (8, 1)))


def evaluate(a, b, now=10.1, *, bounded_refresh=False, tangent_tolerance=1e-3):
    error = None
    try:
        compatible = a.compatible_with(
            b, 1_000_000_000, 4, .04, now, bounded_refresh=bounded_refresh,
            bounded_tangent_tolerance=tangent_tolerance)
    except ValueError as exc:
        compatible, error = False, str(exc)
    return commit_evidence(a, b, 1_000_000_000, 4, .04, now, compatible, error)


def test_identical_refresh_passes_without_action():
    a = reference()
    row = evaluate(a, replace(a, sequence=2))
    assert row["reference_compatible"]
    assert row["position_max_coordinate_change_mm"] == 0
    assert row["current_reference"]["sequence"] == 2


def test_small_position_change_explains_production_rejection():
    a = reference()
    row = evaluate(a, replace(a, sequence=2, positions_m=a.positions_m+2e-6))
    assert not row["reference_compatible"]
    assert row["position_max_coordinate_change_mm"] == pytest.approx(.002)


def test_tangent_only_change_is_visible():
    a = reference()
    row = evaluate(a, replace(a, tangents=np.tile([1., .001, 0.], (8, 1))))
    assert not row["reference_compatible"]
    assert row["position_max_coordinate_change_mm"] == 0
    assert row["tangent_max_component_change"] > 1e-4


@pytest.mark.parametrize("changes", [dict(path_id="other"), dict(final_hold=True),
                                     dict(progress_paused=True)])
def test_policy_identity_changes_still_reject(changes):
    a = reference()
    assert not evaluate(a, replace(a, **changes))["reference_compatible"]


def test_cancelled_reference_and_expiry_remain_distinct():
    a = reference()
    assert evaluate(a, None)["current_reference"] is None
    row = evaluate(a, a, now=11.)
    assert row["gate_error"] == "path_reference_stale"
    assert row["comparison_error"] == "path_reference_stale"


def test_too_short_reference_is_reported():
    a = reference()
    row = evaluate(a, replace(a, positions_m=a.positions_m[:2], tangents=a.tangents[:2]))
    assert row["gate_error"] == "path_reference_too_short"


def test_path_replay_requires_non_actuating_production_probe_configuration():
    from runtime_supervision.compare_ros import main
    with pytest.raises(SystemExit) as exc:
        main(["session", "--model-manifest", "model", "--limits-file", "limits",
              "--output", "out", "--path-reference-commit"])
    assert exc.value.code == 2


def test_bounded_candidate_accepts_small_refresh_but_default_remains_strict():
    a = reference()
    b = replace(a, sequence=2, progress_m=6.7e-5,
                positions_m=a.positions_m+4e-6,
                tangents=np.tile([1., .0008, 0.], (8, 1)))
    assert not evaluate(a, b)["reference_compatible"]
    assert evaluate(a, b, bounded_refresh=True)["reference_compatible"]


@pytest.mark.parametrize("changes", [
    dict(path_id="other"), dict(final_hold=True), dict(progress_paused=True),
    dict(total_length_m=.03), dict(sequence=0),
    dict(positions_m=np.full((8, 3), 11e-6)),
    dict(tangents=np.tile([1., .0021, 0.], (8, 1))),
    dict(progress_m=.00011), dict(progress_m=-2e-6),
    dict(progress_m=.025),  # Closed-circle alias: same XYZ, a whole lap later.
    dict(progress_m=float("nan")),
])
@pytest.mark.parametrize("tangent_tolerance", [1e-3, 2e-3])
def test_bounded_candidate_rejects_material_changes_and_phase_alias(changes, tangent_tolerance):
    a = reference()
    assert not evaluate(a, replace(a, **changes), bounded_refresh=True,
                        tangent_tolerance=tangent_tolerance)["reference_compatible"]


@pytest.mark.parametrize("tangent_tolerance", [1e-3, 2e-3])
def test_bounded_candidate_cannot_accept_cancelled_reference(tangent_tolerance):
    assert not evaluate(reference(), None, bounded_refresh=True,
                        tangent_tolerance=tangent_tolerance)["reference_compatible"]


@pytest.mark.parametrize("changes,now,reason", [
    (dict(received_at_s=9.), 10.1, "path_reference_stale"),
    (dict(source_timestamp_ns=1_100_000_000), 10.1, "path_reference_starts_too_late"),
    (dict(positions_m=np.zeros((2, 3)), tangents=np.tile([1., 0., 0.], (2, 1))),
     10.1, "path_reference_too_short"),
    (dict(received_at_s=10.6), 10.6, "path_reference_stale"),  # Solved version expired.
])
@pytest.mark.parametrize("tangent_tolerance", [1e-3, 2e-3])
def test_bounded_candidate_preserves_both_versions_freshness_and_coverage(
        changes, now, reason, tangent_tolerance):
    a = reference()
    row = evaluate(a, replace(a, **changes), now, bounded_refresh=True,
                   tangent_tolerance=tangent_tolerance)
    assert not row["reference_compatible"]
    assert row["gate_error"] == reason


@pytest.mark.parametrize("tangent_tolerance", [1e-3, 2e-3])
def test_bounded_candidate_keeps_final_hold_endpoint_strict(tangent_tolerance):
    a = replace(reference(), final_hold=True)
    b = replace(a, tangents=np.tile([0., 1., 0.], (8, 1)))
    assert evaluate(a, b, bounded_refresh=True,
                    tangent_tolerance=tangent_tolerance)["reference_compatible"]
    assert not evaluate(a, replace(b, positions_m=b.positions_m+2e-6),
                        bounded_refresh=True,
                        tangent_tolerance=tangent_tolerance)["reference_compatible"]


def test_tangent_relaxed_candidate_changes_only_tangent_bound():
    a = reference()
    b = replace(a, sequence=2, progress_m=3e-5, positions_m=a.positions_m+7.4e-6,
                tangents=np.tile([1., .00165, 0.], (8, 1)))
    assert not evaluate(a, b)["reference_compatible"]
    assert not evaluate(a, b, bounded_refresh=True)["reference_compatible"]
    assert evaluate(a, b, bounded_refresh=True,
                    tangent_tolerance=2e-3)["reference_compatible"]
    # The custom candidate bound cannot affect strict production evaluation.
    assert not evaluate(a, b, tangent_tolerance=2e-3)["reference_compatible"]


@pytest.mark.parametrize("tolerance", [0., -1., float("nan"), float("inf")])
def test_candidate_tangent_tolerance_validation(tolerance):
    a = reference()
    row = evaluate(a, a, bounded_refresh=True, tangent_tolerance=tolerance)
    assert not row["reference_compatible"]
    assert row["gate_error"] == "invalid bounded tangent tolerance"


def transition_plans(policy="final_hold"):
    a = reference()
    b = replace(a, sequence=2, **{policy: True})
    c = replace(b, sequence=3)
    return [
        {"valid": False, "reason": "worker_state_changed", "command_executed": False,
         "proposed_command": [0.]*6, "path_commit": evaluate(a, b),
         "plan_started_offset_s": 0.},
        {"valid": True, "reason": "ok", "command_executed": False,
         "proposed_command": [0.]*6, "path_commit": evaluate(c, c),
         "plan_started_offset_s": 1/15},
    ]


@pytest.mark.parametrize("policy", ["final_hold", "progress_paused"])
def test_only_opt_in_recovered_policy_transitions_qualify(policy):
    from runtime_supervision.compare_ros import plan_coverage, require_measurement_work
    plans = transition_plans(policy)
    assert not plan_outcomes(plans)["passed"]
    outcomes = plan_outcomes(plans, allow_path_transitions=True)
    assert outcomes["passed"]
    assert outcomes["valid_count"] == outcomes["invalid_count"] == 1
    assert outcomes["expected_transition_plan_indices"] == [0]
    assert outcomes["unexpected_invalid_count"] == 0
    stages = {"ros_probe_planner": {"count": 2}}
    require_measurement_work("planner-only", stages, plans, [], allow_path_transitions=True)
    with pytest.raises(RuntimeError, match="invalid=1"):
        require_measurement_work("planner-only", stages, plans, [])
    coverage = plan_coverage(plans, 0., 2/15, 15, allow_path_transitions=True)
    assert coverage["passed"] and coverage["invalid_count"] == 1
    assert not plan_coverage(plans, 0., 1., 15, allow_path_transitions=True)["passed"]


@pytest.mark.parametrize("mutation", [
    "no_transition", "new_path", "no_recovery", "recovery_wrong_policy",
    "recovery_rejected", "deadline", "nonzero", "nan", "stale", "missing_evidence",
    "sequence_rewind", "length_change", "executed", "recovery_executed",
])
def test_transition_exemption_fails_closed(mutation):
    plans = transition_plans()
    row, next_row = plans
    if mutation == "no_transition":
        row["path_commit"]["current_reference"]["final_hold"] = False
    elif mutation == "new_path":
        row["path_commit"]["current_reference"]["path_id"] = "different"
    elif mutation == "no_recovery":
        plans.pop()
    elif mutation == "recovery_wrong_policy":
        next_row["path_commit"]["captured_reference"]["final_hold"] = False
    elif mutation == "recovery_rejected":
        next_row["path_commit"]["reference_compatible"] = False
    elif mutation == "deadline":
        row["reason"] = "deadline_missed"
    elif mutation == "nonzero":
        row["proposed_command"][0] = 1.
    elif mutation == "nan":
        row["proposed_command"][0] = float("nan")
    elif mutation == "stale":
        row["path_commit"]["gate_error"] = "path_reference_stale"
    elif mutation == "missing_evidence":
        del row["path_commit"]["comparison_error"]
    elif mutation == "sequence_rewind":
        next_row["path_commit"]["captured_reference"]["sequence"] = 0
    elif mutation == "length_change":
        row["path_commit"]["current_reference"]["total_length_m"] = .03
    elif mutation == "executed":
        row["command_executed"] = True
    elif mutation == "recovery_executed":
        next_row["command_executed"] = True
    result = plan_outcomes(plans, allow_path_transitions=True)
    assert not result["passed"]
    assert result["expected_transition_supersessions"] == 0
    assert result["unexpected_invalid_count"] == 1
