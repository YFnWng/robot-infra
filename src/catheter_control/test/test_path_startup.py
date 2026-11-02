"""Path preview/warm-up owner regression tests; no nodes or actuator APIs."""
from threading import RLock
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest
from geometry_msgs.msg import Point, Vector3
from control_interface.msg import TipReferenceHorizon

from catheter_control.node import CatheterControlNode


def preview(sequence=1, path_id="circle", final_hold=False, paused=False):
    message = TipReferenceHorizon()
    message.header.frame_id = "robot_base"
    message.header.stamp.nanosec = 800_000_000
    message.path_id = path_id
    message.sequence = sequence
    message.sample_period_s = .02
    message.positions = [Point(x=.02+i*.00001, y=.02, z=.06) for i in range(41)]
    message.tangents = [Vector3(x=1., y=0., z=0.) for _ in message.positions]
    message.expiry_s = .15
    message.total_length_m = .025
    message.final_hold = final_hold
    message.progress_paused = paused
    return message


def owner(planner=None):
    faults = []
    resets = []
    if planner is None:
        planner = SimpleNamespace(reset_capture_target=lambda: resets.append(1))
    node = SimpleNamespace(
        _lock=RLock(), planner=planner, frame_id="robot_base",
        path_reference=None, reference_source="NONE", armed=False,
        target=None, _steady=lambda: 100., _now_ns=lambda: 1_000_000_000,
        get_parameter=lambda _: SimpleNamespace(value=.15),
        reversal_scheduler=SimpleNamespace(reset_intent=lambda: None),
        tip_forecast_monitor=SimpleNamespace(clear=lambda: None),
        _fault_if_active=faults.append, _fault_locked=faults.append,
        _planner_warmup_supersessions=0, maximum_deadline_misses=3,
        planner_warmed=False, last_command=np.ones(6),
        last_effective_command=np.ones(6), last_command_time=None,
        get_logger=lambda: SimpleNamespace(info=lambda *_: None))
    return node, faults, resets


def reference_pair(final_hold=False):
    node, _, _ = owner()
    node.planner.config = SimpleNamespace(horizon_steps=4, step_s=.04)
    message = preview(final_hold=final_hold)
    if final_hold:
        message.positions = [Point(x=.02, y=.02, z=.06) for _ in message.positions]
    CatheterControlNode._path_reference_cb(node, message)
    original = node.path_reference
    positions = original.positions_m.copy()
    if not final_hold:
        positions[:, 0] += .0005*.03  # Same linear path, origin shifted by 30 ms.
    node.path_reference = replace(original, sequence=2,
                                  source_timestamp_ns=830_000_000,
                                  received_at_s=100.03, positions_m=positions)
    return node, original


@pytest.mark.parametrize("final_hold", [False, True])
def test_commit_accepts_time_aligned_refresh(final_hold):
    node, original = reference_pair(final_hold)
    assert node.path_reference is not original
    assert CatheterControlNode._planner_reference_matches_locked(
        node, "PATH", None, original, 1_000_000_000, 100.04)


def test_final_hold_refresh_ignores_unused_tangent_but_rejects_new_endpoint():
    node, original = reference_pair(True)
    node.path_reference = replace(node.path_reference,
                                  tangents=np.tile([0., 1., 0.], (41, 1)))
    assert CatheterControlNode._planner_reference_matches_locked(
        node, "PATH", None, original, 1_000_000_000, 100.04)
    node.path_reference = replace(node.path_reference,
                                  positions_m=node.path_reference.positions_m+.001)
    assert not CatheterControlNode._planner_reference_matches_locked(
        node, "PATH", None, original, 1_000_000_000, 100.04)


@pytest.mark.parametrize("changes", [
    {"path_id": "other"}, {"final_hold": True}, {"progress_paused": True},
    {"total_length_m": .03}, {"sequence": 0},
    {"positions_m": np.full((41, 3), .05)},
    {"tangents": np.tile([0., 1., 0.], (41, 1))},
])
def test_commit_rejects_changed_path_objective(changes):
    node, original = reference_pair()
    node.path_reference = replace(node.path_reference, **changes)
    assert not CatheterControlNode._planner_reference_matches_locked(
        node, "PATH", None, original, 1_000_000_000, 100.04)


@pytest.mark.parametrize("changes,reason", [
    ({"received_at_s": 99.0}, "path_reference_stale"),
    ({"source_timestamp_ns": 1_100_000_000}, "path_reference_starts_too_late"),
    ({"positions_m": np.ones((2, 3)), "tangents": np.ones((2, 3))},
     "path_reference_too_short"),
])
def test_commit_refresh_retains_freshness_and_coverage_checks(changes, reason):
    node, original = reference_pair()
    node.path_reference = replace(node.path_reference, **changes)
    with pytest.raises(ValueError, match=reason):
        CatheterControlNode._planner_reference_matches_locked(
            node, "PATH", None, original, 1_000_000_000, 100.04)


def test_commit_rejects_stale_solved_preview_despite_fresh_refresh():
    node, original = reference_pair(True)
    node.path_reference = replace(node.path_reference, received_at_s=100.2)
    with pytest.raises(ValueError, match="path_reference_stale"):
        CatheterControlNode._planner_reference_matches_locked(
            node, "PATH", None, original, 1_000_000_000, 100.2)


def test_commit_rejects_cancellation_source_switch_and_point_change():
    node, original = reference_pair()
    node.path_reference = None
    assert not CatheterControlNode._planner_reference_matches_locked(
        node, "PATH", None, original, 1_000_000_000, 100.04)
    node.reference_source = "POINT"
    node.target = np.ones(3)
    assert not CatheterControlNode._planner_reference_matches_locked(
        node, "PATH", None, original, 1_000_000_000, 100.04)
    assert CatheterControlNode._planner_reference_matches_locked(
        node, "POINT", np.ones(3), None, 1_000_000_000, 100.04)
    assert not CatheterControlNode._planner_reference_matches_locked(
        node, "POINT", np.zeros(3), None, 1_000_000_000, 100.04)


def test_same_path_rolling_previews_preserve_memory_revision():
    node, faults, resets = owner()
    for sequence in range(1, 8):
        CatheterControlNode._path_reference_cb(node, preview(sequence))
    assert len(resets) == 1
    assert node.path_reference.sequence == 7
    assert not faults


def test_policy_changes_and_new_path_reset_capture_memory():
    node, _, resets = owner()
    for message in [preview(), preview(2, paused=True),
                    preview(3, final_hold=True), preview(1, path_id="new")]:
        CatheterControlNode._path_reference_cb(node, message)
    assert len(resets) == 4


def test_out_of_order_preview_and_armed_new_path_do_not_reset_memory():
    node, faults, resets = owner()
    CatheterControlNode._path_reference_cb(node, preview(2))
    CatheterControlNode._path_reference_cb(node, preview(1))
    node.armed = True
    CatheterControlNode._path_reference_cb(node, preview(1, path_id="new"))
    assert len(resets) == 1
    assert faults == ["path_reference_id_changed"]


def test_cancellation_invalidates_worker_policy():
    node, _, resets = owner()
    CatheterControlNode._path_reference_cb(node, preview())
    cancel = TipReferenceHorizon()
    cancel.path_id = "circle"
    CatheterControlNode._path_reference_cb(node, cancel)
    assert len(resets) == 2
    assert node.reference_source == "NONE"


def test_superseded_warmup_retries_zero_and_then_completes():
    node, faults, _ = owner()
    CatheterControlNode._complete_planner_warmup_locked(
        node, SimpleNamespace(reason="worker_state_changed"))
    assert not node.planner_warmed and not faults
    assert node.state_reason == "planner_warmup_superseded_retry"
    assert not np.any(node.last_command) and not np.any(node.last_effective_command)
    CatheterControlNode._complete_planner_warmup_locked(
        node, SimpleNamespace(reason="ok"))
    assert node.planner_warmed and node._planner_warmup_supersessions == 0


def test_repeated_supersession_is_bounded():
    node, faults, _ = owner()
    for _ in range(3):
        CatheterControlNode._complete_planner_warmup_locked(
            node, SimpleNamespace(reason="worker_state_changed"))
    assert not node.planner_warmed
    assert faults == ["planner_warmup:repeated_state_change"]


@pytest.mark.parametrize("reason", ["worker_failure", "worker_unavailable", "invalid_snapshot"])
def test_real_warmup_failure_still_faults(reason):
    node, faults, _ = owner()
    CatheterControlNode._complete_planner_warmup_locked(node, SimpleNamespace(reason=reason))
    assert not node.planner_warmed
    assert faults == [f"planner_warmup:{reason}"]
