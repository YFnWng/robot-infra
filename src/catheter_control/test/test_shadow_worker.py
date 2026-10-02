import math

import pytest

from catheter_control.orchestration.shadow_worker import (
    WorkerSnapshot, qualify_snapshot)


def snapshot(**changes):
    values = dict(
        velocity=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
        plan_age_s=0.01,
        timing_age_s=0.01,
        plan_valid=True,
        plan_reason="accepted",
        controller_state="ACTIVE",
        controller_reason="active",
        estimator_health="TRACKING",
        estimator_state_timestamp_ns=123,
    )
    values.update(changes)
    return WorkerSnapshot(**values)


def test_fresh_projected_plan_is_accepted():
    assert qualify_snapshot(snapshot(), 0.20) == "accepted"


@pytest.mark.parametrize("change, expected", [
    ({"velocity": None}, "planned_control_unavailable"),
    ({"velocity": (1.0, 2.0)}, "planned_control_dimension_invalid"),
    ({"velocity": (1.0, 2.0, 3.0, 4.0, 5.0, math.nan)},
     "planned_control_nonfinite"),
    ({"plan_age_s": 0.21}, "planned_control_stale"),
    ({"timing_age_s": None}, "planner_timing_unavailable"),
    ({"timing_age_s": 0.21}, "planner_timing_stale"),
    ({"plan_valid": False, "plan_reason": "deadline_missed"},
     "planner_invalid:deadline_missed"),
])
def test_invalid_snapshot_fails_closed(change, expected):
    assert qualify_snapshot(snapshot(**change), 0.20) == expected
