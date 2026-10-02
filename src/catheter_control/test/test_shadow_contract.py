import csv
from pathlib import Path

from catheter_control.orchestration.shadow_contract import (
    ShadowDecisionRecord,
    ShadowRequestRecord,
    decision_rejection_reason,
)


FIXTURE = Path(__file__).parent / "fixtures" / "shadow_decision_cases.csv"


def _integer(row, name):
    return int(row[name])


def test_shared_shadow_decision_cases():
    with FIXTURE.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))

    assert len(rows) >= 10
    for row in rows:
        request = ShadowRequestRecord(
            shell_epoch=_integer(row, "req_epoch"),
            request_sequence=_integer(row, "req_seq"),
            target_revision=_integer(row, "req_target"),
            device_sequence=_integer(row, "req_device"),
            marker_sequence=_integer(row, "req_marker"),
            manager_sequence=_integer(row, "req_manager"),
        )
        decision = ShadowDecisionRecord(
            schema_version=_integer(row, "dec_schema"),
            shell_epoch=_integer(row, "dec_epoch"),
            request_sequence=_integer(row, "dec_seq"),
            target_revision=_integer(row, "dec_target"),
            device_sequence=_integer(row, "dec_device"),
            marker_sequence=_integer(row, "dec_marker"),
            manager_sequence=_integer(row, "dec_manager"),
            computation_start_steady_ns=_integer(row, "start_ns"),
            computation_end_steady_ns=_integer(row, "end_ns"),
            valid=bool(_integer(row, "valid")),
            logical_velocity=[float(row["v0"]), 0, 0, 0, 0, 0],
        )
        actual = decision_rejection_reason(
            decision,
            request,
            current_epoch=_integer(row, "current_epoch"),
            current_target_revision=_integer(row, "current_target"),
            last_accepted_sequence=_integer(row, "last_accepted"),
            now_steady_ns=_integer(row, "now_ns"),
            maximum_result_age_ns=_integer(row, "max_age_ns"),
        )
        assert actual == row["expected"], row["name"]


def test_unknown_request_is_rejected():
    decision = ShadowDecisionRecord(
        schema_version=1,
        shell_epoch=1,
        request_sequence=1,
        target_revision=0,
        device_sequence=0,
        marker_sequence=0,
        manager_sequence=0,
        computation_start_steady_ns=1,
        computation_end_steady_ns=1,
        valid=True,
        logical_velocity=[0.0]*6,
    )
    assert decision_rejection_reason(
        decision,
        None,
        current_epoch=1,
        current_target_revision=0,
        last_accepted_sequence=0,
        now_steady_ns=1,
        maximum_result_age_ns=1,
    ) == "request_unknown"
