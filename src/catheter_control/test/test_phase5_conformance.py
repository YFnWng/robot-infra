import json
from pathlib import Path

from catheter_control.safety.conformance import compare


def test_conformance_fixture_is_packaged_and_non_actuating():
    fixture = (Path(__file__).parents[1]/"catheter_control"/"safety"
               /"fixtures"/"phase5_conformance_v1.json")
    data = json.loads(fixture.read_text(encoding="utf-8"))

    assert data["scope"] == "OFFLINE_NON_ACTUATING_CONFORMANCE"
    assert data["encoder_zero_policy"] == "READ_ONLY_NEVER_SET_ZERO"
    assert set(data["controller"]["modes"]) == {
        "grouped", "plain_with_takeup", "plain"}
    assert (data["controller"]["modes"]["plain"]
            ["transaction_preview"]["policy"] == "direct")


def test_conformance_comparison_uses_numeric_tolerance_and_exact_reasons():
    expected = {"tip_m": [1.0, 2.0], "reason": "accepted"}
    assert compare(expected, {
        "tip_m": [1.0+2e-6, 2.0], "reason": "accepted"}) == []
    failures = compare(expected, {
        "tip_m": [1.0+4e-6, 2.0], "reason": "marker_outlier"})
    assert {failure["path"] for failure in failures} == {
        "tip_m", "reason"}
