import pytest
import torch

from runtime_supervision.benchmark_accelerator import compare_outputs, measure, main


def test_output_contract_rejects_missing_fields_and_numeric_change():
    compare_outputs((torch.ones(3),), (torch.ones(3),))
    with pytest.raises(AssertionError, match="structure"):
        compare_outputs((torch.ones(3),), (torch.ones(3), torch.ones(3)))
    with pytest.raises(AssertionError):
        compare_outputs((torch.zeros(3),), (torch.ones(3),))


def test_wall_measurement_counts_all_calls():
    calls = []
    report = measure(lambda: calls.append(1), 10, torch.device("cpu"))
    assert len(calls) == 10
    assert report["max_ms"] >= report["p99_ms"] >= report["p95_ms"] >= 0


def test_invalid_budget_fails_before_loading_or_output(tmp_path):
    output = tmp_path/"report.json"
    with pytest.raises(SystemExit):
        main(["--model-manifest", "missing.json", "--output", str(output),
              "--samples", "7"])
    assert not output.exists()
