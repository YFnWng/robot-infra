from dataclasses import dataclass

import numpy as np
import torch

from runtime_supervision.benchmark_compute_isolation import differences, state_record


def test_snapshot_report_preserves_optional_and_discrete_state():
    @dataclass
    class State:
        timestamp_ns: int
        value: object
        pending: object = None

    a = state_record(State(123, torch.tensor([1., 2.])))
    assert a == {"timestamp_ns": 123, "value": [1., 2.], "pending": None}
    assert differences(a, a) == []
    assert differences(a, {**a, "timestamp_ns": 124})[0]["reason"] == "discrete"
    assert differences([1., 2.], [1., 2.01])[0]["reason"] == "numeric"
    assert state_record(np.array([3])) == [3]
