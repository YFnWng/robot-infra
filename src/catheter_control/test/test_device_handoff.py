from types import SimpleNamespace

import pytest
import torch

from catheter_control.node import CatheterControlNode


def test_same_device_preserves_existing_snapshot_path():
    root = SimpleNamespace(motor_angle_rad=torch.zeros(3))
    node = SimpleNamespace(planner_runtime=SimpleNamespace(device=torch.device("cpu"), dtype=torch.float32))
    assert CatheterControlNode._planner_root_on_device(node, root) is root


@pytest.mark.parametrize("fail", [False, True])
def test_split_handoff_is_once_and_timed_even_on_failure(fail):
    copies, timings = [], []
    result = object()
    def clone_to(device):
        copies.append(device)
        if fail:
            raise RuntimeError("copy failed")
        return result
    clock = iter([1., 1.002])
    node = SimpleNamespace(
        planner_runtime=SimpleNamespace(device=torch.device("cuda:0"), dtype=torch.float32),
        _steady=lambda: next(clock),
        _timing=SimpleNamespace(record_seconds=lambda *args: timings.append(args)))
    root = SimpleNamespace(motor_angle_rad=torch.zeros(3), clone_to=clone_to)
    if fail:
        with pytest.raises(RuntimeError, match="copy failed"):
            CatheterControlNode._planner_root_on_device(node, root)
    else:
        assert CatheterControlNode._planner_root_on_device(node, root) is result
    assert copies == [torch.device("cuda:0")]
    assert timings[0][0] == "plan_snapshot_transfer"
    assert timings[0][1] == pytest.approx(.002)


def test_same_device_mixed_precision_snapshot_is_cast_once():
    copies = []
    root = SimpleNamespace(motor_angle_rad=torch.zeros(3, dtype=torch.float64),
                           clone_to=lambda device, **kwargs: copies.append((device, kwargs)))
    node = SimpleNamespace(planner_runtime=SimpleNamespace(device=torch.device("cpu"), dtype=torch.float32),
                           _steady=lambda: 1., _timing=SimpleNamespace(record_seconds=lambda *args: None))
    CatheterControlNode._planner_root_on_device(node, root)
    assert copies == [(torch.device("cpu"), {"dtype": torch.float32})]
