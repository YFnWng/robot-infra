import pytest
import torch

from catheter_control.orchestration.compute_device import (
    compute_device_diagnostics, resolve_compute_device)


def test_cpu_device_is_resolved_without_cuda():
    info = resolve_compute_device("cpu")

    assert info.device == torch.device("cpu")
    assert info.name == "CPU"
    diagnostics = compute_device_diagnostics(info)
    assert diagnostics["compute_device"] == "cpu"
    assert diagnostics["compute_device_name"] == "CPU"


def test_unknown_accelerator_is_rejected():
    with pytest.raises(ValueError, match="compute device must be"):
        resolve_compute_device("meta")


def test_unavailable_cuda_is_rejected_at_startup(monkeypatch):
    monkeypatch.setattr(torch.cuda, "is_available", lambda: False)

    with pytest.raises(RuntimeError, match="CUDA was requested"):
        resolve_compute_device("cuda")
