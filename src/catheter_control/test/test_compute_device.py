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


@pytest.mark.skipif(not torch.cuda.is_available(), reason="CUDA required")
def test_cuda_memory_diagnostics_match_torch_and_leave_no_cyclic_garbage():
    import gc
    from catheter_control.orchestration.compute_device import compute_device_diagnostics, resolve_compute_device
    info = resolve_compute_device("cuda")
    buffer = torch.zeros(4096, device=info.device)   # allocator initialised, as after warm-up
    values = compute_device_diagnostics(info)
    scale = 1024.0*1024.0
    assert values["cuda_memory_allocated_mb"] == format(torch.cuda.memory_allocated(info.device)/scale, ".3f")
    assert values["cuda_memory_reserved_mb"] == format(torch.cuda.memory_reserved(info.device)/scale, ".3f")
    assert values["cuda_peak_memory_allocated_mb"] == format(torch.cuda.max_memory_allocated(info.device)/scale, ".3f")
    # The armed controller defers cyclic GC: a diagnostic must be freed by reference counting alone.
    gc.collect()
    gc.disable()
    try:
        for _ in range(50):
            compute_device_diagnostics(info)
        assert gc.collect() == 0
    finally:
        gc.enable()
    del buffer
