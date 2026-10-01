"""Fail-closed PyTorch device selection and lightweight diagnostics."""
from __future__ import annotations

from dataclasses import dataclass

import torch


@dataclass(frozen=True)
class ComputeDevice:
    requested: str
    device: torch.device
    name: str
    cuda_available: bool


def resolve_compute_device(requested: str) -> ComputeDevice:
    """Resolve CPU/CUDA and prove that the requested accelerator is usable."""
    text = str(requested).strip().lower()
    if not text:
        raise ValueError("compute device must not be empty")
    try:
        device = torch.device(text)
    except (RuntimeError, ValueError) as error:
        raise ValueError(f"invalid compute device {requested!r}") from error
    if device.type not in {"cpu", "cuda"}:
        raise ValueError("compute device must be cpu, cuda, or cuda:<index>")
    if device.type == "cpu":
        return ComputeDevice(
            text, device, "CPU", bool(torch.cuda.is_available()))
    cuda_available = bool(torch.cuda.is_available())
    if not cuda_available:
        raise RuntimeError(
            "CUDA was requested but is unavailable to the cr-venv PyTorch "
            "runtime; verify nvidia-smi, the NVIDIA driver, and "
            "torch.cuda.is_available()")
    index = torch.cuda.current_device() if device.index is None else device.index
    if index < 0 or index >= torch.cuda.device_count():
        raise ValueError(
            f"CUDA device index {index} is outside the available range")
    resolved = torch.device("cuda", index)
    try:
        # Force context creation now. Do not defer a driver/access failure to
        # the first armed planner callback.
        torch.empty(1, device=resolved)
        torch.cuda.synchronize(resolved)
        name = torch.cuda.get_device_name(resolved)
    except (RuntimeError, AssertionError) as error:
        raise RuntimeError(
            f"CUDA device {resolved} could not execute a preflight operation") \
            from error
    return ComputeDevice(text, resolved, str(name), True)


def compute_device_diagnostics(info: ComputeDevice) -> dict[str, str]:
    """Return non-synchronizing diagnostic values for the active device."""
    values = {
        "compute_device_requested": info.requested,
        "compute_device": str(info.device),
        "compute_device_name": info.name,
        "cuda_available": str(info.cuda_available),
    }
    if info.device.type == "cuda":
        scale = 1024.0*1024.0
        values.update({
            "cuda_memory_allocated_mb": format(
                torch.cuda.memory_allocated(info.device)/scale, ".3f"),
            "cuda_memory_reserved_mb": format(
                torch.cuda.memory_reserved(info.device)/scale, ".3f"),
            "cuda_peak_memory_allocated_mb": format(
                torch.cuda.max_memory_allocated(info.device)/scale, ".3f"),
        })
    return values
