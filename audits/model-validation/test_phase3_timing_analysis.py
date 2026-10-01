"""Focused regression tests for the Phase-3 offline timing evaluator."""
from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np


SCRIPT = Path(__file__).with_name(
    "evaluate_causal_proximal_experiment.py")
SPEC = importlib.util.spec_from_file_location("causal_evaluator", SCRIPT)
EVALUATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EVALUATOR)


def test_valid_staggered_raw_waveform_passes_topology_gate():
    rows = []
    for tick in range(121):
        time_ns = tick * 10_000_000
        time_s = tick * 0.01
        rows.append({
            "timestamp_ns": time_ns,
            "raw": np.asarray([
                2.0 if time_s < 1.0 else 0.0,
                2.0 if 0.02 <= time_s < 1.02 else 0.0,
            ]),
        })
    result = EVALUATOR._phase3_command_waveform(
        rows, 0, 1_200_000_000, [0.0, 20.0], [1, 1], [2.0, 2.0],
        value_key="raw")
    assert result["valid"], result["violations"]
    assert result["pulse_regions"] == [1, 1]


def test_waveform_gate_rejects_interruption_and_inactive_axis_motion():
    rows = [
        {"timestamp_ns": tick * 10_000_000,
         "raw": np.asarray([
             2.0 if tick < 20 or 30 <= tick < 50 else 0.0,
             1.0,
         ])}
        for tick in range(61)
    ]
    result = EVALUATOR._phase3_command_waveform(
        rows, 0, 600_000_000, [0.0, -1.0], [1, 0], [0.4, 0.0],
        value_key="raw")
    assert not result["valid"]
    assert "axis_0_pulse_regions=2" in result["violations"]
    assert "axis_1_unexpected_motion" in result["violations"]


def test_encoder_onset_accounts_for_negative_tendon_encoder_polarity():
    rows = []
    for tick, count in enumerate((0, -1, -4, -8, -12)):
        data = np.zeros(6)
        data[2] = count
        rows.append({"timestamp_ns": tick * 20_000_000, "data": data})
    assert EVALUATOR._credible_encoder_onsets(
        rows, 0, 80_000_000, [0, 1], [3.0, 3.0]) == [
            None, 40_000_000]
