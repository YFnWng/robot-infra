"""tendon_reversal schedule: on-the-fly reversals, references, bounds and manager-projection invariance."""
from collections import Counter

import numpy as np
import pytest

from experiments.causal_experiment import (
    CausalExperimentConfig, CausalExperimentGenerator)

START = np.array([20.0, 0.0, 0.0])
LOWER, UPPER = [-10.0, -270.0, 0.0], [50.0, 270.0, 15.0]
VMIN, VMAX = [2.0, 7.0, 2.0], [10.0, 40.0, 4.5]  # imricor_test reliable band
DT = 0.01


def generator():
    return CausalExperimentGenerator(START, LOWER, UPPER, VMIN, VMAX, DT,
                                     CausalExperimentConfig(schedule="tendon_reversal"))


def by_basis(gen, basis):
    return [e for e in gen.episodes if e.excitation_basis == basis]


def moving(profile, axis):
    """Nonzero raw speeds of one shaft (1: shaft 0, 2: shaft 2) in profile order."""
    return [s[axis] for s in profile if s[axis] != 0.0]


def test_plan_is_short_starts_and_ends_at_rest_and_holds_the_bend_band():
    gen = generator()
    assert gen.duration < 90.0
    for t in (0.0, gen.duration - DT):
        position, velocity, _ = gen.state(t)
        np.testing.assert_allclose(position, 0.0, atol=1e-9)
        np.testing.assert_allclose(velocity, 0.0, atol=1e-12)
    for episode in gen.episodes:
        if episode.timing_raw_profile is None:
            continue
        for t in np.arange(0.0, episode.duration_s, DT):
            bend = episode.state(t)[0][2] + START[2]
            assert 0.5 - 1e-9 <= bend <= 10.7 + 1e-9, episode.name


def test_block_sizes_and_reversal_is_a_direct_sign_change():
    gen = generator()
    counts = Counter(e.excitation_basis for e in gen.episodes)
    assert counts["tendon_reversal"] == 12 and counts["tendon_reversal_rest"] == 12
    assert counts["tendon_reversal_speed_change"] == 12
    assert counts["tendon_reversal_insertion"] == 12
    assert counts["tendon_reversal_with_insertion"] == 8
    for basis, axis in (("tendon_reversal", 2), ("tendon_reversal_speed_change", 2),
                        ("tendon_reversal_insertion", 1), ("tendon_reversal_with_insertion", 2)):
        for episode in by_basis(gen, basis):
            profile = episode.timing_raw_profile
            # rest, then two adjacent segments of opposite sign: no zero command between
            assert profile[0][1:] == (0.0, 0.0)
            assert len(profile) == 3 and profile[1][axis]*profile[2][axis] < 0, episode.name
    for episode in by_basis(gen, "tendon_reversal_rest"):
        profile = episode.timing_raw_profile
        assert [s[2] for s in profile][2] == 0.0  # matched reference rests between
        assert profile[1][2] == -profile[3][2]


def test_held_shafts_and_constant_insertion_during_tendon_reversal():
    gen = generator()
    for basis in ("tendon_reversal", "tendon_reversal_rest", "tendon_reversal_speed_change"):
        for episode in by_basis(gen, basis):
            assert all(s[1] == 0.0 for s in episode.timing_raw_profile)
    for episode in by_basis(gen, "tendon_reversal_insertion"):
        assert all(s[2] == 0.0 for s in episode.timing_raw_profile)
    for episode in by_basis(gen, "tendon_reversal_with_insertion"):
        shaft0 = moving(episode.timing_raw_profile, 1)
        assert len(shaft0) == 2 and shaft0[0] == shaft0[1]  # insertion keeps running


def test_speeds_cover_both_directions_and_tendon_returns_per_episode():
    gen = generator()
    cells = Counter()
    for episode in by_basis(gen, "tendon_reversal"):
        first, second = moving(episode.timing_raw_profile, 2)
        cells[(abs(first), np.sign(first))] += 1
        assert first == -second
        np.testing.assert_allclose(episode.timing_raw_end[1], episode.timing_raw_start[1], atol=1e-12)
    assert set(cells) == {(2.0, 1), (2.0, -1), (4.5, 1), (4.5, -1)}
    assert all(n == 3 for n in cells.values())
    changes = by_basis(gen, "tendon_reversal_speed_change")
    pairs = Counter(tuple(abs(v) for v in moving(e.timing_raw_profile, 2)) for e in changes)
    assert pairs == {(2.0, 4.5): 6, (4.5, 2.0): 6}


def test_manager_projection_leaves_every_raw_command_unchanged():
    contract_module = pytest.importorskip("catheter_control.safety.hardware_contract")
    from pathlib import Path
    limits = Path(__file__).resolve().parents[2]/"control_interface/config/catheter_limits.yaml"
    contract = contract_module.load_hardware_contract(str(limits), "imricor_test")
    gen = CausalExperimentGenerator(START, LOWER, UPPER, contract.velocity_min[:3],
        contract.velocity_max[:3], DT, CausalExperimentConfig(schedule="tendon_reversal"))
    checked = 0
    for t in np.arange(0.0, gen.duration, DT):
        raw = gen.timing_raw_velocity(t)
        if raw is None:
            continue
        position, velocity, _ = gen.state(t)
        command = np.zeros(6);command[:3] = velocity
        full = np.zeros(6);full[:3] = START + position
        projected = contract.project_logical_velocity(command, full)
        for value in (command, projected):
            np.testing.assert_allclose(
                np.asarray(contract.logical_to_motor_axis_velocity(value))[[0, 2]], raw,
                atol=1e-9, rtol=0)
        checked += 1
    assert checked > 5000
