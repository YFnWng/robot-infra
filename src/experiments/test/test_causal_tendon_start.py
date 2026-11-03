"""tendon_start schedule: raw-shaft topology, coverage, bounds and manager-projection invariance."""
from collections import Counter
from dataclasses import replace

import numpy as np
import pytest

from experiments.causal_experiment import (
    CausalExperimentConfig, CausalExperimentGenerator)

START = np.array([20.0, 0.0, 0.0])
LOWER, UPPER = [-10.0, -270.0, 0.0], [50.0, 270.0, 15.0]
VMIN, VMAX = [2.0, 7.0, 2.0], [10.0, 40.0, 4.5]  # imricor_test reliable band
DT = 0.01


def generator(**changes):
    config = replace(CausalExperimentConfig(schedule="tendon_start"), **changes)
    return CausalExperimentGenerator(START, LOWER, UPPER, VMIN, VMAX, DT, config)


def pulses(gen, prefix="tendon_start"):
    return [e for e in gen.episodes if e.timing_raw_profile is not None
            and e.excitation_basis.startswith(prefix)]


def test_plan_is_short_starts_and_ends_at_rest_and_holds_the_bend_band():
    gen = generator()
    assert gen.duration < 200.0
    for t in (0.0, gen.duration - DT):
        position, velocity, _ = gen.state(t)
        np.testing.assert_allclose(position, 0.0, atol=1e-9)
        np.testing.assert_allclose(velocity, 0.0, atol=1e-12)
    for episode in pulses(gen):
        for t in np.arange(0.0, episode.duration_s, DT):
            bend = episode.state(t)[0][2] + START[2]
            assert 0.5 - 1e-9 <= bend <= 10.7 + 1e-9, episode.name


@pytest.mark.parametrize("basis", ["tendon_start", "tendon_start_dwell",
                                   "tendon_start_speed_change"])
def test_tendon_only_pulses_hold_raw_shaft_zero(basis):
    gen = generator()
    selected = [e for e in pulses(gen) if e.excitation_basis == basis]
    assert selected
    for episode in selected:
        assert all(segment[1] == 0.0 for segment in episode.timing_raw_profile)
        for t in np.arange(0.0, episode.duration_s, DT):
            raw0, raw2 = episode.timing_raw_velocity(t)
            assert raw0 == 0.0
            position, velocity, _ = episode.state(t)
            assert velocity[0] == velocity[2]  # logical insertion follows bend


def test_every_position_covers_both_directions_speeds_and_histories():
    gen = generator()
    cells = Counter()
    previous = 0
    for episode in gen.episodes:
        if episode.excitation_basis != "tendon_start":
            moving = [] if episode.timing_raw_profile is None else [
                s[2] for s in episode.timing_raw_profile if s[2]]
            if moving:
                previous = int(np.sign(moving[-1]))
            continue
        speed = [s[2] for s in episode.timing_raw_profile if s[2]][0]
        sign = int(np.sign(speed))
        position = episode.name.split("_")[2]  # block index p0, p1, p2
        if previous and position.startswith("p"):
            cells[(position, abs(speed), sign, sign == previous)] += 1
        previous = sign
    positions = {key[0] for key in cells}
    assert len(positions) >= 3
    for position in positions:
        for speed in (2.0, 4.5):
            for sign in (-1, 1):
                for same in (True, False):
                    assert cells[(position, speed, sign, same)] >= 2, (
                        position, speed, sign, same)


def test_middle_blocks_profiles_and_planned_onsets():
    gen = generator()
    change = [e for e in pulses(gen) if e.excitation_basis == "tendon_start_speed_change"]
    assert len(change) == 6
    speeds = [abs(s[2]) for s in change[0].timing_raw_profile]
    assert speeds == [0.0, 2.0, 4.0, 2.0]
    for lead_ms in (0, 40, 80):
        combined = [e for e in pulses(gen)
                    if e.excitation_basis == f"tendon_start_combined_lead_{lead_ms:03d}ms"]
        assert len(combined) == 8
        for episode in combined:
            raw0_onset, raw2_onset = episode.planned_raw_onset_delays_s
            assert raw2_onset == pytest.approx(0.3)
            assert raw0_onset == pytest.approx(0.3 + lead_ms*1e-3)
            # Both shafts move for one pulse length; tendon starts and stops first.
            travel = np.subtract(episode.timing_raw_end, episode.timing_raw_start)
            raw0, raw2 = [s for s in episode.timing_raw_profile if s[1] and s[2]][0][1:]
            np.testing.assert_allclose(travel, [0.3*raw0, 0.3*raw2], atol=1e-12)
    insertion = [e for e in pulses(gen) if e.excitation_basis == "tendon_start_insertion"]
    assert len(insertion) == 12
    assert all(s[2] == 0.0 for e in insertion for s in e.timing_raw_profile)


def test_manager_projection_leaves_every_raw_command_unchanged():
    contract_module = pytest.importorskip("catheter_control.safety.hardware_contract")
    from pathlib import Path
    limits = Path(__file__).resolve().parents[2]/"control_interface/config/catheter_limits.yaml"
    contract = contract_module.load_hardware_contract(str(limits), "imricor_test")
    gen = CausalExperimentGenerator(START, LOWER, UPPER, contract.velocity_min[:3],
        contract.velocity_max[:3], DT, CausalExperimentConfig(schedule="tendon_start"))
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
    assert checked > 10000


@pytest.mark.parametrize("changes, message", [
    (dict(tendon_start_speeds=(1.0, 4.5)), "reliable band"),
    (dict(tendon_start_speeds=(2.0, 5.0)), "reliable band"),
    (dict(tendon_start_positions=(2.0, 6.0, 10.75)), "outside"),
])
def test_out_of_band_speeds_or_positions_are_rejected(changes, message):
    with pytest.raises(ValueError, match=message):
        generator(**changes)


def test_existing_timing_schedule_has_no_profiles():
    gen = CausalExperimentGenerator(START, LOWER, UPPER, VMIN, VMAX, DT,
                                    CausalExperimentConfig(schedule="timing"))
    assert all(e.timing_raw_profile is None for e in gen.episodes)
    assert "raw_profile" in gen.metadata["episodes"][0]
