import numpy as np
import pytest

from automation.collection.identification import (
    IdentificationConfig, IdentificationGenerator)


LOWER = np.array([0.0, -180.0, 0.0])
UPPER = np.array([40.0, 180.0, 10.0])
VMIN = np.array([2.0, 7.0, 1.0])
VMAX = np.array([10.0, 40.0, 1.0])


def make_generator(**config_overrides):
    config = IdentificationConfig(**config_overrides)
    return IdentificationGenerator(
        [5.0, 0.0, 1.0], LOWER, UPPER, VMIN, VMAX, 0.02, config)


def test_plan_has_expected_episode_order_and_is_deterministic():
    first = make_generator(seed=7)
    second = make_generator(seed=7)
    assert len(first.episodes) == 22
    assert first.episode_names[0] == "settle_start"
    assert first.episode_names[-1] == "settle_end"
    assert first.episode_names == second.episode_names
    times = np.linspace(0.0, first.duration, 1000)
    assert np.allclose(
        [first.relative_position(t) for t in times],
        [second.relative_position(t) for t in times])


def test_plan_is_bounded_and_starts_and_finishes_at_rest():
    generator = make_generator()
    times = np.arange(0.0, generator.duration + 0.01, 0.01)
    position = np.asarray([generator.relative_position(t) for t in times])
    velocity = np.asarray([generator.relative_velocity(t) for t in times])
    absolute = position + generator.start_position
    assert np.all(absolute >= LOWER - 1e-9)
    assert np.all(absolute <= UPPER + 1e-9)
    assert np.all(np.abs(velocity) <= VMAX + 1e-7)
    assert np.allclose(generator.state(0.0)[0:2], 0.0)
    assert np.allclose(generator.state(generator.duration)[0:2], 0.0)


def test_mechanical_zero_is_valid_for_one_sided_axes():
    generator = IdentificationGenerator(
        [0.0, 0.0, 0.0], LOWER, UPPER, VMIN, VMAX, 0.02)
    assert np.allclose(generator.amplitudes, [20.0, 100.0, 6.0])


def test_every_episode_has_continuous_position_velocity_and_acceleration():
    generator = make_generator()
    for episode in generator.episodes:
        start = episode.state(0.0)
        end = episode.state(episode.duration_s)
        for derivative in range(3):
            assert np.allclose(start[derivative], 0.0, atol=1e-8)
            assert np.allclose(end[derivative], 0.0, atol=1e-8)


def test_rate_pair_uses_same_normalized_insertion_path():
    generator = make_generator()
    slow = next(ep for ep in generator.episodes if ep.name == "insertion_slow")
    medium = next(ep for ep in generator.episodes if ep.name == "insertion_medium")
    for fraction in np.linspace(0.0, 1.0, 51):
        assert np.allclose(
            slow.state(fraction * slow.duration_s)[0],
            medium.state(fraction * medium.duration_s)[0])


def test_amplitude_is_reduced_from_run_start_and_too_little_room_rejects():
    reduced = IdentificationGenerator(
        [30.0, 0.0, 1.0], LOWER, UPPER, VMIN, VMAX, 0.02)
    assert reduced.amplitudes[0] == pytest.approx(9.5)
    with pytest.raises(ValueError, match="insufficient safe excursion"):
        IdentificationGenerator(
            [38.5, 0.0, 1.0], LOWER, UPPER, VMIN, VMAX, 0.02)


def test_bend_has_only_one_rate_for_equal_minimum_and_maximum_speed():
    generator = make_generator()
    assert not generator.medium_enabled[2]
    assert "bend_medium" not in generator.episode_names
    assert "bend_medium_skipped" in generator.episode_names


def test_direct_drive_bend_rate_pair_uses_same_normalized_path():
    generator = IdentificationGenerator(
        [5.0, 0.0, 1.0], LOWER, UPPER,
        [2.0, 7.0, 2.0], [10.0, 40.0, 4.9], 0.02)
    assert generator.medium_enabled[2]
    assert "bend_medium" in generator.episode_names
    assert "bend_medium_skipped" not in generator.episode_names

    slow = next(ep for ep in generator.episodes
                if ep.name == "bend_out_and_back")
    medium = next(ep for ep in generator.episodes
                  if ep.name == "bend_medium")
    for fraction in np.linspace(0.0, 1.0, 51):
        assert np.allclose(
            slow.state(fraction * slow.duration_s)[0],
            medium.state(fraction * medium.duration_s)[0])

    limits = generator.command_speed_limits(
        medium.start_s + 0.25 * medium.duration_s)
    assert np.allclose(limits, [0.0, 0.0, 3.43])


def test_command_ceiling_is_zero_in_dwells_and_on_stationary_axes():
    generator = make_generator()
    dwell = next(ep for ep in generator.episodes
                 if ep.name == "dwell_after_insertion_slow")
    assert np.allclose(
        generator.command_speed_limits(dwell.start_s + 0.5 * dwell.duration_s),
        [0.0, 0.0, 0.0])

    rotation = next(ep for ep in generator.episodes
                    if ep.name == "rotation_slow")
    limits = generator.command_speed_limits(
        rotation.start_s + 0.25 * rotation.duration_s)
    assert np.allclose(limits, [0.0, 12.0, 0.0])


def test_internal_hold_has_zero_command_ceiling():
    generator = make_generator()
    episode = next(ep for ep in generator.episodes
                   if ep.name == "bend_hold_unload_relax")
    first_move_duration = episode.segments[0].duration
    assert episode.segments[1].kind == "hold"
    limits = generator.command_speed_limits(
        episode.start_s + first_move_duration
        + 0.5 * episode.segments[1].duration)
    assert np.allclose(limits, 0.0)
