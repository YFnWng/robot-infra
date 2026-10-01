from pathlib import Path

import numpy as np
import pytest

from automation.collection.causal_experiment import (
    CausalExperimentConfig, CausalExperimentGenerator,
    resolve_tolerance_qualified_start)


LOWER = np.array([-10.0, -270.0, 0.0])
UPPER = np.array([50.0, 270.0, 15.0])
VMIN = np.array([2.0, 7.0, 2.0])
VMAX = np.array([10.0, 40.0, 4.5])


def make_generator(start=(20.0, 0.0, 0.0), **overrides):
    return CausalExperimentGenerator(
        start, LOWER, UPPER, VMIN, VMAX, 0.01,
        CausalExperimentConfig(**overrides))


def test_five_blocks_are_ordered_and_end_at_run_start():
    generator = make_generator()
    names = generator.episode_names
    assert names[0] == "static_start"
    assert names[-1] == "static_end"
    assert any(name.startswith("insertion_") for name in names)
    assert any(name.startswith("rotation_") for name in names)
    assert any(name.startswith("bend_motor_only_") for name in names)
    assert any(name.startswith("bend_compensated_") for name in names)
    assert "mixed_validation" in names
    assert np.allclose(generator.relative_position(0.0), 0.0)
    assert np.allclose(generator.relative_position(generator.duration), 0.0)


def test_plan_is_continuous_and_inside_hard_limits():
    generator = make_generator()
    for left, right in zip(generator.episodes[:-1], generator.episodes[1:]):
        assert np.allclose(
            left.state(left.duration_s)[0], right.state(0.0)[0])
        assert np.allclose(
            left.state(left.duration_s)[1], right.state(0.0)[1])
    times = np.arange(0.0, generator.duration + 0.02, 0.02)
    position = generator.start_position + np.asarray([
        generator.relative_position(t) for t in times])
    assert np.all(position >= LOWER - 1e-9)
    assert np.all(position <= UPPER + 1e-9)


def test_bend_motor_only_preserves_exact_firmware_cancellation():
    generator = make_generator()
    for episode in generator.episodes:
        if episode.excitation_basis != "shaft_2":
            continue
        times = np.linspace(
            episode.start_s, episode.start_s + episode.duration_s, 101)
        for t in times[:-1]:
            shaped = generator.relative_velocity(t) + np.array(
                [0.2, 0.0, -0.1])
            command = generator.enforce_basis(t, shaped)
            assert command[0] == command[2]
            assert command[0] - command[2] == 0.0


def test_compensated_bend_is_labelled_validation_not_single_column():
    generator = make_generator()
    episodes = [episode for episode in generator.episodes
                if episode.name.startswith("bend_compensated_")]
    assert episodes
    assert all(episode.excitation_basis == "shaft_0_plus_2_validation"
               for episode in episodes)


def test_amplitudes_reduce_explicitly_and_reject_below_minimum():
    reduced = make_generator(
        start=(42.0, 0.0, 0.0), margins=(3.0, 20.0, 1.0))
    assert reduced.amplitudes[0] == pytest.approx(5.0)
    assert reduced.metadata["resolved_amplitudes"][0] == pytest.approx(5.0)
    with pytest.raises(ValueError, match="insufficient margin-qualified"):
        make_generator(
            start=(42.1, 0.0, 0.0), margins=(3.0, 20.0, 1.0))


def test_default_insertion_envelope_is_far_from_historical_boundaries():
    generator = make_generator()
    assert generator.usable_lower[0] == pytest.approx(5.0)
    assert generator.usable_upper[0] == pytest.approx(35.0)


def test_simulation_can_center_from_zero_without_weakening_experiment_margin():
    generator = make_generator(
        start=(0.0, 0.0, 0.0), allow_insertion_centering=True,
        insertion_center_position=20.0)
    assert generator.episode_names[0] == "insertion_center_enter"
    assert generator.episode_names[-1] == "insertion_center_exit"
    assert np.allclose(generator.experiment_center, [20.0, 0.0, 0.0])
    assert generator.usable_lower[0] == pytest.approx(5.0)
    assert generator.usable_upper[0] == pytest.approx(35.0)
    assert np.allclose(generator.relative_position(0.0), 0.0)
    assert np.allclose(generator.relative_position(generator.duration), 0.0)


def test_zero_start_without_explicit_centering_remains_rejected():
    with pytest.raises(ValueError, match="insertion center is outside"):
        make_generator(start=(0.0, 0.0, 0.0))


def test_default_plan_has_two_speeds_three_repetitions():
    generator = make_generator()
    assert np.allclose(generator.amplitudes, [6.0, 75.0, 5.5])
    assert generator.config.minimum_amplitudes == (5.0, 65.0, 4.75)
    assert generator.config.bend_bias_position == pytest.approx(7.5)
    assert generator.duration < generator.config.max_duration_s
    for prefix in ("insertion", "rotation", "bend_motor_only",
                   "bend_compensated"):
        selected = [episode for episode in generator.episodes
                    if episode.name.startswith(prefix + "_")]
        assert len(selected) == 6
        assert {episode.speed_tier for episode in selected} == {"slow", "fast"}
        assert {episode.repetition for episode in selected} == {1, 2, 3}


def test_tolerance_qualified_start_is_clipped_to_exact_command_limits():
    measured = np.array([19.998298645, 0.0, -0.00074414065])
    resolved = resolve_tolerance_qualified_start(
        measured, LOWER, UPPER, [0.001, 0.01, 0.002])
    assert np.allclose(resolved, [19.998298645, 0.0, 0.0])
    generator = CausalExperimentGenerator(
        resolved, LOWER, UPPER, VMIN, VMAX, 0.01)
    assert np.all(generator.start_position >= LOWER)


def test_tolerance_qualified_start_rejects_a_real_limit_violation():
    with pytest.raises(ValueError, match="axis=2"):
        resolve_tolerance_qualified_start(
            [20.0, 0.0, -0.0021], LOWER, UPPER,
            [0.001, 0.01, 0.002])


def test_stationary_schedule_never_centers_or_commands_motion():
    generator = make_generator(
        start=(0.0, 0.0, 0.0), schedule="stationary",
        allow_insertion_centering=True, static_s=10.0)
    assert generator.episode_names == ["stationary_noise"]
    assert generator.duration == pytest.approx(10.0)
    assert np.allclose(generator.experiment_center, [0.0, 0.0, 0.0])
    for time_s in np.linspace(0.0, generator.duration, 20):
        assert np.allclose(generator.relative_position(time_s), 0.0)
        assert np.allclose(generator.relative_velocity(time_s), 0.0)


@pytest.mark.parametrize(
    "schedule,required_prefix,forbidden_prefixes",
    [
        ("insertion", "insertion_",
         ("rotation_", "bend_motor_only_", "bend_compensated_")),
        ("tendon_motor", "bend_motor_only_",
         ("insertion_", "rotation_", "bend_compensated_")),
        ("compensated_bend", "bend_compensated_",
         ("insertion_", "rotation_", "bend_motor_only_")),
    ])
def test_single_basis_schedules_are_selective(
        schedule, required_prefix, forbidden_prefixes):
    generator = make_generator(schedule=schedule)
    names = generator.episode_names
    assert any(name.startswith(required_prefix) for name in names)
    for prefix in forbidden_prefixes:
        assert not any(name.startswith(prefix) for name in names)
    assert "mixed_validation" not in names
    assert np.allclose(generator.relative_position(generator.duration), 0.0)


def test_phase_0_2_excludes_rotation_and_mixed_validation():
    generator = make_generator(schedule="phase_0_2")
    names = generator.episode_names
    assert any(name.startswith("insertion_") for name in names)
    assert any(name.startswith("bend_motor_only_") for name in names)
    assert any(name.startswith("bend_compensated_") for name in names)
    assert not any(name.startswith("rotation_") for name in names)
    assert "mixed_validation" not in names


def test_phase_aliases_resolve_to_named_schedule():
    assert make_generator(schedule="phase_1").schedule == "stationary"
    assert make_generator(schedule="phase_2a").schedule == "insertion"
    assert make_generator(schedule="phase_2b").schedule == "tendon_motor"
    assert make_generator(schedule="phase_2c").schedule == "compensated_bend"
    assert make_generator(schedule="phase_3").schedule == "timing"
    assert make_generator(
        schedule="phase_2d").schedule == "insertion_rotation"
    assert make_generator(
        schedule="phase_2e").schedule == (
            "compensated_bend_insertion_sweep")
    assert make_generator(
        schedule="backdrive").schedule == "chassis_knob_backdrive"


def test_insertion_stratified_bend_has_two_balanced_visits_per_level():
    generator = make_generator(schedule="phase_2e")
    measured = [
        episode for episode in generator.episodes
        if episode.excitation_basis == "compensated_bend_at_insertion"]
    assert len(measured) == 8
    expected = (0.0, 40.0 / 3.0, 80.0 / 3.0, 40.0)
    for plateau in expected:
        selected = [episode for episode in measured
                    if episode.insertion_plateau_mm == pytest.approx(plateau)]
        assert len(selected) == 2
        assert {episode.speed_tier for episode in selected} == {"slow", "fast"}
        assert {episode.branch_order for episode in selected} == {
            "high_first", "low_first"}
        assert {episode.repetition for episode in selected} == {1, 2}
    assert [episode.insertion_plateau_mm for episode in measured[:4]] == (
        pytest.approx(expected))
    assert [episode.insertion_plateau_mm for episode in measured[4:]] == (
        pytest.approx(tuple(reversed(expected))))


def test_insertion_stratified_bend_holds_insertion_and_sweeps_full_range():
    generator = make_generator(schedule="compensated_bend_insertion_sweep")
    measured = [
        episode for episode in generator.episodes
        if episode.excitation_basis == "compensated_bend_at_insertion"]
    for episode in measured:
        samples = np.linspace(0.0, episode.duration_s, 301)
        absolute = generator.start_position + np.asarray([
            episode.state(value)[0] for value in samples])
        assert np.allclose(absolute[:, 0], episode.insertion_plateau_mm)
        assert np.allclose(absolute[:, 1], 0.0)
        assert absolute[:, 2].min() == pytest.approx(0.0, abs=2e-3)
        assert absolute[:, 2].max() == pytest.approx(15.0, abs=2e-3)

        midpoint = episode.start_s + 0.5 * episode.segments[0].duration
        requested = generator.relative_velocity(midpoint)
        shaped = requested + np.array([0.4, 3.0, -0.2])
        enforced = generator.enforce_basis(midpoint, shaped)
        assert enforced[0] == 0.0
        assert enforced[1] == 0.0
        assert enforced[2] != 0.0


def test_insertion_stratified_bend_duration_and_metadata_are_exact():
    generator = make_generator(schedule="phase_2e")
    assert generator.duration == pytest.approx(413.3125)
    assert np.allclose(generator.amplitudes, [20.0, 0.0, 7.5])
    assert generator.usable_lower[0] == pytest.approx(LOWER[0])
    assert generator.usable_upper[0] == pytest.approx(UPPER[0])
    assert generator.usable_lower[2] == pytest.approx(LOWER[2])
    assert generator.usable_upper[2] == pytest.approx(UPPER[2])
    assert generator.metadata["insertion_plateau_visits"] == 2
    assert generator.metadata["tendon_sweep_limits"] == [0.0, 15.0]
    assert generator.metadata["duration_s"] == pytest.approx(413.3125)
    assert np.allclose(generator.relative_position(generator.duration), 0.0)


def test_insertion_stratified_bend_rejects_invalid_design_parameters():
    with pytest.raises(ValueError, match="four strictly increasing"):
        make_generator(schedule="phase_2e", insertion_plateaus=(0.0, 20.0, 40.0))
    with pytest.raises(ValueError, match="must equal two"):
        make_generator(schedule="phase_2e", insertion_plateau_visits=3)
    with pytest.raises(ValueError, match="inside bend hard limits"):
        make_generator(schedule="phase_2e", tendon_sweep_limits=(-0.1, 15.0))


def test_insertion_rotation_schedule_is_balanced_and_returns_home():
    generator = make_generator(schedule="insertion_rotation")
    probes = [episode for episode in generator.episodes
              if episode.excitation_basis == "shaft_0_at_rotation_bias"]
    setups = [episode for episode in generator.episodes
              if episode.excitation_basis == "shaft_1_coupling_setup"]
    assert len(probes) == 2 * 3 * 2
    assert len(setups) == 2 * len(probes)
    assert {episode.speed_tier for episode in probes} == {"slow", "fast"}
    assert {episode.repetition for episode in probes} == {1, 2, 3}
    assert sum("_pos_" in episode.name for episode in probes) == 6
    assert sum("_neg_" in episode.name for episode in probes) == 6
    assert generator.episode_names[0] == "static_start"
    assert generator.episode_names[-1] == "static_end"
    assert np.allclose(generator.relative_position(generator.duration), 0.0)
    assert generator.duration < generator.config.max_duration_s


def test_insertion_rotation_probe_holds_commanded_rotation_and_bend():
    generator = make_generator(schedule="insertion_rotation")
    for episode in generator.episodes:
        if episode.excitation_basis != "shaft_0_at_rotation_bias":
            continue
        start = episode.state(0.0)[0]
        assert abs(start[1]) == pytest.approx(generator.amplitudes[1])
        assert start[2] == pytest.approx(0.0)
        samples = np.linspace(0.0, episode.duration_s, 101)
        positions = np.asarray([episode.state(t)[0] for t in samples])
        assert np.allclose(positions[:, 1], start[1])
        assert np.allclose(positions[:, 2], 0.0)
        assert positions[:, 0].max() == pytest.approx(
            generator.amplitudes[0], abs=2e-3)
        assert positions[:, 0].min() == pytest.approx(
            -generator.amplitudes[0], abs=2e-3)

        midpoint = episode.start_s + 0.5 * episode.segments[0].duration
        requested = generator.relative_velocity(midpoint)
        shaped = requested + np.array([0.3, 4.0, -0.2])
        enforced = generator.enforce_basis(midpoint, shaped)
        assert enforced[0] != 0.0
        assert enforced[1] == 0.0
        assert enforced[2] == 0.0


def test_insertion_rotation_setup_is_rotation_only_after_shaping():
    generator = make_generator(schedule="insertion_rotation")
    episode = next(
        item for item in generator.episodes
        if item.excitation_basis == "shaft_1_coupling_setup")
    midpoint = episode.start_s + 0.5 * episode.segments[0].duration
    requested = generator.relative_velocity(midpoint)
    shaped = requested + np.array([0.3, -1.0, -0.2])
    enforced = generator.enforce_basis(midpoint, shaped)
    assert enforced[0] == 0.0
    assert enforced[1] != 0.0
    assert enforced[2] == 0.0


def test_chassis_knob_backdrive_probe_has_four_ordered_moves():
    generator = make_generator(
        schedule="chassis_knob_backdrive",
        amplitudes=(10.0, 0.0, 5.0),
        minimum_amplitudes=(8.0, 0.0, 4.0),
        static_s=3.0, endpoint_dwell_s=4.0)
    assert generator.episode_names == [
        "backdrive_static_start",
        "backdrive_chassis_backward",
        "backdrive_knob_forward_bend",
        "backdrive_chassis_forward",
        "backdrive_knob_backward_relax",
        "backdrive_static_end",
    ]
    expected_endpoints = [
        [0.0, 0.0, 0.0],
        [-10.0, 0.0, 0.0],
        [-5.0, 0.0, 5.0],
        [5.0, 0.0, 5.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
    ]
    for episode, expected in zip(generator.episodes, expected_endpoints):
        assert np.allclose(
            episode.state(episode.duration_s)[0], expected)


def test_backdrive_knob_moves_command_zero_raw_chassis_velocity():
    generator = make_generator(
        schedule="backdrive", amplitudes=(10.0, 0.0, 5.0),
        minimum_amplitudes=(8.0, 0.0, 4.0), static_s=3.0)
    selected = [episode for episode in generator.episodes
                if episode.excitation_basis == "shaft_2_backdrive_probe"]
    assert len(selected) == 2
    for episode in selected:
        time_s = episode.start_s + 0.5 * episode.segments[0].duration
        requested = generator.relative_velocity(time_s)
        shaped = requested + np.array([0.4, 0.0, -0.2])
        enforced = generator.enforce_basis(time_s, shaped)
        assert enforced[0] == enforced[2]
        assert enforced[0] - enforced[2] == 0.0


def test_backdrive_probe_stays_inside_hard_limits_and_returns_home():
    generator = make_generator(
        schedule="backdrive", amplitudes=(5.0, 0.0, 5.0),
        minimum_amplitudes=(4.0, 0.0, 4.0), static_s=3.0)
    time = np.arange(0.0, generator.duration + .01, .01)
    absolute = generator.start_position + np.asarray([
        generator.relative_position(value) for value in time])
    assert np.all(absolute >= LOWER-1e-9)
    assert np.all(absolute <= UPPER+1e-9)
    assert np.allclose(generator.relative_position(generator.duration), 0.0)


def test_phase_3_builds_balanced_single_publisher_timing_trials():
    generator = make_generator(schedule="timing")
    measured = [episode for episode in generator.episodes
                if episode.is_timing_episode]
    assert len(measured) == 9 * 2 * 3
    assert generator.duration < generator.config.max_duration_s
    assert {episode.speed_tier for episode in measured} == {"slow", "fast"}
    assert {episode.repetition for episode in measured} == {1, 2, 3}
    names = {episode.name for episode in measured}
    for delay in (20, 40, 80):
        assert any(f"insertion_lead_{delay:03d}ms" in name for name in names)
        assert any(f"tendon_lead_{delay:03d}ms" in name for name in names)
    assert all(name.endswith("_pos") for name in names)
    assert not any(name.startswith("rotation_") for name in generator.episode_names)


def test_phase_3_stagger_keeps_delayed_raw_shaft_zero():
    generator = make_generator(schedule="timing")
    episode = next(
        item for item in generator.episodes
        if "insertion_lead_080ms_slow_rep1" in item.name
        and item.is_timing_episode)
    before_tendon = episode.start_s + 0.04
    requested = generator.relative_velocity(before_tendon)
    shaped = requested + np.array([0.7, 0.0, 0.3])
    enforced = generator.enforce_basis(before_tendon, shaped)
    assert enforced[2] == 0.0
    assert enforced[0] != 0.0
    after_tendon = episode.start_s + 0.12
    requested = generator.relative_velocity(after_tendon)
    enforced = generator.enforce_basis(after_tendon, requested)
    assert enforced[2] != 0.0
    assert enforced[0] - enforced[2] != 0.0


def test_phase_3_uses_one_constant_pulse_per_active_raw_shaft():
    generator = make_generator(schedule="timing")
    for episode in generator.episodes:
        if not episode.is_timing_episode:
            continue
        for axis in range(2):
            start = episode.timing_raw_start[axis]
            end = episode.timing_raw_end[axis]
            delta = end - start
            delay = episode.timing_raw_delays_s[axis]
            if abs(delta) <= 1e-12:
                assert episode.timing_raw_velocity(0.5)[axis] == 0.0
                continue
            speed = episode.timing_raw_speeds[axis]
            duration = abs(delta) / speed
            assert episode.timing_raw_velocity(max(0.0, delay - 0.005))[axis] == (
                0.0 if delay > 0.005 else pytest.approx(
                    np.sign(delta) * speed))
            assert episode.timing_raw_velocity(delay + 0.005)[axis] == pytest.approx(
                np.sign(delta) * speed)
            assert episode.timing_raw_velocity(
                delay + 0.5 * duration)[axis] == pytest.approx(
                    np.sign(delta) * speed)
            assert episode.timing_raw_velocity(
                delay + duration + 0.005)[axis] == 0.0
            assert episode._timing_axis_state(axis, delay + duration)[0] == (
                pytest.approx(end))


def test_phase_3_raw_pulses_convert_to_reliable_logical_commands():
    generator = make_generator(schedule="timing")
    for episode in generator.episodes:
        if not episode.is_timing_episode:
            continue
        checkpoints = {0.0}
        for axis in range(2):
            delta = episode.timing_raw_end[axis] - episode.timing_raw_start[axis]
            if abs(delta) <= 1e-12:
                continue
            delay = episode.timing_raw_delays_s[axis]
            duration = abs(delta) / episode.timing_raw_speeds[axis]
            checkpoints.update((delay + 0.005, delay + 0.5 * duration))
        for local_t in checkpoints:
            raw = episode.timing_raw_velocity(local_t)
            logical = episode.state(local_t)[1]
            assert np.allclose([logical[0] - logical[2], logical[2]], raw)
            for local_axis, joint in enumerate((0, 2)):
                if raw[local_axis] != 0.0:
                    assert abs(raw[local_axis]) >= VMIN[joint]
                    assert abs(raw[local_axis]) <= episode.timing_raw_speeds[
                        local_axis]


def test_phase_3_negative_direction_is_explicit_and_safe():
    generator = make_generator(schedule="timing", timing_direction=-1)
    measured = [episode for episode in generator.episodes
                if episode.is_timing_episode]
    assert measured
    assert all(episode.name.endswith("_neg") for episode in measured)
    times = np.arange(0.0, generator.duration + 0.02, 0.02)
    position = generator.start_position + np.asarray([
        generator.relative_position(t) for t in times])
    assert np.all(position >= LOWER - 1e-9)
    assert np.all(position <= UPPER + 1e-9)


def test_phase_3_leads_must_align_to_command_period():
    with pytest.raises(ValueError, match="align with the command period"):
        make_generator(schedule="timing", timing_leads_ms=(25.0,))


def test_sim_runtime_identity_uses_node_name_not_topic_namespace():
    launch = (Path(__file__).resolve().parents[1]
              / "launch" / "causal_experiment.launch.py")
    source = launch.read_text(encoding="utf-8")
    assert '"/sim_catheter_mppi" if use_sim else "/catheter_mppi"' in source


def test_phase_2e_launch_exposes_the_reviewed_design_parameters():
    launch = (Path(__file__).resolve().parents[1]
              / "launch" / "causal_experiment.launch.py")
    source = launch.read_text(encoding="utf-8")
    for parameter in (
            "insertion_plateaus", "insertion_plateau_visits",
            "insertion_plateau_dwell_s", "tendon_sweep_limits"):
        assert f'DeclareLaunchArgument(\n            "{parameter}"' in source
        assert f'"causal_{parameter}"' in source


def test_launch_can_make_online_tracking_diagnostic_without_dropping_interlocks():
    launch = (Path(__file__).resolve().parents[1]
              / "launch" / "causal_experiment.launch.py")
    source = launch.read_text(encoding="utf-8")
    assert '"require_estimator_tracking"' in source
    assert '"causal_require_estimator_tracking"' in source
    assert '"causal_require_controller_disarmed": True' in source
    assert '"causal_require_adaptation_disabled": True' in source
