import pytest

from runtime_supervision.compare_ros import measurement_work, require_measurement_work


@pytest.mark.parametrize("extra", [[], ["--estimator-device", "cuda"],
    ["--estimator-device", "cpu", "--ros-ablation", "planner-only"],
    ["--estimator-device", "cpu", "--ros-plan-fixture", "fixture.pt"]])
def test_production_worker_rejects_unsupported_compositions(extra):
    from runtime_supervision.compare_ros import main
    with pytest.raises(SystemExit) as error:
        main(["session", "--model-manifest", "model", "--limits-file", "limits",
              "--output", "out", "--planner-process", *extra])
    assert error.value.code == 2


@pytest.mark.parametrize("estimator,expected", [(None, "cuda"), ("cuda", "cuda"), ("cpu", "cpu")])
def test_device_assignment_preserves_interlocks(estimator, expected):
    from runtime_supervision.compare_ros import probe_parameters
    params = probe_parameters({"command_output_enabled": True, "estimator_dtype": "float64"},
                              "model", "limits", "cuda", estimator)
    assert params["device"] == "cuda"
    assert params["estimator_device"] == expected
    assert params["estimator_dtype"] == "float32"
    assert params["command_output_enabled"] is False
    assert params["use_sim_time"] is False


@pytest.mark.parametrize("mode,expected", [
    ("combined", (True, True, True)),
    ("no-diagnostics", (True, True, False)),
    ("estimator-only", (True, False, True)),
    ("planner-only", (False, True, True)),
])
def test_ablation_work(mode, expected):
    assert tuple(measurement_work(mode).values()) == expected


def test_unknown_ablation_rejected():
    with pytest.raises(ValueError):
        measurement_work("unknown")


def test_estimator_only_requires_estimator_but_no_plans():
    require_measurement_work("estimator-only", {"ros_estimator_callback": {"count": 2}}, [], [])
    with pytest.raises(RuntimeError, match="incomplete"):
        require_measurement_work("estimator-only", {}, [], [])


def test_planner_only_requires_actual_plan_work():
    require_measurement_work("planner-only", {"ros_probe_planner": {"count": 2}}, [{"valid": True}], [])
    with pytest.raises(RuntimeError, match="incomplete"):
        require_measurement_work("planner-only", {"ros_probe_planner": {"count": 2}}, [], [])


def test_failures_never_qualify():
    with pytest.raises(RuntimeError, match="incomplete"):
        require_measurement_work("estimator-only", {"ros_estimator_callback": {"count": 2}}, [], ["error"])


def test_invalid_plans_never_qualify_even_without_exceptions():
    with pytest.raises(RuntimeError, match="invalid=1"):
        require_measurement_work("planner-only", {"ros_probe_planner": {"count": 2}},
                                 [{"valid": False, "reason": "deadline_missed"}], [])


def test_fixture_roundtrip_and_validation(tmp_path):
    import torch
    from runtime_supervision.compare_ros import save_plan_fixture, load_plan_fixture
    path = tmp_path / "plan.pt"
    contract = {"model_sha256": "model", "planner_config": {"seed": 17}}
    save_plan_fixture(path, (torch.tensor([1., 2.]),), contract)
    inputs, digest = load_plan_fixture(path, contract, "cpu")
    assert torch.equal(inputs[0], torch.tensor([1., 2.]))
    assert len(digest) == 64
    with pytest.raises(FileExistsError):
        save_plan_fixture(path, inputs, contract)
    with pytest.raises(ValueError, match="configuration"):
        load_plan_fixture(path, {}, "cpu")
    path.write_bytes(b"corrupted")
    with pytest.raises(ValueError, match="hash mismatch"):
        load_plan_fixture(path, contract, "cpu")


def test_fresh_planners_repeat_proposals():
    import numpy as np
    from types import SimpleNamespace
    from catheter_control.planning.mppi import CatheterMppi, MppiConfig
    config = MppiConfig(samples=512, horizon_steps=4, seed=17)
    contract = SimpleNamespace(velocity_max=np.ones(3))
    a, b = CatheterMppi(None, contract, config), CatheterMppi(None, contract, config)
    assert np.array_equal(a._samples(), b._samples())


def test_coordinated_epoch(tmp_path):
    import json
    import time
    from runtime_supervision.compare_ros import coordinated_epoch
    epoch = time.monotonic()+10
    (tmp_path/"epoch.json").write_text(json.dumps({"epoch_monotonic_s": epoch}))
    assert coordinated_epoch(tmp_path, "planner") == epoch
    assert (tmp_path/"planner.ready").exists()
    with pytest.raises(FileExistsError):
        coordinated_epoch(tmp_path, "planner")
    (tmp_path/"epoch.json").write_text(json.dumps({"epoch_monotonic_s": 0}))
    with pytest.raises(RuntimeError, match="expired"):
        coordinated_epoch(tmp_path, "estimator")


def test_barrier_timeout(tmp_path):
    from runtime_supervision.compare_ros import coordinated_epoch
    with pytest.raises(TimeoutError):
        coordinated_epoch(tmp_path, "planner", timeout_s=.01)


def test_plan_coverage_requires_cadence_not_just_work():
    from runtime_supervision.compare_ros import plan_coverage
    plans = [{'plan_started_offset_s': 32+i/15, 'valid': True} for i in range(30)]
    assert plan_coverage(plans, 32, 2, 15)['passed']
    assert not plan_coverage([], 32, 2, 15)['passed']
    assert not plan_coverage(plans[:10], 32, 2, 15)['passed']
    # A clustered burst cannot qualify even if the total count is sufficient.
    burst = [{'plan_started_offset_s': 32+i*.001} for i in range(30)]
    assert not plan_coverage(burst, 32, 2, 15)['passed']
    plans[0]['valid'] = False
    coverage = plan_coverage(plans, 32, 2, 15)
    assert not coverage['passed'] and coverage['invalid_count'] == 1
