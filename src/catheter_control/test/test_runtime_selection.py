from pathlib import Path

import pytest

from catheter_control.orchestration.runtime import resolve_model_selection


def test_manifest_is_the_unambiguous_selection(tmp_path):
    manifest = tmp_path / "model.json"
    result = resolve_model_selection(str(manifest))
    assert result.manifest_path == str(manifest.resolve())
    assert result.compatibility_alias is None


def test_new_and_old_selection_are_rejected_together(tmp_path):
    with pytest.raises(ValueError, match="cannot be combined"):
        resolve_model_selection(
            str(tmp_path / "model.json"),
            legacy={"cr_meta_lnn_root": str(tmp_path)})


def test_legacy_root_maps_only_to_selected_manifest(tmp_path):
    result = resolve_model_selection(
        "", legacy={"cr_meta_lnn_root": str(tmp_path)})
    assert result.manifest_path == str(
        (tmp_path / "artifacts/manifests/"
         "20260929_175554_grouped_no_rotation_v2.json").resolve())
    assert result.compatibility_alias == "cr_meta_lnn_root"


def test_individual_legacy_artifacts_fail_closed():
    with pytest.raises(ValueError, match="selection is retired"):
        resolve_model_selection(
            "", legacy={"v171_distal_checkpoint": "/tmp/model.pt"})


def test_missing_selection_fails_closed():
    with pytest.raises(ValueError, match="model_manifest is required"):
        resolve_model_selection("")


def test_runtime_pair_reuses_same_device_and_checks_split_identity(monkeypatch):
    from types import SimpleNamespace
    from catheter_control.orchestration import runtime

    identity = SimpleNamespace(manifest_sha256="abc", artifacts=("hash",), dtype="float32", runtime_family="v171")
    calls = []
    def load(manifest, **kwargs):
        calls.append(kwargs["device"])
        return SimpleNamespace(identity=identity, runtime=object())
    monkeypatch.setattr(runtime, "load_runtime", load)
    a, b = runtime.load_runtime_pair("manifest", estimator_device="cpu", planner_device="cpu")
    assert a is b and calls == ["cpu"]
    a, b = runtime.load_runtime_pair("manifest", estimator_device="cpu", planner_device="cuda:0")
    assert a is not b
    def mismatch(manifest, **kwargs):
        other = SimpleNamespace(manifest_sha256=kwargs["device"], artifacts=("hash",), dtype="float32", runtime_family="v171")
        return SimpleNamespace(identity=other)
    monkeypatch.setattr(runtime, "load_runtime", mismatch)
    with pytest.raises(ValueError, match="identity mismatch"):
        runtime.load_runtime_pair("manifest", estimator_device="cpu", planner_device="cuda:0")


def test_split_runtime_rejects_old_snapshot_api(monkeypatch):
    from types import SimpleNamespace
    from catheter_control.orchestration import runtime
    import cr_meta_lnn.deployment as deployment

    monkeypatch.setattr(runtime, "load_runtime", lambda *args, **kwargs: SimpleNamespace())
    monkeypatch.setattr(deployment, "RuntimeState", object)
    with pytest.raises(ValueError, match="updated model package"):
        runtime.load_runtime_pair("manifest", estimator_device="cpu", planner_device="cuda:0")


def test_mixed_precision_pair_on_same_device_is_not_reused(monkeypatch):
    from types import SimpleNamespace
    from catheter_control.orchestration import runtime
    calls = []
    def load(manifest, **kwargs):
        calls.append(kwargs)
        return SimpleNamespace(identity=SimpleNamespace(
            manifest_sha256="abc", artifacts=("hash",), dtype=kwargs["dtype"], runtime_family="v171"))
    monkeypatch.setattr(runtime, "load_runtime", load)
    a, b = runtime.load_runtime_pair("manifest", estimator_device="cpu", planner_device="cpu",
                                    estimator_dtype="float64")
    assert a is not b
    assert [call["dtype"] for call in calls] == ["float64", "float32"]
    with pytest.raises(ValueError, match="dtype must"):
        runtime.load_runtime_pair("manifest", estimator_device="cpu", planner_device="cpu",
                                 estimator_dtype="float16")


def test_mixed_precision_rejects_old_dtype_api(monkeypatch):
    from types import SimpleNamespace
    from catheter_control.orchestration import runtime
    import cr_meta_lnn.deployment as deployment
    class OldState:
        def clone_to(self, device):
            pass
    monkeypatch.setattr(runtime, "load_runtime", lambda *args, **kwargs: SimpleNamespace())
    monkeypatch.setattr(deployment, "RuntimeState", OldState)
    with pytest.raises(ValueError, match="dtype-aware"):
        runtime.load_runtime_pair("manifest", estimator_device="cpu", planner_device="cuda:0",
                                 estimator_dtype="float64")
