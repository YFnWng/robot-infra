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
