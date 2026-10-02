import json
from pathlib import Path

from runtime_supervision.runtime_identity import (
    _manifest_identity, _package_identity)


def test_package_identity_includes_deployment_distributions():
    identity = _package_identity()
    assert identity["catheter-control"] == "0.1.0"
    assert identity["cr-meta-lnn"] == "1.0.0"
    assert identity["cr-common"] == "0.1.0"


def test_manifest_identity_records_bundle_and_artifact_hashes(tmp_path):
    manifest = tmp_path / "model.json"
    manifest.write_text(json.dumps({
        "schema_version": 2,
        "bundle_name": "fixture",
        "deployment_api_version": "1.0",
        "runtime_family": "test",
        "artifacts": [{
            "id": "distal", "bytes": 4, "sha256": "a" * 64}],
    }))
    identity = _manifest_identity(manifest)
    assert identity["exists"] is True
    assert identity["schema_version"] == 2
    assert identity["bundle_name"] == "fixture"
    assert identity["artifacts"] == [{
        "id": "distal", "bytes": 4, "sha256": "a" * 64}]
