"""Phase 3 automation responsibility and compatibility contracts."""
from automation.collection import causal_experiment as legacy_causal
from automation.collection import identification as legacy_identification
from automation.marker_tracking import node as legacy_tracking
from automation.experiments import causal_experiment, identification
from automation.perception import marker_tracking


def test_legacy_automation_imports_export_canonical_implementations():
    assert (legacy_causal.CausalExperimentGenerator
            is causal_experiment.CausalExperimentGenerator)
    assert (legacy_identification.IdentificationGenerator
            is identification.IdentificationGenerator)
    assert legacy_tracking.MarkerTrackingNode is marker_tracking.MarkerTrackingNode
