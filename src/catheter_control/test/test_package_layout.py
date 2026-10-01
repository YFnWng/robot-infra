"""Phase 3 package-boundary and compatibility contracts."""
from pathlib import Path

from catheter_control import backlash as legacy_backlash
from catheter_control import hardware_contract as legacy_contract
from catheter_control import mppi as legacy_mppi
from catheter_control import sim_plant as legacy_sim_plant
from catheter_control.planning import mppi
from catheter_control.safety import hardware_contract
from catheter_control.simulation import sim_plant
from catheter_control.transmission import backlash


PACKAGE = Path(__file__).resolve().parents[1] / "catheter_control"


def test_legacy_imports_export_canonical_implementations():
    assert legacy_mppi.CatheterMppi is mppi.CatheterMppi
    assert legacy_backlash.BacklashStateEstimator is backlash.BacklashStateEstimator
    assert legacy_contract.HardwareContract is hardware_contract.HardwareContract
    assert legacy_sim_plant.ModelInLoopPlant is sim_plant.ModelInLoopPlant


def test_canonical_implementation_modules_do_not_import_legacy_shims():
    forbidden = (
        "from catheter_control.mppi import",
        "from catheter_control.backlash import",
        "from catheter_control.hardware_contract import",
        "from catheter_control.sim_plant import",
    )
    canonical = (
        PACKAGE / "planning",
        PACKAGE / "transmission",
        PACKAGE / "safety",
        PACKAGE / "simulation",
        PACKAGE / "applications",
        PACKAGE / "orchestration",
    )
    for directory in canonical:
        for source in directory.glob("*.py"):
            text = source.read_text(encoding="utf-8")
            assert not any(item in text for item in forbidden), source
