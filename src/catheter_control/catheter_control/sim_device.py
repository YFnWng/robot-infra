"""Compatibility entry point for :mod:`catheter_control.simulation.sim_device`."""
from ._compat import reexport
_implementation = reexport(".simulation.sim_device", globals())

if __name__ == "__main__":
    _implementation.main()
