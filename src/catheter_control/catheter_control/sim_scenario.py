"""Compatibility entry point for simulation scenarios."""
from ._compat import reexport
_implementation = reexport(".simulation.sim_scenario", globals())

if __name__ == "__main__":
    _implementation.main()
