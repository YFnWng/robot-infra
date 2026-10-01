"""Compatibility entry point for simulation targets."""
from ._compat import reexport
_implementation = reexport(".simulation.sim_target", globals())

if __name__ == "__main__":
    _implementation.main()
