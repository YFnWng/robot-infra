"""Compatibility entry point for simulation perception."""
from ._compat import reexport
_implementation = reexport(".simulation.sim_perception", globals())

if __name__ == "__main__":
    _implementation.main()
