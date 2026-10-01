"""Compatibility entry point for the simulation visualizer."""
from ._compat import reexport
_implementation = reexport(".simulation.sim_visualizer", globals())

if __name__ == "__main__":
    _implementation.main()
