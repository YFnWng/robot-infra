"""Compatibility import for the simulation plant."""
from ._compat import reexport
_implementation = reexport(".simulation.sim_plant", globals())
