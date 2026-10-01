"""Compatibility import for :mod:`catheter_control.planning.trajectory`."""
from ._compat import reexport
_implementation = reexport(".planning.trajectory", globals())
