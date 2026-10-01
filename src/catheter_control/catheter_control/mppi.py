"""Compatibility import for :mod:`catheter_control.planning.mppi`."""
from ._compat import reexport
_implementation = reexport(".planning.mppi", globals())
