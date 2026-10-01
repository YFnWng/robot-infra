"""Compatibility import for :mod:`catheter_control.planning.path_tracking`."""
from ._compat import reexport
_implementation = reexport(".planning.path_tracking", globals())
