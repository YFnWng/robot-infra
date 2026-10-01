"""Compatibility import for :mod:`catheter_control.planning.tracking`."""
from ._compat import reexport
_implementation = reexport(".planning.tracking", globals())
