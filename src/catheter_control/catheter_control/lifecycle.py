"""Compatibility import for :mod:`catheter_control.safety.lifecycle`."""
from ._compat import reexport
_implementation = reexport(".safety.lifecycle", globals())
