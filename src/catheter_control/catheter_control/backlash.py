"""Compatibility import for :mod:`catheter_control.transmission.backlash`."""
from ._compat import reexport
_implementation = reexport(".transmission.backlash", globals())
