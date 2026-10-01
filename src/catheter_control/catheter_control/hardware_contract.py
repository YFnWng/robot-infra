"""Compatibility import for :mod:`catheter_control.safety.hardware_contract`."""
from ._compat import reexport
_implementation = reexport(".safety.hardware_contract", globals())
