"""Compatibility import for controller compute-device selection."""
from ._compat import reexport
_implementation = reexport(".orchestration.compute_device", globals())
