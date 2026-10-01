"""Compatibility import for the estimation protocol."""
from .._compat import reexport
_implementation = reexport("..perception.estimation_protocol", globals())
