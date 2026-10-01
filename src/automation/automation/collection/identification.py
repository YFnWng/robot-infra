"""Compatibility import for identification schedules."""
from .._compat import reexport
_implementation = reexport("..experiments.identification", globals())
