"""Compatibility import for causal experiment schedules."""
from .._compat import reexport
_implementation = reexport("..experiments.causal_experiment", globals())
