"""Compatibility import for causal estimator scheduling."""
from ._compat import reexport
_implementation = reexport(".orchestration.causal_schedule", globals())
