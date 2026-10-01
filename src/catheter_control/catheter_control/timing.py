"""Compatibility import for controller timing instrumentation."""
from ._compat import reexport
_implementation = reexport(".orchestration.timing", globals())
