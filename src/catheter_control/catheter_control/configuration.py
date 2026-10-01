"""Compatibility import for controller configuration composition."""
from ._compat import reexport
_implementation = reexport(".orchestration.configuration", globals())
