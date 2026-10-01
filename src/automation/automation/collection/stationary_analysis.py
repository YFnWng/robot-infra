"""Compatibility entry point for stationary-session analysis."""
from .._compat import reexport
_implementation = reexport("..supervision.stationary_analysis", globals())

if __name__ == "__main__":
    _implementation.main()
