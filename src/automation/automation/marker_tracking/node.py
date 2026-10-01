"""Compatibility entry point for perception marker tracking."""
from .._compat import reexport
_implementation = reexport("..perception.marker_tracking", globals())

if __name__ == "__main__":
    _implementation.main()
