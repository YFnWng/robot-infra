"""Compatibility entry point for session supervision."""
from .._compat import reexport
_implementation = reexport("..supervision.session_check", globals())

if __name__ == "__main__":
    _implementation.main()
