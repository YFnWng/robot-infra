"""Compatibility entry point for runtime identity supervision."""
from .._compat import reexport
_implementation = reexport("..supervision.runtime_identity", globals())

if __name__ == "__main__":
    _implementation.main()
