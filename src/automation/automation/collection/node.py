"""Compatibility entry point for experiment collection."""
from .._compat import reexport
_implementation = reexport("..experiments.collection", globals())

if __name__ == "__main__":
    _implementation.main()
