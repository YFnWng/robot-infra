"""Compatibility entry point for sparse-point experiments."""
from ._compat import reexport
_implementation = reexport(".applications.sparse_point_experiment", globals())

if __name__ == "__main__":
    _implementation.main()
