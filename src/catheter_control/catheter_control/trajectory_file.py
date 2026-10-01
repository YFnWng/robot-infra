"""Compatibility entry point for trajectory files."""
from ._compat import reexport
_implementation = reexport(".applications.trajectory_file", globals())

if __name__ == "__main__":
    _implementation.main()
