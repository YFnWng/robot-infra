"""Compatibility entry point for path files."""
from ._compat import reexport
_implementation = reexport(".applications.path_file", globals())

if __name__ == "__main__":
    _implementation.main()
