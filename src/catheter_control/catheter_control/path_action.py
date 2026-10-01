"""Compatibility entry point for path actions."""
from ._compat import reexport
_implementation = reexport(".applications.path_action", globals())

if __name__ == "__main__":
    _implementation.main()
