"""Compatibility entry point for trajectory actions."""
from ._compat import reexport
_implementation = reexport(".applications.trajectory_action", globals())

if __name__ == "__main__":
    _implementation.main()
