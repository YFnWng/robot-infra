"""Compatibility entry point for RViz recording."""
from ._compat import reexport
_implementation = reexport(".applications.rviz_record", globals())

if __name__ == "__main__":
    _implementation.main()
