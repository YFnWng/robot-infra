"""Compatibility entry point for target offsets."""
from ._compat import reexport
_implementation = reexport(".applications.target_offset", globals())

if __name__ == "__main__":
    _implementation.main()
