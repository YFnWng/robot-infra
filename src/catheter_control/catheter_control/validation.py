"""Compatibility entry point for :mod:`catheter_control.safety.validation`."""
from ._compat import reexport
_implementation = reexport(".safety.validation", globals())

if __name__ == "__main__":
    _implementation.main()
