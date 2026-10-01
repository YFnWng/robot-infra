"""Compatibility entry point for the perception EM bridge."""
from .._compat import reexport
_implementation = reexport("..perception.em_bridge", globals())

if __name__ == "__main__":
    _implementation.main()
