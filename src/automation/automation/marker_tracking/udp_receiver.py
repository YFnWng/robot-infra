"""Compatibility entry point for perception marker UDP input."""
from .._compat import reexport
_implementation = reexport("..perception.marker_udp_receiver", globals())

if __name__ == "__main__":
    _implementation.main()
