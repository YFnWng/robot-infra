"""Compatibility entry point for the perception state estimator."""
from .._compat import reexport
_implementation = reexport("..perception.state_estimator", globals())

if __name__ == "__main__":
    _implementation.main()
