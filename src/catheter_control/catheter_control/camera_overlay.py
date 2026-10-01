"""Compatibility entry point for camera overlays."""
from ._compat import reexport
_implementation = reexport(".applications.camera_overlay", globals())

if __name__ == "__main__":
    _implementation.main()
