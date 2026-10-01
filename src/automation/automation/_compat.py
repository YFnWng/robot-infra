"""Helpers for behavior-preserving legacy automation module paths."""
from importlib import import_module


def reexport(relative_name: str, target: dict):
    implementation = import_module(relative_name, target["__package__"])
    for name, value in vars(implementation).items():
        if not name.startswith("__"):
            target[name] = value
    target["__all__"] = getattr(
        implementation, "__all__",
        tuple(name for name in vars(implementation) if not name.startswith("_")))
    return implementation
