"""Learned-runtime loading for the ROS composition root."""
from pathlib import Path
import sys


def load_runtime(cr_meta_root: str, cr_common_root: str):
    meta = Path(cr_meta_root).expanduser().resolve()
    common = Path(cr_common_root).expanduser().resolve()
    if not (meta / "deployment" / "v171_streaming_runtime.py").is_file():
        raise ValueError(f"invalid cr_meta_lnn_root: {meta}")
    if not (common / "cr_common" / "__init__.py").is_file():
        raise ValueError(f"invalid cr_common_root: {common}")
    for path in (meta.parent, common):
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))
    from cr_meta_lnn.deployment import V171StreamingCatheterRuntime
    return V171StreamingCatheterRuntime
