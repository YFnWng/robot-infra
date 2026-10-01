"""Record the catheter RViz window to a presentation-ready MP4 on X11."""
from __future__ import annotations

import argparse
from datetime import datetime
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys


DEFAULT_ROOT = Path(
    "/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions")


def _window_id(tree: str, title_fragment: str) -> int:
    fragment = title_fragment.casefold()
    matches = []
    for line in tree.splitlines():
        match = re.match(r'\s*(0x[0-9a-fA-F]+)\s+"([^"]*)"', line)
        if match and fragment in match.group(2).casefold():
            matches.append((int(match.group(1), 16), match.group(2)))
    exact = [item for item in matches if item[1].casefold() == fragment]
    if len(exact) == 1:
        return exact[0][0]
    decorated = [item for item in matches
                 if item[1].casefold().endswith(" - "+fragment)]
    if len(decorated) == 1:
        return decorated[0][0]
    if len(matches) != 1:
        detail = ", ".join(title for _, title in matches) or "none"
        raise RuntimeError(
            f"RViz window match {title_fragment!r} found {len(matches)} "
            f"windows: {detail}")
    return matches[0][0]


def _arguments(argv):
    parser = argparse.ArgumentParser(
        description="Record only the active catheter RViz window to MP4")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--label", default="mppi_demo")
    parser.add_argument("--window-title", default="RViz")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--duration-s", type=float)
    parser.add_argument("--crf", type=int, default=18)
    parsed = parser.parse_args(argv[1:])
    if parsed.fps < 1 or parsed.fps > 120:
        parser.error("--fps must be in [1,120]")
    if parsed.duration_s is not None and parsed.duration_s <= 0.0:
        parser.error("--duration-s must be positive")
    if parsed.crf < 0 or parsed.crf > 51:
        parser.error("--crf must be in [0,51]")
    return parsed


def main(args=None):
    argv = sys.argv if args is None else args
    parsed = _arguments(argv)
    ffmpeg = shutil.which("ffmpeg")
    xwininfo = shutil.which("xwininfo")
    if ffmpeg is None or xwininfo is None:
        raise RuntimeError("recording requires ffmpeg and xwininfo")
    tree = subprocess.run(
        [xwininfo, "-root", "-tree"], check=True, text=True,
        stdout=subprocess.PIPE).stdout
    window_id = _window_id(tree, parsed.window_title)
    output = parsed.output
    if output is None:
        output = DEFAULT_ROOT / (
            datetime.now().strftime("%Y%m%d_%H%M%S_")
            + parsed.label + ".mp4")
    output = output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    command = [
        ffmpeg, "-y", "-f", "x11grab", "-framerate", str(parsed.fps),
        "-window_id", str(window_id), "-draw_mouse", "0",
        "-i", os.environ.get("DISPLAY", ":0"),
    ]
    if parsed.duration_s is not None:
        command.extend(["-t", str(parsed.duration_s)])
    command.extend([
        "-an", "-c:v", "libx264", "-preset", "veryfast",
        "-crf", str(parsed.crf), "-pix_fmt", "yuv420p",
        "-movflags", "+faststart", str(output),
    ])
    print(f"Recording RViz window 0x{window_id:x} to {output}")
    print("Press Ctrl-C to stop and finalize the MP4.")
    try:
        completed = subprocess.run(command, check=False)
    except KeyboardInterrupt:
        # ffmpeg receives the same terminal SIGINT and finalizes the MP4 before
        # subprocess.run returns control to this wrapper.
        return
    if completed.returncode not in (0, 255):
        raise RuntimeError(f"ffmpeg exited with status {completed.returncode}")


if __name__ == "__main__":
    main()
