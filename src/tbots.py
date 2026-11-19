#!/usr/bin/env python3
"""Launcher for the tbots CLI (//cli:tbots), run with Bazel's hermetic Python.

Only uses the standard library, so it runs with the system Python.
"""

import os
import subprocess
import sys
import threading
from pathlib import Path

# --output_base: separate Bazel server, so the CLI doesn't clear the main build's cache
BAZEL = ["bazel", "--quiet", f"--output_base={Path.home() / '.cache/bazel/tbots_cli'}"]

# --symlink_prefix=/: don't overwrite the main bazel-bin/bazel-out symlinks in src/
# --ui_event_filters, --noshow_progress: hide Bazel's output
FLAGS = [
    "--symlink_prefix=/",
    "--ui_event_filters=-info,-stdout,-stderr",
    "--noshow_progress",
]


def main() -> None:
    os.chdir(Path(__file__).resolve().parent)

    # First build can take a while, so print a message if it takes over 3s
    notice = threading.Timer(
        3,
        lambda: print(
            "tbots.py: building the tbots CLI, this can take a few minutes the first time...",
            file=sys.stderr,
        ),
    )
    notice.daemon = True
    notice.start()
    try:
        build = subprocess.run([*BAZEL, "build", *FLAGS, "//cli:tbots"], check=False)
    except KeyboardInterrupt:
        sys.exit(130)
    finally:
        notice.cancel()
    if build.returncode != 0:
        sys.exit(build.returncode)

    # Replace this process with the CLI
    os.execvp("bazel", [*BAZEL, "run", *FLAGS, "//cli:tbots", "--", *sys.argv[1:]])


if __name__ == "__main__":
    main()