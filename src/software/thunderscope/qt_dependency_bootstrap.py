# import ctypes
# import glob
import os
import sys

"""Make the Qt6 binaries bundled in the ``PyQt6-Qt6`` wheel discoverable.

PyQt6 and PyQt6-Qt6 are published as separate wheels, so Bazel unpacks them
into two separate repositories. The PyQt6 extension modules (``QtCore.abi3.so``
etc.) expect the Qt frameworks/libraries in a sibling ``PyQt6/Qt6`` directory
(referenced via ``@loader_path`` on macOS and rpath on Linux), but that
directory only exists inside the PyQt6-Qt6 repository.

``DYLD_FRAMEWORK_PATH`` (macOS) and ``LD_LIBRARY_PATH`` (Linux) are read by the
dynamic loader when the process starts, so setting them from Python is too
late. Instead we locate the Qt6 directory on ``sys.path`` and re-exec the
interpreter once with the variables in place.
"""

from pathlib import Path

_READY = "BAZEL_PYQT_QT6_ENV_READY"


def _find_qt6_dir():
    for entry in sys.path:
        if not entry:
            continue
        qt6 = Path(entry) / "PyQt6" / "Qt6"
        if (qt6 / "lib").is_dir():
            return qt6
    return None


def prepare():
    if os.environ.get(_READY):
        return

    qt6 = _find_qt6_dir()
    if qt6 is None:
        return

    if sys.platform == "darwin":
        var = "DYLD_FRAMEWORK_PATH"
    else:
        var = "LD_LIBRARY_PATH"
    existing = os.environ.get(var)
    os.environ[var] = os.pathsep.join(
        [str(qt6 / "lib")] + ([existing] if existing else [])
    )
    os.environ.setdefault("QT_PLUGIN_PATH", str(qt6 / "plugins"))
    os.environ[_READY] = "1"

    os.execve(sys.executable, [sys.executable, *sys.argv], os.environ)
