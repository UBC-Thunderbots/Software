import ctypes
import glob
import os
import sys


def preload_bundled_qt_libs() -> None:
    """Pre-load the Qt libraries bundled with PyQt6 before importing Qt bindings.

    Ubuntu ships an older system Qt6 (e.g. 6.4 on 24.04) that shadows the newer
    Qt shipped by PyQt6, which causes version errors at import time. Using
    ctypes here forces the bundled libraries to take precedence.
    """
    for path in sys.path:
        qt_lib_dir = os.path.join(path, "PyQt6", "Qt6", "lib")
        if os.path.isfile(os.path.join(qt_lib_dir, "libQt6Core.so.6")):
            for lib in sorted(glob.glob(os.path.join(qt_lib_dir, "libQt6*.so.6"))):
                try:
                    ctypes.CDLL(lib)
                except OSError:
                    break
            break
