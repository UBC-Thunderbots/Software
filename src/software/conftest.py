from software.thunderscope.qt_dependency_bootstrap import preload_bundled_qt_libs

# Must run before importing any Qt bindings (see qt_dependency_bootstrap).
preload_bundled_qt_libs()

import pytest  # noqa: F401

from software.gameplay_tests.field_test_fixture import field_test_runner  # noqa: F401
from software.gameplay_tests.simulated_test_fixture import (
    simulated_test_runner,  # noqa: F401
)

# Pytest requires that all tests fixtures shared across a package be defined
# in a single conftest.py file in the parent directory of the package.
