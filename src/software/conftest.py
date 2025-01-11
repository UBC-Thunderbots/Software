import pytest  # noqa: F401
from software.simulated_tests.simulated_test_fixture import simulated_test_runner  # noqa: F401
from software.field_tests.field_test_fixture import field_test_runner  # noqa: F401

# Pytest requires that all tests fixtures shared across a package be defined
# in a single conftest.py file in the parent directory of the package.
