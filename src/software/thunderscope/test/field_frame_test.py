"""Tests for the half-turn that maps the vision frame onto the friendly frame."""

import math

from software.gameplay_tests.util import pytest_main
from software.thunderscope.field_frame import rotate_pose_180


def test_rotate_pose_180_negates_position_and_shifts_heading():
    """A pose on the positive half lands on the opposite side, facing backward."""
    x, y, angle = rotate_pose_180(1.5, -0.25, 0.3)

    assert x == -1.5
    assert y == 0.25
    assert math.isclose(angle, 0.3 + math.pi)


if __name__ == "__main__":
    pytest_main(__file__)
