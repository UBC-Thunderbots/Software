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


def test_rotate_pose_180_defaults_heading_to_a_half_turn():
    """Omitting the heading treats the pose as facing 0, then adds half a turn."""
    x, y, angle = rotate_pose_180(2.0, 3.0)

    assert (x, y) == (-2.0, -3.0)
    assert math.isclose(angle, math.pi)


def test_rotate_pose_180_is_its_own_inverse():
    """Applying the half-turn twice restores the pose.

    The heading comes back a full turn away, which is the same direction.
    """
    original = (1.5, -0.25, 0.3)
    once = rotate_pose_180(*original)
    x, y, angle = rotate_pose_180(*once)

    assert math.isclose(x, original[0])
    assert math.isclose(y, original[1])
    assert math.isclose(math.remainder(angle - original[2], 2 * math.pi), 0.0)


if __name__ == "__main__":
    pytest_main(__file__)
