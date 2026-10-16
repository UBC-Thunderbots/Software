import math

import pytest
from software.gameplay_tests.open_loop_field_test_helpers import run_tracked_segments
from software.gameplay_tests.util import load_command_line_arguments, pytest_main

# This test can only be run on the field. The robot drives in a straight line
# at a fixed heading in its local frame, without rotating. The heading comes
# from --angle_deg: 0 degrees is forward and angles increase CCW, so 90 degrees
# is to the robot's left.


@pytest.mark.parametrize("speed_m_per_s,duration_s", [
    (0.5, 1.5),
])
def test_open_loop_angle(open_loop_field_test_runner, speed_m_per_s, duration_s):
    angle_deg = load_command_line_arguments().angle_deg
    assert 0 <= angle_deg <= 360, f"angle_deg must be in [0, 360], got {angle_deg}"

    angle_rad = math.radians(angle_deg)
    segments = [
        (
            speed_m_per_s * math.cos(angle_rad),
            speed_m_per_s * math.sin(angle_rad),
            0,
            duration_s,
        ),
    ]

    def execute_test():
        run_tracked_segments(open_loop_field_test_runner, segments)

    open_loop_field_test_runner.run(execute_test)


if __name__ == "__main__":
    pytest_main(__file__)
