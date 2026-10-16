import math

import pytest
from software.gameplay_tests.open_loop_field_test_helpers import run_tracked_segments
from software.gameplay_tests.util import pytest_main

# These tests can only be run on the field. Each one should bring the robot
# back to roughly where it started, so drift can be judged by eye.


@pytest.mark.parametrize("speed_m_per_s,side_length_m", [
    (0.5, 0.7),
    # (1.00, 1.5),
    # (1.0, 1.0),
])
def test_open_loop_square(open_loop_field_test_runner, speed_m_per_s, side_length_m):
    duration_s = side_length_m / speed_m_per_s
    segments = [
        (speed_m_per_s, 0, 0, duration_s),
        (0, speed_m_per_s, 0, duration_s),
        (-speed_m_per_s, 0, 0, duration_s),
        (0, -speed_m_per_s, 0, duration_s),
    ]

    def execute_test():
        run_tracked_segments(open_loop_field_test_runner, segments)

    open_loop_field_test_runner.run(execute_test)


# @pytest.mark.parametrize("angular_velocity_rad_per_s", [math.pi / 2, math.pi])
# def test_open_loop_rotation(open_loop_field_test_runner, angular_velocity_rad_per_s):
#     # One full turn each way
#     duration_s = 2 * math.pi / angular_velocity_rad_per_s
#     segments = [
#         (0, 0, angular_velocity_rad_per_s, duration_s),
#         (0, 0, -angular_velocity_rad_per_s, duration_s),
#     ]
#
#     def execute_test():
#         run_tracked_segments(open_loop_field_test_runner, segments)
#
#     open_loop_field_test_runner.run(execute_test)


if __name__ == "__main__":
    pytest_main(__file__)
