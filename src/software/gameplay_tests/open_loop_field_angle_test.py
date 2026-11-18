import math

import pytest
from software.gameplay_tests.open_loop_field_test_helpers import (
    drive_ramp,
    log_velocity_tracking,
    start_tracked_robot,
)
from software.gameplay_tests.util import load_command_line_arguments, pytest_main

# This test can only be run on the field. The robot drives in a straight line
# at a fixed heading in its local frame, without rotating. The heading comes
# from --angle_deg: 0 degrees is forward and angles increase CCW, so 90 degrees
# is to the robot's left. The commanded speed ramps up linearly over
# ramp_duration_s, then is held for duration_s.


@pytest.mark.parametrize("speed_m_per_s,ramp_duration_s,duration_s", [
    (0.5, 0.5, 2),
])
def test_open_loop_angle(
    open_loop_field_test_runner, speed_m_per_s, ramp_duration_s, duration_s
):
    angle_deg = load_command_line_arguments().angle_deg
    assert 0 <= angle_deg <= 360, f"angle_deg must be in [0, 360], got {angle_deg}"

    angle_rad = math.radians(angle_deg)
    x = speed_m_per_s * math.cos(angle_rad)
    y = speed_m_per_s * math.sin(angle_rad)

    def execute_test():
        start_tracked_robot(open_loop_field_test_runner)
        drive_ramp(open_loop_field_test_runner, x, y, 0, ramp_duration_s)
        robot_statuses = open_loop_field_test_runner.drive(
            x_m_per_s=x, y_m_per_s=y, duration_s=duration_s
        )
        log_velocity_tracking(robot_statuses, (x, y, 0))
        open_loop_field_test_runner.stop()

    open_loop_field_test_runner.run(execute_test)


if __name__ == "__main__":
    pytest_main(__file__)
