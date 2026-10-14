import math

import pytest
from software.gameplay_tests.simulated_test_fixture import (
    pytest_main,
)
from software.logger.logger import create_logger

logger = create_logger(__name__)

# Fraction of each segment at the start to ignore when averaging the measured
# velocity, so the acceleration ramp doesn't skew the steady-state numbers
SEGMENT_RAMP_FRACTION = 0.5


def log_velocity_tracking(robot_statuses, commanded):
    """Log the mean measured local velocity of each robot against the commanded
    velocity, over the steady-state part of a segment.

    :param robot_statuses: RobotStatuses recorded during the segment
    :param commanded: (x_m_per_s, y_m_per_s, angular_velocity_rad_per_s) commanded
    """
    statuses_by_robot = {}
    for robot_status in robot_statuses:
        statuses_by_robot.setdefault(robot_status.robot_id, []).append(robot_status)

    for robot_id, statuses in sorted(statuses_by_robot.items()):
        steady_state = statuses[int(len(statuses) * SEGMENT_RAMP_FRACTION) :]
        if len(steady_state) == 0:
            logger.warning(f"robot {robot_id}: no RobotStatus in steady state")
            continue

        motor_statuses = [status.motor_status for status in steady_state]
        measured = (
            sum(m.local_velocity.x_component_meters for m in motor_statuses)
            / len(motor_statuses),
            sum(m.local_velocity.y_component_meters for m in motor_statuses)
            / len(motor_statuses),
            sum(m.angular_velocity.radians_per_second for m in motor_statuses)
            / len(motor_statuses),
        )
        logger.info(
            f"robot {robot_id}: commanded "
            f"(x={commanded[0]:.2f}, y={commanded[1]:.2f}, w={commanded[2]:.2f}) "
            f"measured (x={measured[0]:.2f}, y={measured[1]:.2f}, w={measured[2]:.2f})"
        )


def run_segments(open_loop_field_test_runner, segments):
    """Drive the controlled robots through a list of open-loop segments,
    stopping between each one.

    :param open_loop_field_test_runner: the open loop field test runner
    :param segments: list of (x_m_per_s, y_m_per_s, angular_velocity_rad_per_s, duration_s)
    """
    for x, y, w, duration_s in segments:
        logger.info(f"Driving x={x} y={y} w={w} for {duration_s}s")
        robot_statuses = open_loop_field_test_runner.drive(
            x_m_per_s=x,
            y_m_per_s=y,
            angular_velocity_rad_per_s=w,
            duration_s=duration_s,
        )
        log_velocity_tracking(robot_statuses, (x, y, w))
        open_loop_field_test_runner.stop()


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
        robot_id = min(open_loop_field_test_runner.connected_robot_ids)
        logger.info(f"Running test on robot {robot_id}")
        open_loop_field_test_runner.set_controlled_robots([robot_id])
        open_loop_field_test_runner.wait_for_estop_play()
        run_segments(open_loop_field_test_runner, segments)

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
#         robot_id = min(open_loop_field_test_runner.connected_robot_ids)
#         logger.info(f"Running test on robot {robot_id}")
#         open_loop_field_test_runner.set_controlled_robots([robot_id])
#         open_loop_field_test_runner.wait_for_estop_play()
#         run_segments(open_loop_field_test_runner, segments)
#
#     open_loop_field_test_runner.run(execute_test)


if __name__ == "__main__":
    pytest_main(__file__)
