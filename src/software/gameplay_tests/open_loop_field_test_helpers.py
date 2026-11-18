from software.gameplay_tests.open_loop_field_test_runner import (
    MOTOR_CONTROL_RESEND_PERIOD_S,
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


def drive_ramp(open_loop_field_test_runner, x, y, w, ramp_duration_s):
    """Linearly ramp the commanded velocity from zero up to (x, y, w) over
    ramp_duration_s, leaving the final velocity commanded.

    :param open_loop_field_test_runner: the open loop field test runner
    :param x: final forward velocity in m/s
    :param y: final leftward velocity in m/s
    :param w: final angular velocity in rad/s
    :param ramp_duration_s: how long the ramp takes
    """
    logger.info(f"Ramping to x={x} y={y} w={w} over {ramp_duration_s}s")
    num_steps = max(1, round(ramp_duration_s / MOTOR_CONTROL_RESEND_PERIOD_S))
    for step in range(1, num_steps + 1):
        fraction = step / num_steps
        open_loop_field_test_runner.drive(
            x_m_per_s=x * fraction,
            y_m_per_s=y * fraction,
            angular_velocity_rad_per_s=w * fraction,
            duration_s=ramp_duration_s / num_steps,
        )


def start_tracked_robot(open_loop_field_test_runner):
    """Control the lowest-id connected robot, wait for the estop to be in PLAY,
    and start sending the expected path and the vision position to PlotJuggler.

    :param open_loop_field_test_runner: the open loop field test runner
    """
    robot_id = min(open_loop_field_test_runner.connected_robot_ids)
    logger.info(f"Running test on robot {robot_id}")
    open_loop_field_test_runner.set_controlled_robots([robot_id])
    open_loop_field_test_runner.wait_for_estop_play()
    open_loop_field_test_runner.start_path_tracking(robot_id)


def run_tracked_segments(open_loop_field_test_runner, segments):
    """Run segments on the lowest-id connected robot once the estop is in
    PLAY, sending the expected path and the vision position to PlotJuggler.

    :param open_loop_field_test_runner: the open loop field test runner
    :param segments: list of (x_m_per_s, y_m_per_s, angular_velocity_rad_per_s, duration_s)
    """
    start_tracked_robot(open_loop_field_test_runner)
    run_segments(open_loop_field_test_runner, segments)
