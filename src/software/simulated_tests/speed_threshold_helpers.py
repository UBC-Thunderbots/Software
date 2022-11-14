import software.python_bindings as tbots
from proto.import_all_protos import *
from software.py_constants import *
from software.thunderscope.constants import SPEED_SEGMENT_SCALE
import math

"""Helper functions for robot_speed_threshold and ball_speed_threshold"""

VALIDATION_LINE_SCALE_FACTOR = 300


def get_current_robot_position(robot):
    robot_x = robot.current_state.global_position.x_meters * MILLIMETERS_PER_METER
    robot_y = robot.current_state.global_position.y_meters * MILLIMETERS_PER_METER

    return robot_x, robot_y


def get_current_ball_position(ball):
    ball_x = ball.current_state.global_position.x_meters * MILLIMETERS_PER_METER
    ball_y = ball.current_state.global_position.y_meters * MILLIMETERS_PER_METER

    return ball_x, ball_y


def get_current_ball_angle(ball):
    ball_angle = math.pi / 2
    if ball.current_state.global_velocity.x_component_meters != 0:
        ball_angle = math.atan2(
            ball.current_state.global_velocity.y_component_meters,
            ball.current_state.global_velocity.x_component_meters,
        )

    return ball_angle


def get_ball_speed(ball):
    ball_speed = math.sqrt(
        ball.current_state.global_velocity.x_component_meters ** 2
        + ball.current_state.global_velocity.y_component_meters ** 2
    )

    return ball_speed


def get_validation_centre_position(x_pos, y_pos, speed_threshold, angle):
    validation_centre_x = (
        x_pos
        + MILLIMETERS_PER_METER
        * speed_threshold
        * SPEED_SEGMENT_SCALE
        * math.cos(angle)
    )
    validation_centre_y = (
        y_pos
        + MILLIMETERS_PER_METER
        * speed_threshold
        * SPEED_SEGMENT_SCALE
        * math.sin(angle)
    )

    return validation_centre_x, validation_centre_y


def get_validation_line_endpoints(validation_centre_x, validation_centre_y, angle):
    validation_start_x = (
        validation_centre_x - math.sin(angle) * VALIDATION_LINE_SCALE_FACTOR
    )
    validation_end_x = (
        validation_centre_x + math.sin(angle) * VALIDATION_LINE_SCALE_FACTOR
    )

    validation_start_y = (
        validation_centre_y + math.cos(angle) * VALIDATION_LINE_SCALE_FACTOR
    )
    validation_end_y = (
        validation_centre_y - math.cos(angle) * VALIDATION_LINE_SCALE_FACTOR
    )

    if angle > math.pi or angle < 0:
        validation_start_y = (
            validation_centre_y + math.cos(angle) * VALIDATION_LINE_SCALE_FACTOR
        )
        validation_end_y = (
            validation_centre_y - math.cos(angle) * VALIDATION_LINE_SCALE_FACTOR
        )

    return validation_start_x, validation_end_x, validation_start_y, validation_end_y
