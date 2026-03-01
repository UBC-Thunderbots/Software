import software.python_bindings as tbots_cpp
from proto.import_all_protos import ValidationStatus, ValidationGeometry

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)
from typing import override


class ExcessivelyDribbling(Validation):
    """Checks if any friendly robot is excessively dribbling the ball, i.e. for over 1m."""

    def __init__(self):
        self.continuous_dribbling_start_point = None
        self.dribbler_tolerance = 0.05
        self.max_dribbling_displacement = 1.00
        self.dribbling_error_margin = 0.05

    @override
    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if any friendly robot is excessively dribbling the ball past the max dribble displacement
        minus the dribbling error margin

        :param world: The world msg to validate
        :return: FAILING when the robot is excessively dribbling
                 PASSING when the robot is not excessively dribbling
        """
        ball_position = tbots_cpp.createPoint(world.ball.current_state.global_position)
        for robot in world.friendly_team.team_robots:
            if tbots_cpp.Robot(robot).isNearDribbler(
                ball_position, self.dribbler_tolerance
            ):
                if self.continuous_dribbling_start_point is None:
                    # Set the dribbling validation start point to the current ball position
                    self.continuous_dribbling_start_point = ball_position
                elif (
                    ball_position - self.continuous_dribbling_start_point
                ).length() > (
                    self.max_dribbling_displacement - self.dribbling_error_margin
                ):
                    return ValidationStatus.FAILING
                return ValidationStatus.PASSING

        # Reset the dribbling validation start point if no robots are near the ball
        self.continuous_dribbling_start_point = None
        return ValidationStatus.PASSING

    @override
    def get_validation_geometry(self, world) -> ValidationGeometry:
        """(override) Shows the max allowed dribbling circle"""
        return create_validation_geometry(
            [
                tbots_cpp.Circle(
                    self.continuous_dribbling_start_point,
                    self.max_dribbling_displacement,
                ),
                tbots_cpp.Circle(
                    self.continuous_dribbling_start_point,
                    self.max_dribbling_displacement - self.dribbling_error_margin,
                ),
            ]
            if self.continuous_dribbling_start_point is not None
            else []
        )

    @override
    def __repr__(self):
        return f"Check that the dribbling robot has not dribbled for more than {self.max_dribbling_displacement} m minus error margin ({self.dribbling_error_margin} m)"


(
    EventuallyStopExcessivelyDribbling,
    EventuallyStartsExcessivelyDribbling,
    NeverExcessivelyDribbles,
    AlwaysExcessivelyDribbles,
) = create_validation_types(ExcessivelyDribbling)
