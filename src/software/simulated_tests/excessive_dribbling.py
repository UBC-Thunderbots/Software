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
        self.dribbling_error_margin = 0.06

    @override
    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if any friendly robot is excessively dribbling the ball for over the max dribble displacement
        minus the dribbling error margin

        :param world: The world msg to validate
        :return: FAILING when the robot is excessively dribbling
                 PASSING when the robot is not excessively dribbling
        """
        ball_position = tbots_cpp.createPoint(world.ball.current_state.global_position)
        for robot in world.friendly_team.team_robots:
            if not tbots_cpp.Robot(robot).isNearDribbler(
                ball_position, self.dribbler_tolerance
            ):
                # if ball is not near dribbler then de-activate this validation
                self.continuous_dribbling_start_point = None
            elif (
                ball_position - (self.continuous_dribbling_start_point or ball_position)
            ).length() > (
                self.max_dribbling_displacement - self.dribbling_error_margin
            ):
                return ValidationStatus.FAILING
            elif self.continuous_dribbling_start_point is None:
                # if ball is near dribbler and dribbling start point hasn't been set yet, set dribbling start point
                self.continuous_dribbling_start_point = ball_position

        return ValidationStatus.PASSING

    @override
    def get_validation_geometry(self, world) -> ValidationGeometry:
        """(override) Shows the max allowed dribbling circle"""
        return create_validation_geometry(
            [
                tbots_cpp.Circle(
                    self.continuous_dribbling_start_point,
                    self.max_dribbling_displacement,
                )
            ]
            if self.continuous_dribbling_start_point is not None
            else []
        )

    @override
    def __repr__(self):
        return "Check that the dribbling robot has not dribbled for more than 1m"


(
    EventuallyStopExcessivelyDribbling,
    EventuallyStartsExcessivelyDribbling,
    NeverExcessivelyDribbles,
    AlwaysExcessivelyDribbles,
) = create_validation_types(ExcessivelyDribbling)
