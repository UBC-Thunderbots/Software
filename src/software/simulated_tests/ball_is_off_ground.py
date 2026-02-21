import software.python_bindings as tbots_cpp
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)
from typing import override


class BallIsOffGround(Validation):
    """Checks if a ball is of ground (i.e. it has air time)"""

    LINE_LENGTH = 0.2
    LINE_WIDTH = 0.08
    TRIANGLE_HEIGHT = 0.15
    TRIANGLE_WIDTH = 0.2

    def __init__(self, threshold=0.2):
        self.threshold = threshold

    @override
    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if the ball has is threshold meters off the ground

        :param world: The world msg to validate
        :return: FAILING until a ball has a positive threshold distance off the ground
                 PASSING when the ball has a positive threshold distance off the ground
        """
        if world.ball.current_state.distance_from_ground > self.threshold:
            return ValidationStatus.PASSING
        else:
            return ValidationStatus.FAILING

    @override
    def get_validation_geometry(self, world) -> ValidationGeometry:
        """Returns the underlying geometry this validation is checking

        :param world: The world msg to create validation geometry from
        :return: ValidationGeometry containing geometry to visualize
        """
        velocity = tbots_cpp.createVector(world.ball.current_state.global_velocity)
        ball_position = tbots_cpp.createPoint(world.ball.current_state.global_position)

        if velocity.length() < 0.01:
            return create_validation_geometry()

        direction = velocity.normalize()

        return self.create_arrow_geometry(ball_position, direction)

    @override
    def __repr__(self):
        return "Check if the ball is chipped"

    def create_arrow_geometry(self, start_point, direction):
        """Returns arrow validation geometry

        :param start_point: The starting position of arrow
        :param direction: The direction the arrow is pointing
        :return: An arrow ValidationGeometry

        """
        end_point = start_point + direction * self.LINE_LENGTH
        perpendicular = direction.perpendicular()

        line_bottom_left = start_point - perpendicular * (self.LINE_WIDTH / 2)
        line_bottom_right = start_point + perpendicular * (self.LINE_WIDTH / 2)
        line_top_right = end_point + perpendicular * (self.LINE_WIDTH / 2)
        line_top_left = end_point - perpendicular * (self.LINE_WIDTH / 2)

        triangle_top = end_point + direction * self.TRIANGLE_HEIGHT
        triangle_bottom_left = end_point + perpendicular * (self.TRIANGLE_WIDTH / 2)
        triangle_bottom_right = end_point - perpendicular * (self.TRIANGLE_WIDTH / 2)

        return create_validation_geometry(
            [
                tbots_cpp.Polygon(
                    [
                        line_bottom_left,
                        line_top_left,
                        triangle_bottom_left,
                        triangle_top,
                        triangle_bottom_right,
                        line_top_right,
                        line_bottom_right,
                    ]
                ),
            ]
        )


(
    BallIsEventuallyOffGround,
    BallIsEventuallyOnGround,
    BallIsAlwaysOffGround,
    BallIsAlwaysOnGround,
) = create_validation_types(BallIsOffGround)
