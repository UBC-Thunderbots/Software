import software.python_bindings as tbots_cpp
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class BallIsOffGround(Validation):
    """Checks if a ball is of ground (i.e. it has air time)"""

    def __init__(self, threshold=0.01):
        self.threshold = threshold

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

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """Returns the underlying geometry this validation is checking

        :param world: The world msg to create validation geometry from
        :return: ValidationGeometry containing geometry to visualize

        """
        velocity = tbots_cpp.createVector(world.ball.current_state.global_velocity)
        ball_position = tbots_cpp.createPoint(world.ball.current_state.global_position)

        if velocity.length() < 0.01:
            return self.create_octagon_geometry(ball_position)

        direction = velocity.normalize()

        return self.create_arrow_geometry(ball_position, direction)

    def __repr__(self):
        return "Check if the ball is chipped"

    def create_octagon_geometry(self, centre_point):
        radius = 0.2
        start = tbots_cpp.Vector(radius, 0.0)
        # offset 45/2 degrees so octagon is parallel to the x/y axis
        start = start.rotate(tbots_cpp.Angle().fromDegrees(45.0 / 2.0))

        return create_validation_geometry(
            [
                tbots_cpp.Polygon(
                    [
                        centre_point
                        + start.rotate(tbots_cpp.Angle().fromDegrees(45.0 * i))
                        for i in range(7)
                    ]
                )
            ]
        )

    def create_arrow_geometry(self, start_point, direction):
        line_length = 0.2
        line_width = 0.08
        triangle_height = 0.15
        triangle_width = 0.2

        end_point = start_point + direction * line_length
        perpendicular = direction.perpendicular()

        line_bottom_left = start_point - perpendicular * (line_width / 2)
        line_bottom_right = start_point + perpendicular * (line_width / 2)
        line_top_right = end_point + perpendicular * (line_width / 2)
        line_top_left = end_point - perpendicular * (line_width / 2)

        triangle_top = end_point + direction * triangle_height
        triangle_bottom_left = end_point + perpendicular * (triangle_width / 2)
        triangle_bottom_right = end_point - perpendicular * (triangle_width / 2)

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
