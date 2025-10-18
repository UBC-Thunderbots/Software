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
        # TODO #3244: Make this a nicer visualization

        direction = tbots_cpp.createVector(
            world.ball.current_state.global_velocity
        ).normalize()
        start_point = tbots_cpp.createPoint(world.ball.current_state.global_position)

        if direction.x() == 0.0 and direction.y() == 0.0:
            # NOTE if ball is not moving, displays arrow pointing to the right
            direction = tbots_cpp.Vector(1.0, 0.0)

        perpendicular = direction.perpendicular()

        line_width = 0.08
        line_length = 0.2
        triangle_height = 0.15
        triangle_width = 0.2

        end_point = start_point + direction * line_length

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
                    [line_bottom_left, line_bottom_right, line_top_right, line_top_left]
                ),
                tbots_cpp.Polygon(
                    [triangle_top, triangle_bottom_left, triangle_bottom_right]
                ),
            ]
        )

    def __repr__(self):
        return "Check if the ball is chipped"


(
    BallIsEventuallyOffGround,
    BallIsEventuallyOnGround,
    BallIsAlwaysOffGround,
    BallIsAlwaysOnGround,
) = create_validation_types(BallIsOffGround)
