import software.python_bindings as tbots_cpp
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class BallIsOffGround(Validation):

    """Checks if a ball is of ground (i.e. it has air time)"""

    def __init__(self, ball=None, threshold=0.01):
        self.ball = ball
        self.threshold = threshold

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if the ball has is threshold meters off the ground

        :param world: The world msg to validate
        :returns: FAILING until a ball has a positive threshold distance off the ground
                  PASSING when the ball has a positive threshold distance off the ground
        """
        if world.ball.current_state.distance_from_ground > self.threshold:
            return ValidationStatus.PASSING
        else:
            return ValidationStatus.FAILING
    def get_validation_geometry(self, world) -> ValidationGeometry:
        """Returns the underlying geometry this validation is checking

        :param world: The world msg to create validation geometry from
        :returns: ValidationGeometry containing geometry to visualize

        """
        return create_validation_geometry(
            [tbots_cpp.Circle(world.ball.position, 0.05)]
        )

    def __repr__(self):
        return "Check if the ball is chipped"


(
    BallIsEventuallyOffGround,
    BallIsEventuallyOnGround,
    BallIsAlwaysOffGround,
    BallIsAlwaysOnGround,
) = create_validation_types(BallIsOffGround)
