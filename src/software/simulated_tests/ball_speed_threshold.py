import software.python_bindings as tbots
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)

class BallSpeedThreshold(Validation):

    """Checks if the ball speed is at or above some threshold."""

    def __init__(self, speed_threshold):
        """
        :param speed_threshold: The speed threshold in m/s
        """
        self.speed_threshold = speed_threshold

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if the ball speed is at or above some threshold

        :param world: The world msg to validate
        :returns: FAILING if the ball speed is below some threshold
                  PASSING if the ball speed is at or above some threshold
        """
        if (
            tbots.createVector(world.ball.current_state.global_velocity).length()
            >= self.speed_threshold
        ):
            return ValidationStatus.PASSING

        return ValidationStatus.FAILING

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """override"""
        # TODO (#2556): visualize
        print("got validation geometry")
        return create_validation_geometry(
            [
                # tbots.Segment(ball.current_state.global_velocity) 
            ]
        )

    def __repr__(self):
        return "Check that the ball speed is at or above above " + str(
            self.speed_threshold
        )


(
    BallSpeedEventuallyAtOrAboveThreshold,
    BallSpeedEventuallyBelowThreshold,
    BallSpeedAlwaysAtOrAboveThreshold,
    BallSpeedAlwaysBelowThreshold,
) = create_validation_types(BallSpeedThreshold)
