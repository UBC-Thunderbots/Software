import software.python_bindings as tbots_cpp
from proto.import_all_protos import *

from software.simulated_tests.validation.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)
from typing import override


class BallSpeedThresholdTrue(Validation):
    """Checks if the ball's true (simulator) speed is at or above some threshold."""

    def __init__(self, speed_threshold):
        """Constructor

        :param speed_threshold: The speed threshold in m/s
        """
        self.speed_threshold = speed_threshold

    @override
    def get_validation_status(self, world, simulator_state=None) -> ValidationStatus:
        """Checks if the ball's true speed is at or above the threshold.

        Falls back to sensor-fused velocity when simulator_state is unavailable.

        :param world: The world msg (used as fallback)
        :param simulator_state: The simulator state with true ball velocity
        :return: FAILING if below threshold, PASSING if at or above
        """
        if simulator_state is not None and simulator_state.HasField("ball"):
            speed = tbots_cpp.Vector(
                simulator_state.ball.v_x, simulator_state.ball.v_y
            ).length()
        else:
            speed = tbots_cpp.createVector(
                world.ball.current_state.global_velocity
            ).length()

        if speed >= self.speed_threshold:
            return ValidationStatus.PASSING

        return ValidationStatus.FAILING

    @override
    def get_validation_geometry(self, world) -> ValidationGeometry:
        return create_validation_geometry([])

    @override
    def __repr__(self):
        return (
            "Check that the true ball speed is at or above "
            + str(self.speed_threshold)
        )


(
    BallSpeedEventuallyAtOrAboveThresholdTrue,
    BallSpeedEventuallyBelowThresholdTrue,
    BallSpeedAlwaysAtOrAboveThresholdTrue,
    BallSpeedAlwaysBelowThresholdTrue,
) = create_validation_types(BallSpeedThresholdTrue)
