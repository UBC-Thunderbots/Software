import software.python_bindings as tbots_cpp
from proto.import_all_protos import *

from software.simulated_tests.validation.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)
from typing import override


class BallEntersRegionTrue(Validation):
    """Checks if the ball's true (simulator) position enters any of the provided regions."""

    def __init__(self, regions=None):
        self.regions = regions if regions else []
        self.ball_position = None

    @override
    def get_validation_status(self, world, simulator_state=None) -> ValidationStatus:
        """Checks if the ball's true position enters the provided regions.

        Falls back to the sensor-fused world position when simulator_state is unavailable.

        :param world: The world msg (used as fallback)
        :param simulator_state: The simulator state with true ball position
        :return: FAILING until the ball enters any region, PASSING when it does
        """
        if simulator_state is not None and simulator_state.HasField("ball"):
            self.ball_position = tbots_cpp.Point(
                simulator_state.ball.p_x, simulator_state.ball.p_y
            )
        else:
            self.ball_position = tbots_cpp.createPoint(
                world.ball.current_state.global_position
            )

        for region in self.regions:
            if tbots_cpp.contains(region, self.ball_position):
                return ValidationStatus.PASSING
        return ValidationStatus.FAILING

    @override
    def get_validation_geometry(self, world) -> ValidationGeometry:
        return create_validation_geometry(self.regions)

    @override
    def __repr__(self):
        return (
            "Checking true ball position in regions "
            + ",".join(repr(region) for region in self.regions)
            + (", ball position: " + str(self.ball_position) if self.ball_position else "")
        )


(
    BallEventuallyEntersRegionTrue,
    BallEventuallyExitsRegionTrue,
    BallAlwaysStaysInRegionTrue,
    BallNeverEntersRegionTrue,
) = create_validation_types(BallEntersRegionTrue)
