import software.python_bindings as tbots_cpp
from proto.import_all_protos import *

from software.simulated_tests.validation.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)
from typing import override


class BallStopsInRegionTrue(Validation):
    """Checks if the ball's true (simulator) position stops in any of the provided regions."""

    def __init__(self, regions=None):
        self.regions = regions if regions else []

    @override
    def get_validation_status(self, world, simulator_state=None) -> ValidationStatus:
        """Checks if the ball's true position stops in the provided regions.

        Falls back to the sensor-fused world position when simulator_state is unavailable.

        :param world: The world msg (used as fallback and for velocity)
        :param simulator_state: The simulator state with true ball position/velocity
        :return: FAILING until the ball stops in any region, PASSING when it does
        """
        if simulator_state is not None and simulator_state.HasField("ball"):
            ball_point = tbots_cpp.Point(
                simulator_state.ball.p_x, simulator_state.ball.p_y
            )
            ball_speed = tbots_cpp.Vector(
                simulator_state.ball.v_x, simulator_state.ball.v_y
            ).length()
        else:
            ball_point = tbots_cpp.createPoint(world.ball.current_state.global_position)
            ball_speed = tbots_cpp.createVector(
                world.ball.current_state.global_velocity
            ).length()

        for region in self.regions:
            if tbots_cpp.contains(region, ball_point) and ball_speed <= 0.01:
                return ValidationStatus.PASSING

        return ValidationStatus.FAILING

    @override
    def get_validation_geometry(self, world) -> ValidationGeometry:
        return create_validation_geometry(self.regions)

    @override
    def __repr__(self):
        return "Checking true ball position stops in regions " + ",".join(
            repr(region) for region in self.regions
        )


(
    BallEventuallyStopsInRegionTrue,
    BallEventuallyMovesInRegionTrue,
    BallAlwaysStopsInRegionTrue,
    BallNeverStopsInRegionTrue,
) = create_validation_types(BallStopsInRegionTrue)
