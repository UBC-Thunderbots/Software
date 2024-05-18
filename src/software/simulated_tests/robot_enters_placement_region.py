import software.python_bindings as tbots_cpp
import time
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class RobotEntersPlacementRegion(Validation):

    """Checks if a Robot enters the ball placement stadium region."""

    def __init__(self, placement_point):
        self.placement_point = placement_point
        self.first_enter_time = None
        self.interference_radius = 0.5
        self.stay_in_region_threshold = (
            2  # By SSL rules, only staying in the stadium for over 2 seconds counts
        )

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if robots enter the ball placement region

        :param world: The world msg to validate
        :returns: PASSING when robots enter and stay for over two seconds
                  FAILING otherwise
        """
        segment = tbots_cpp.Segment(
            self.placement_point,
            tbots_cpp.createPoint(world.ball.current_state.global_position),
        )
        stadium = tbots_cpp.Stadium(segment, self.interference_radius)

        for robot in world.friendly_team.team_robots:
            if tbots_cpp.contains(
                stadium, tbots_cpp.createPoint(robot.current_state.global_position)
            ):
                if not self.first_enter_time:
                    self.first_enter_time = time.time()
                else:
                    if (
                        time.time() - self.first_enter_time
                        > self.stay_in_region_threshold
                    ):
                        return ValidationStatus.PASSING
                break
        else:
            self.first_enter_time = None

        return ValidationStatus.FAILING

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """
        (override) shows regions to enter
        """
        segment = tbots_cpp.Segment(
            self.placement_point,
            tbots_cpp.createPoint(world.ball.current_state.global_position),
        )
        stadium = tbots_cpp.Stadium(segment, 0.5)
        return create_validation_geometry([stadium])

    def __repr__(self):
        return "Check for robot in ball placement stadium region"


(
    RobotEventuallyEntersPlacementRegion,
    RobotEventuallyExitsPlacementRegion,
    RobotAlwaysStaysInPlacementRegion,
    RobotNeverEntersPlacementRegion,
) = create_validation_types(RobotEntersPlacementRegion)
