import time
import software.python_bindings as tbots
from proto.import_all_protos import *
from software.py_constants import NANOSECONDS_PER_SECOND

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class ExcessivePossession(Validation):

    """Checks if any friendly robot is excessively dribbling the ball, i.e. for over 1m."""

    def __init__(self, max_possession_time):
        """
        Constructs a new validation with the specified max possession time
        :param max_possession_time: the max time a robot can hold the ball before validation fails
        """
        self.max_possession_time = max_possession_time
        self.possession_start_times_dict = {}


    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if any friendly robot possesses the ball for more than the specified max.

        :param world: The world msg to validate
        :returns: FAILING when the robot is possesses the ball for too long
                  PASSING when the robot possesses the ball for less than the max time
        """
        print("HERE")
        ball_position = tbots.createPoint(world.ball.current_state.global_position)
        for robot in world.friendly_team.team_robots:
            if tbots.Robot(robot).isNearDribbler(ball_position, 0.01):
                self.possession_start_times_dict[robot.id] = time.time_ns()
                print(f"START: {self.possession_start_times_dict[robot.id]}")
            else:
                if robot.id in self.possession_start_times_dict:
                    curr_time = time.time_ns()
                    print(f"CURR: {curr_time}")
                    print(f"START: {self.possession_start_times_dict[robot.id]}")
                    possession_time = abs(curr_time - self.possession_start_times_dict[robot.id])
                    print(f"POSS: {possession_time}")
                    del self.possession_start_times_dict[robot.id]
                    if (possession_time / NANOSECONDS_PER_SECOND) > self.max_possession_time:
                        return ValidationStatus.FAILING
        return ValidationStatus.PASSING

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """
        (override) highlights the dribbler area of the robots
        """
        return create_validation_geometry(
            [
                tbots.Robot(robot).dribblerArea()
                for robot in world.friendly_team.team_robots
            ]
        )

    def __repr__(self):
        return f"Check that any friendly robot does not have possession for more than {self.max_possession_time} seconds"


(
    EventuallyDoesNotExcessivelyPossess,
    EventuallyDoesExcessivelyPossess,
    NeverExcessivelyPossesses,
    AlwaysExcessivelyPossesses,
) = create_validation_types(ExcessivePossession)
