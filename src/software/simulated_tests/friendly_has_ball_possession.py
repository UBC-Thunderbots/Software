import software.python_bindings as tbots_cpp
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)

class FriendlyHasBallPossession(Validation):
    """Checks if a single friendly robot has possession of the ball."""  
    def __init__(self, robot_id: int, tolerance: float = 0.01) -> None:
        """
        Initializes the validation to check for if the robot with the given id has possession

        :param robot_id: the robot which should have possession
        :param tolerance: the tolerance for possessing the ball
        """
        self.robot_id = robot_id
        self.tolerance = tolerance

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if the specified friendly robot has possession of the ball

        :param world: The world msg to validate
        :returns: FAILING when the specified friendly robot doesn't have possession of the ball
                  PASSING when the specified friendly robot has possession of the ball
        """
        ball_position = tbots_cpp.createPoint(world.ball.current_state.global_position)
        robot = world.friendly_team.team_robots[self.robot_id]

        if tbots_cpp.Robot(robot).isNearDribbler(ball_position, self.tolerance):
            return ValidationStatus.PASSING
        return ValidationStatus.FAILING

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """
        (override) highlights the dribbler area of the specified robot
        """
        return create_validation_geometry(
            [
                tbots_cpp.Robot(world.friendly_team.team_robots[self.robot_id]).dribblerArea()
            ]
        )

    def __repr__(self):
        return f"Check that friendly robot {self.robot_id} has possession of the ball"

(
    FriendlyEventuallyHasBallPossession,
    FriendlyEventuallyLosesBallPossession,
    FriendlyAlwaysHasBallPossession,
    FriendlyNeverHasBallPossession,
) = create_validation_types(FriendlyHasBallPossession)



class AnyFriendlyHasBallPossession(FriendlyHasBallPossession):

    """Checks if any friendly robot has possession of the ball."""

    def __init__(self) -> None:
        """
        Initializes the validation with no specific robot id (will be set later)
        """
        self.robot_id = None

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if any friendly robot has possession of the ball

        :param world: The world msg to validate
        :returns: FAILING when no friendly robot has possession of the ball
                  PASSING when any friendly robot has possession of the ball
        """
        for robot, index in world.friendly_team.team_robots:
            self.robot_id = index
            if super().get_validation_status(world) == ValidationStatus.PASSING:
                return ValidationStatus.PASSING
        return ValidationStatus.FAILING

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """
        (override) highlights the dribbler area of the robots
        """
        return create_validation_geometry(
            [
                tbots_cpp.Robot(robot).dribblerArea()
                for robot in world.friendly_team.team_robots
            ]
        )

    def __repr__(self):
        return "Check that the friendly team has possession of the ball"


(
    AnyFriendlyEventuallyHasBallPossession,
    AnyFriendlyEventuallyLosesBallPossession,
    AnyFriendlyAlwaysHasBallPossession,
    AnyFriendlyNeverHasBallPossession,
) = create_validation_types(AnyFriendlyHasBallPossession)
