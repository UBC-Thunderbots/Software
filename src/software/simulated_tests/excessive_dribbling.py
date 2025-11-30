import software.python_bindings as tbots_cpp
from proto.import_all_protos import *
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class ExcessivelyDribbling(Validation):
    """Checks if any friendly robot is excessively dribbling the ball, i.e. for over 1m."""

    def __init__(self, buffer_size: int = 5):
        self.continous_dribbling_start_point = None

        self.world_buffer = ThreadSafeBuffer(buffer_size, World)

    def get_validation_status(self, world, max_dribble_length: int = 1) -> ValidationStatus:
        """Checks if any friendly robot is excessively dribbling the ball, i.e. for over 1m.

        :param world: The world msg to validate
        :param max_dribble_length: the maximum dribble distance allowed (competition 1m)
        :return: FAILING when the robot is excessively dribbling
                 PASSING when the robot is not excessively dribbling
        """

        if world.HasField("dribble_displacement"):
            dribble_disp = world.dribble_displacement
            dist = tbots_cpp.createSegment(dribble_disp).length()
            if dist > max_dribble_length:
                return ValidationStatus.FAILING

        return ValidationStatus.PASSING



        ball_position = tbots_cpp.createPoint(world.ball.current_state.global_position)
        ball = world.friendly_team.team_robots
        for robot in world.friendly_team.team_robots:
            if not tbots_cpp.Robot(robot).isNearDribbler(ball_position, 0.02):
                # if ball is not near dribbler then de-activate this validation
                self.continous_dribbling_start_point = None
            else:
                if self.continous_dribbling_start_point is None:
                    self.continous_dribbling_start_point = ball_position
                print(ball_position)
                if (ball_position - self.continous_dribbling_start_point).length() > 1:
                    return ValidationStatus.FAILING
        return ValidationStatus.PASSING


    def get_validation_geometry(self, world) -> ValidationGeometry:
        """(override) Shows the max allowed dribbling circle"""
        return create_validation_geometry(
            [tbots_cpp.Circle(self.continous_dribbling_start_point, 1.0)]
            if self.continous_dribbling_start_point is not None
            else []
        )

    def __repr__(self):
        return "Check that the dribbling robot has not dribbled for more than 1m"


(
    EventuallyStopExcessivelyDribbling,
    EventuallyStartsExcessivelyDribbling,
    NeverExcessivelyDribbles,
    AlwaysExcessivelyDribbles,
) = create_validation_types(ExcessivelyDribbling)
