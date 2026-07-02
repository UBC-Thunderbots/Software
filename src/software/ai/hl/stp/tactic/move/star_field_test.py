import math

import pytest
import software.python_bindings as tbots_cpp

from proto.import_all_protos import *
from proto.message_translation.tbots_protobuf import create_world_state
from software.gameplay_tests.util import pytest_main
from software.gameplay_tests.validation.duration_validation import DurationValidation

from software.gameplay_tests.validation.robot_enters_region import (
    RobotEventuallyEntersRegion,
)

# The robot traces a "star" centered at the origin: for each ray it drives out
# 1m from the center, then returns to the center, before repeating for the next
# ray. Rays sweep the full circle in fixed angular steps. Throughout every leg
# the robot keeps facing the +x axis (the 0 degree direction).
CENTER = tbots_cpp.Point(0, 0)
RAY_LENGTH_M = 1.0
ANGLE_STEP_DEG = 45
MOVEMENT_ORIENTATION = tbots_cpp.Angle.fromDegrees(0)

# Tolerance for the "always facing 0 degrees" check. This has to be looser than
# the field runner's setup orientation tolerance (SETUP_ORIENTATION_TOLERANCE_DEG
# = 15) since the robot may still be up to that far off when validation starts.
ORIENTATION_THRESHOLD_RAD = math.radians(20)

# The robot has to reach the center, so keep the ball well outside the star so
# it never gets in the way (the move tactic avoids the ball).
BALL_POSITION = tbots_cpp.Point(2.5, 2.0)


def _ray_tip(angle_deg):
    """Returns the point RAY_LENGTH_M from the origin along the given angle.

    :param angle_deg: the ray's direction, in degrees
    :return: the tbots_cpp.Point at the tip of the ray
    """
    angle_rad = math.radians(angle_deg)
    return tbots_cpp.Point(
        RAY_LENGTH_M * math.cos(angle_rad),
        RAY_LENGTH_M * math.sin(angle_rad),
    )


def _star_legs():
    """Builds the ordered (start, end) legs that trace the star.

    For each ray the robot drives from the center out to the ray tip, then back
    to the center, so the legs alternate outward and return movements.

    :return: list of (start_position, end_position) tuples
    """
    legs = []
    for angle_deg in range(0, 360, ANGLE_STEP_DEG):
        tip = _ray_tip(angle_deg)
        legs.append((CENTER, tip))
        legs.append((tip, CENTER))
    return legs


@pytest.mark.parametrize("start_position, end_position", _star_legs())
def test_star_field(start_position, end_position, gameplay_test_runner):
    def setup():
        gameplay_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[start_position],
                blue_robot_orientations=[MOVEMENT_ORIENTATION],
                yellow_robot_locations=[],
                ball_location=BALL_POSITION,
                ball_velocity=tbots_cpp.Vector(0, 0),
            ),
        )

        tactic = MoveTactic(
            destination=tbots_cpp.createPointProto(end_position),
            dribbler_mode=DribblerMode.OFF,
            final_orientation=tbots_cpp.createAngleProto(MOVEMENT_ORIENTATION),
            ball_collision_type=BallCollisionType.AVOID,
            auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=0.0),
            max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
            obstacle_avoidance_mode=ObstacleAvoidanceMode.SAFE,
        )

        gameplay_test_runner.set_tactics(
            blue_tactics={
                0: tactic,
            },
        )

    gameplay_test_runner.run_test(
        setup=setup,
        test_timeout_s=4,
        eventually_validation_sequence_set=[
            [
                DurationValidation(
                    duration_s=1,
                    validation=RobotEventuallyEntersRegion(
                        regions=[tbots_cpp.Circle(end_position, 0.05)]
                    ),
                ),
            ],
        ],
    )


if __name__ == "__main__":
    pytest_main(__file__)
