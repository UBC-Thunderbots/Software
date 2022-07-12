import pytest

import software.python_bindings as tbots
from software.simulated_tests.pytest_main import pytest_main
from proto.ssl_gc_common_pb2 import Team
from proto.import_all_protos import *
from software.field_tests.field_test_fixture import *
from software.simulated_tests.simulated_test_fixture import *
from software.simulated_tests.tbots_test_fixture import tbots_test_runner
from software.simulated_tests.data_logger import DataLogger


# this test can be run either in simulation or on the field
@pytest.mark.parametrize(
    "robot_x_position",
    "robot_y_position"[(-2.0, -0.3), (-2.0, 0.3), (-1.0, 0.3), (-1.0, -0.3)],
)
def test_basic_movement(tbots_test_runner, robot_x_position, robot_y_position):

    rob_pos_p = Point(x_meters=robot_x_position, y_meters=robot_y_position)

    logger.info(angle)
    move_tactic = MoveTactic()
    move_tactic.destination.CopyFrom(rob_pos_p)
    move_tactic.final_speed = 0.0
    move_tactic.dribbler_mode = DribblerMode.OFF
    move_tactic.final_orientation.CopyFrom(Angle(radians=angle))
    move_tactic.ball_collision_type = BallCollisionType.AVOID
    move_tactic.auto_chip_or_kick.CopyFrom(AutoChipOrKick(autokick_speed_m_per_s=0.0))
    move_tactic.max_allowed_speed_mode = MaxAllowedSpeedMode.PHYSICAL_LIMIT
    move_tactic.target_spin_rev_per_s = 0.0

    # Setup Tactic
    params = AssignedTacticPlayControlParams()

    params.assigned_tactics[0].move.CopyFrom(move_tactic)

    # Eventually Validation
    eventually_validation_sequence_set = [
        [
            RobotEventuallyEntersRegion(
                regions=[
                    tbots.Circle(tbots.Point(robot_x_position, robot_y_position), 0.2)
                ]
            ),
        ]
    ]

    tbots_test_runner.set_tactics(params, True)

    tbots_test_runner.run_test(
        eventually_validation_sequence_set=eventually_validation_sequence_set,
        test_timeout_s=5,
    )


# this test can only be run on the field
def test_basic_rotation(field_test_runner):

    test_angles = [0, 45, 90, 180, 270, 0]

    # for fixed pos
    x, y = (-1, 0)
    rob_pos_p = Point(x_meters=x, y_meters=y)

    for angle in test_angles:

        move_tactic = MoveTactic()
        move_tactic.destination.CopyFrom(rob_pos_p)
        move_tactic.final_speed = 0.0
        move_tactic.dribbler_mode = DribblerMode.OFF
        move_tactic.final_orientation.CopyFrom(Angle(radians=angle))
        move_tactic.ball_collision_type = BallCollisionType.AVOID
        move_tactic.auto_chip_or_kick.CopyFrom(
            AutoChipOrKick(autokick_speed_m_per_s=0.0)
        )
        move_tactic.max_allowed_speed_mode = MaxAllowedSpeedMode.PHYSICAL_LIMIT
        move_tactic.target_spin_rev_per_s = 0.0

        # Setup Tactic
        params = AssignedTacticPlayControlParams()

        params.assigned_tactics[0].move.CopyFrom(move_tactic)

        tbots_test_runner.set_tactics(params, True)

        # validate by eye
        logger.info(f"robot set to {angle} orientation")

        time.sleep(5)


if __name__ == "__main__":
    pytest_main(__file__)
