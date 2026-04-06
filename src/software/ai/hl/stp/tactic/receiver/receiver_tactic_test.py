import pytest

from proto.import_all_protos import *
import software.python_bindings as tbots_cpp
from software.simulated_tests.ball_enters_region import BallEventuallyEntersRegion
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)
from proto.message_translation.tbots_protobuf import create_world_state

corner = tbots_cpp.Field.createSSLDivisionBField().enemyDefenseArea().negXNegYCorner()
centre = tbots_cpp.Field.createSSLDivisionBField().enemyDefenseArea().centre()


@pytest.mark.parametrize(
    "blue_bots, yellow_bots, ball_initial_pos, ball_initial_velocity",
    [
        # Test bottom left medium distance from attacker
        (
            [
                corner + tbots_cpp.Vector(-1.5, -1.5),
                corner + tbots_cpp.Vector(-0.5, -0.5),
            ],
            [],
            corner + tbots_cpp.Vector(-1.3, -1.3),
            tbots_cpp.Vector(0, 0),
        ),
        # Test bottom left close distance from attacker
        (
            [
                corner + tbots_cpp.Vector(-1.5, -1.5),
                corner + tbots_cpp.Vector(-1.0, -1.0),
            ],
            [],
            corner + tbots_cpp.Vector(-1.3, -1.3),
            tbots_cpp.Vector(0, 0),
        ),
        # Test centre medium distance from attacker
        (
            [
                centre + tbots_cpp.Vector(-2.5, 0.0),
                centre + tbots_cpp.Vector(-1.0, 0.0),
            ],
            [],
            centre + tbots_cpp.Vector(-2.2, 0.0),
            tbots_cpp.Vector(0, 0),
        ),
        # Test centre close distance from attacker
        (
            [
                centre + tbots_cpp.Vector(-1.5, 0.0),
                centre + tbots_cpp.Vector(-1.0, 0.0),
            ],
            [],
            centre + tbots_cpp.Vector(-1.3, 0.0),
            tbots_cpp.Vector(0, 0),
        ),
    ],
)
def test_receiver_move_away_from_shot_in_progress(
    blue_bots,
    yellow_bots,
    ball_initial_pos,
    ball_initial_velocity,
    simulated_test_runner,
):
    # Setup Robot
    def setup(*args):
        simulated_test_runner.simulator_proto_unix_io.send_proto(
            WorldState,
            create_world_state(
                blue_robot_locations=blue_bots,
                yellow_robot_locations=yellow_bots,
                ball_location=ball_initial_pos,
                ball_velocity=ball_initial_velocity,
            ),
        )

        # Setup Tactic
        params = AssignedTacticPlayControlParams()

        passer_point = blue_bots[0]
        receiver_point = blue_bots[1]
        receive_speed_m_per_s = 2.0
        min_pass_speed_m_per_s = 1.0
        max_pass_speed_m_per_s = 4.0
        pass_to_test = tbots_cpp.Pass.fromDestReceiveSpeed(
            passer_point,
            receiver_point,
            receive_speed_m_per_s,
            min_pass_speed_m_per_s,
            max_pass_speed_m_per_s,
        )

        possible_pass = Pass(
            passer_point=Point(
                x_meters=pass_to_test.passerPoint().x(),
                y_meters=pass_to_test.passerPoint().y(),
            ),
            receiver_point=Point(
                x_meters=pass_to_test.receiverPoint().x(),
                y_meters=pass_to_test.receiverPoint().y(),
            ),
            pass_speed_m_per_s=pass_to_test.speed(),
        )

        params.assigned_tactics[0].attacker.CopyFrom(
            AttackerTactic(
                best_pass_so_far=possible_pass,  # optional
                pass_committed=False,
                # chip_target={"x_meters": 0.0, "y_meters": 0.0},  # optional
            )
        )

        params.assigned_tactics[1].receiver.CopyFrom(
            ReceiverTactic(
                pass_=possible_pass,  # optional
                disable_one_touch_shot=True,
            )
        )

        simulated_test_runner.blue_full_system_proto_unix_io.send_proto(
            AssignedTacticPlayControlParams, params
        )

    # Always Validation
    always_validation_sequence_set = [[]]

    # Eventually Validation
    eventually_validation_sequence_set = [
        [
            BallEventuallyEntersRegion(
                regions=[tbots_cpp.Field.createSSLDivisionBField().enemyGoal()]
            )
        ]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=4,
    )


if __name__ == "__main__":
    pytest_main(__file__)
