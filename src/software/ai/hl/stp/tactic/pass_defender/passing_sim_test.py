import math

import pytest

import software.python_bindings as tbots
from proto.import_all_protos import *
from software.simulated_tests.simulated_test_fixture import (
    simulated_test_runner,
    pytest_main,
)
from proto.message_translation.tbots_protobuf import create_world_state
from software.simulated_tests.friendly_receives_ball_slow import FriendlyReceivesBallSlow

@pytest.mark.parametrize(
    "ball_initial_position,ball_initial_velocity,attacker_robot_position,receiver_robot_positions",
    [
        (
            tbots.Point(-0.5, 0),
            tbots.Vector(0.0, 0.0),
            tbots.Point(-1.0, 0.0),
            [
                tbots.Point(1.0, 0.0)
            ]
        )
    ]
)
def test_passing(
    ball_initial_position,
    ball_initial_velocity,
    attacker_robot_position,
    receiver_robot_positions,
    simulated_test_runner
):
    blue_robot_locations = [attacker_robot_position].append(receiver_robot_positions)

    # Setup Robot
    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            [],
            blue_robot_locations=blue_robot_locations,
            ball_location=ball_initial_position,
            ball_velocity=ball_initial_velocity,
        ),
    )

    world = tbots.World(
        tbots.Field.createSSLDivisionBField(),
        tbots.Ball(
            ball_initial_position,
            ball_initial_velocity,
            tbots.Timestamp()
        ),
        tbots.Team(
            [tbots.Robot(
                index,
                location,
                tbots.Vector(0.0, 0.0),
                tbots.Angle.fromRadians(0),
                tbots.Angle(),
                tbots.Timestamp(),

            ) for index, location in enumerate(blue_robot_locations)]
        ),
        tbots.Team([])
    )

    pass_generator = tbots.EighteenZoneIdPassGenerator(
        tbots.EighteenZonePitchDivision(
            tbots.Field.createSSLDivisionBField()
        ),
        PassingConfig(
            min_pass_speed_m_per_s=min_pass_speed_m_per_s,
            max_pass_speed_m_per_s=max_pass_speed_m_per_s
        )
    )

    pass_evaluation = pass_generator.generatePassEvaluation(world)

    best_pass_eval = pass_evaluation.getBestPassOnField()
    best_pass = best_pass_eval.pass_value

    kick_vec = tbots.Vector(
        best_pass.receiverPoint().x() - best_pass.passerPoint().x(),
        best_pass.receiverPoint().y() - best_pass.passerPoint().y()
    )
    # Setup Tactic
    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[0].kick.CopyFrom(
        KickTactic(
            kick_origin=Point(
                x_meters=best_pass.passerPoint().x(),
                y_meters=best_pass.passerPoint().y()
            ),
            kick_direction=Angle(
                radians=kick_vec.orientation().toRadians()
            ),
            kick_speed_meters_per_second=best_pass.speed()
        )
    )
    receiver_args = {
        "pass": Pass(
            passer_point=Point(
                x_meters=best_pass.passerPoint().x(),
                y_meters=best_pass.passerPoint().y()
            ),
            receiver_point=Point(
                x_meters=best_pass.receiverPoint().x(),
                y_meters=best_pass.receiverPoint().y()
            ),
            pass_speed_m_per_s=best_pass.speed()
        ),
        "disable_one_touch_shot": True
    }
    params.assigned_tactics[1].receiver.CopyFrom(
        ReceiverTactic(
            **receiver_args
        )
    )
    simulated_test_runner.blue_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    # Setup no tactics on the enemy side
    params = AssignedTacticPlayControlParams()
    simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    # Eventually Validation
    eventually_validation_sequence_set = [[]]
    always_validation_sequence_set = [
        [
            FriendlyReceivesBallSlow(
                robot_id=1,
                max_receive_speed=3
            )
        ]
    ]

    simulated_test_runner.run_test(
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
    )


if __name__ == "__main__":
    pytest_main(__file__)