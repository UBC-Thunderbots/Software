import pytest

import software.python_bindings as tbots_cpp
import sys
from proto.import_all_protos import *
from software.field_tests.field_test_fixture import *
from software.simulated_tests.friendly_receives_ball_slow import (
    FriendlyAlwaysReceivesBallSlow,
)
from software.simulated_tests.ball_moves_in_direction import (
    BallAlwaysMovesInDirectionInRegions,
)
from software import py_constants


def test_passing(field_test_runner):
    passer_robot_id = 3
    receiver_robot_id = 5
    should_receive_pass = True

    world = field_test_runner.world_buffer.get(block=True, timeout=WORLD_BUFFER_TIMEOUT)
    passer_point = tbots_cpp.createPoint(world.ball.current_state.global_position)
    receiver_point = None
    for robot in world.friendly_team.team_robots:
        if robot.id == receiver_robot_id:
            receiver_point = tbots_cpp.createPoint(robot.current_state.global_position)

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

    kick_vec = tbots_cpp.Vector(
        pass_to_test.receiverPoint().x() - pass_to_test.passerPoint().x(),
        pass_to_test.receiverPoint().y() - pass_to_test.passerPoint().y(),
    )

    # Setup the passer's tactic
    # We use KickTactic since AttackerTactic shoots towards the goal instead if open
    # KickTactic just does the kick we want
    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[passer_robot_id].kick.CopyFrom(
        KickTactic(
            kick_origin=Point(
                x_meters=pass_to_test.passerPoint().x(),
                y_meters=pass_to_test.passerPoint().y(),
            ),
            kick_direction=Angle(radians=kick_vec.orientation().toRadians()),
            kick_speed_meters_per_second=pass_to_test.speed(),
        )
    )

    # if we want a friendly robot to receive the pass
    if should_receive_pass:
        # arguments for a ReceiverTactic
        receiver_args = {
            "pass": Pass(
                passer_point=Point(
                    x_meters=pass_to_test.passerPoint().x(),
                    y_meters=pass_to_test.passerPoint().y(),
                ),
                receiver_point=Point(
                    x_meters=pass_to_test.receiverPoint().x(),
                    y_meters=pass_to_test.receiverPoint().y(),
                ),
                pass_speed_m_per_s=pass_to_test.speed(),
            ),
            "disable_one_touch_shot": True,
        }

        params.assigned_tactics[receiver_robot_id].receiver.CopyFrom(
            ReceiverTactic(**receiver_args)
        )

    field = tbots_cpp.Field.createSSLDivisionBField()
    tbots_cpp.EighteenZonePitchDivision(field)

    # Validate that the ball is always received by the other robot
    # slower than the max receive speed
    # and also that the ball is not passed backwards over long distances
    always_validation_sequence_set = [
        [
            FriendlyAlwaysReceivesBallSlow(
                robot_id=receiver_robot_id, max_receive_speed=2.5
            )
        ]
    ]

    field_test_runner.set_tactics(params, True)
    field_test_runner.run_test(
        always_validation_sequence_set=always_validation_sequence_set,
        eventually_validation_sequence_set=[[]],
        test_timeout_s=5,
    )

    # Send a stop tactic after the test finishes
    stop_tactic = StopTactic()
    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[passer_robot_id].stop.CopyFrom(stop_tactic)
    params.assigned_tactics[receiver_robot_id].stop.CopyFrom(stop_tactic)
    # send the stop tactic
    field_test_runner.set_tactics(params, True)


if __name__ == "__main__":
    # Run the test, -s disables all capturing at -vv increases verbosity
    sys.exit(pytest.main([__file__, "-svv"]))
