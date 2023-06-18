import pytest

import software.python_bindings as tbots
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
    passer_robot_id = 1
    receiver_robot_id = 6
    should_receive_pass = True

    world = field_test_runner.world_buffer.get(block=True, timeout=WORLD_BUFFER_TIMEOUT)
    tbots_world = tbots.World(world)

    # construct a pass generator with a max receive speed set
    pass_generator = tbots.EighteenZoneIdPassGenerator(
        tbots.EighteenZonePitchDivision(tbots.Field.createSSLDivisionBField()),
        PassingConfig(max_receive_speed=py_constants.MAX_PASS_RECEIVE_SPEED),
    )

    # generate the best pass on the world 100 times
    # this improves the passes generated over time
    for index in range(0, 100):
        pass_eval = pass_generator.generatePassEvaluation(tbots_world)
        best_pass_eval = pass_eval.getBestPassOnField()
        best_pass = best_pass_eval.pass_value

    # after 100 times, get the best pass we have on the field
    pass_evaluation = pass_generator.generatePassEvaluation(tbots_world)
    best_pass_eval = pass_evaluation.getBestPassInZones(
        {
            tbots.EighteenZoneId.ZONE_1,
            tbots.EighteenZoneId.ZONE_2,
            tbots.EighteenZoneId.ZONE_3,
            tbots.EighteenZoneId.ZONE_4,
            tbots.EighteenZoneId.ZONE_5,
            tbots.EighteenZoneId.ZONE_6,
            tbots.EighteenZoneId.ZONE_7,
            tbots.EighteenZoneId.ZONE_8,
            tbots.EighteenZoneId.ZONE_9,
        }
    )
    best_pass = best_pass_eval.pass_value

    kick_vec = tbots.Vector(
        best_pass.receiverPoint().x() - best_pass.passerPoint().x(),
        best_pass.receiverPoint().y() - best_pass.passerPoint().y(),
    )

    # Setup the passer's tactic
    # We use KickTactic since AttackerTactic shoots towards the goal instead if open
    # KickTactic just does the kick we want
    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[passer_robot_id].kick.CopyFrom(
        KickTactic(
            kick_origin=Point(
                x_meters=best_pass.passerPoint().x(),
                y_meters=best_pass.passerPoint().y(),
            ),
            kick_direction=Angle(radians=kick_vec.orientation().toRadians()),
            kick_speed_meters_per_second=best_pass.speed(),
        )
    )

    # if we want a friendly robot to receive the pass
    if should_receive_pass:
        # arguments for a ReceiverTactic
        receiver_args = {
            "pass": Pass(
                passer_point=Point(
                    x_meters=best_pass.passerPoint().x(),
                    y_meters=best_pass.passerPoint().y(),
                ),
                receiver_point=Point(
                    x_meters=best_pass.receiverPoint().x(),
                    y_meters=best_pass.receiverPoint().y(),
                ),
                pass_speed_m_per_s=best_pass.speed(),
            ),
            "disable_one_touch_shot": True,
        }

        params.assigned_tactics[receiver_robot_id].receiver.CopyFrom(
            ReceiverTactic(**receiver_args)
        )

    field = tbots.Field.createSSLDivisionBField()
    eighteen_zones = tbots.EighteenZonePitchDivision(field)

    # Validate that the ball is always received by the other robot
    # slower than the max receive speed
    # and also that the ball is not passed backwards over long distances
    always_validation_sequence_set = [
        [FriendlyAlwaysReceivesBallSlow(robot_id=1, max_receive_speed=2.5)],
        [
            BallAlwaysMovesInDirectionInRegions(
                initial_ball_position=tbots_world.ball().position(),
                direction=True,
                regions=[
                    eighteen_zones.getZone(tbots.EighteenZoneId.ZONE_7),
                    eighteen_zones.getZone(tbots.EighteenZoneId.ZONE_8),
                    eighteen_zones.getZone(tbots.EighteenZoneId.ZONE_9),
                ],
            )
        ],
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
