import software.python_bindings as tbots_cpp

from proto.import_all_protos import *
from proto.message_translation.tbots_protobuf import create_world_state
from software.gameplay_tests.util import pytest_main
from software.gameplay_tests.validation.friendly_receives_ball_slow import (
    FriendlyAlwaysReceivesBallSlow,
)


def test_passing(gameplay_test_runner):
    passer_robot_id = 0
    receiver_robot_id = 1

    should_receive_pass = True
    passer_point = tbots_cpp.Point(0, 0)
    receiver_point = tbots_cpp.Point(2, 0)

    def setup():
        gameplay_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[tbots_cpp.Point(1, 1), receiver_point],
                yellow_robot_locations=[],
                ball_location=passer_point,
                ball_velocity=tbots_cpp.Vector(0, 0),
            )
        )

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

        kick_vec = pass_to_test.receiverPoint() - pass_to_test.passerPoint()

        # Setup the passer's tactic
        # We use KickTactic since AttackerTactic shoots towards the goal instead if open
        # KickTactic just does the kick we want
        blue_tactics = {
            passer_robot_id: KickTactic(
                kick_origin=Point(
                    x_meters=pass_to_test.passerPoint().x(),
                    y_meters=pass_to_test.passerPoint().y(),
                ),
                kick_direction=Angle(radians=kick_vec.orientation().toRadians()),
                kick_speed_meters_per_second=pass_to_test.speed(),
            )
        }

        # if we want a friendly robot to receive the pass
        if should_receive_pass:
            # arguments for a ReceiverTactic
            receiver_args = {
                "pass": Pass(
                    passer_point=tbots_cpp.createPointProto(pass_to_test.passerPoint()),
                    receiver_point=tbots_cpp.createPointProto(
                        pass_to_test.receiverPoint()
                    ),
                    pass_speed_m_per_s=pass_to_test.speed(),
                ),
                "disable_one_touch_shot": True,
            }

            blue_tactics[receiver_robot_id] = ReceiverTactic(**receiver_args)

        gameplay_test_runner.set_tactics(blue_tactics=blue_tactics)

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

    gameplay_test_runner.run_test(
        setup=setup,
        always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=5,
    )


if __name__ == "__main__":
    pytest_main(__file__)
