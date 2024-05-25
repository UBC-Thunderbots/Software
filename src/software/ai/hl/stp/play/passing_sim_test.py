import pytest
import math
import software.python_bindings as tbots_cpp
from proto.import_all_protos import *
from software.simulated_tests.simulated_test_fixture import (
    simulated_test_runner,
    pytest_main,
)
from proto.message_translation.tbots_protobuf import create_world_state
from software.simulated_tests.friendly_receives_ball_slow import (
    FriendlyAlwaysReceivesBallSlow,
)
from software.simulated_tests.friendly_has_ball_possession import (
    FriendlyEventuallyHasBallPossession,
)
from software.simulated_tests.ball_moves_in_direction import BallMovesForwardInRegions
from software.simulated_tests.ball_enters_region import (
    BallEventuallyExitsRegion,
    BallEventuallyEntersRegion,
)
from software import py_constants


def setup_pass_and_robots(
    ball_initial_position,
    ball_initial_velocity,
    attacker_robot_position,
    receiver_robot_positions,
    friendly_orientations,
    enemy_robot_positions,
    receive_pass,
    simulated_test_runner,
):
    """
    Sets up a test involving 1 robot passing the ball
    With any number of friendly and enemy robots on the field
    Can specify if the first friendly robot should receive the pass or not
    :param ball_initial_position: the initial position of the ball
    :param ball_initial_velocity: the initial velocity of the ball
    :param attacker_robot_position: the position of the robot doing the pass
    :param receiver_robot_positions: the positions of the friendly robots
    :param friendly_orientations: the orientations of the friendly robots
    :param enemy_robot_positions: the positions of the enemy robots
    :param receive_pass: whether a friendly robot should try to receive the pass
    :param simulated_test_runner: the test runner
    :return the best pass we generate
    """
    blue_robot_locations = [attacker_robot_position, *receiver_robot_positions]

    # Setup the world state
    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            yellow_robot_locations=enemy_robot_positions,
            blue_robot_locations=blue_robot_locations,
            ball_location=ball_initial_position,
            ball_velocity=ball_initial_velocity,
            blue_robot_orientations=friendly_orientations,
        ),
    )

    # construct a world object to match the one sent to the test runner
    world = tbots_cpp.World(
        tbots_cpp.Field.createSSLDivisionBField(),
        tbots_cpp.Ball(
            ball_initial_position, ball_initial_velocity, tbots_cpp.Timestamp()
        ),
        tbots_cpp.Team(
            [
                tbots_cpp.Robot(
                    index,
                    location,
                    tbots_cpp.Vector(0.0, 0.0),
                    tbots_cpp.Angle.fromRadians(0),
                    tbots_cpp.Angle(),
                    tbots_cpp.Timestamp(),
                )
                for index, location in enumerate(blue_robot_locations)
            ]
        ),
        tbots_cpp.Team(
            [
                tbots_cpp.Robot(
                    index,
                    location,
                    tbots_cpp.Vector(0.0, 0.0),
                    tbots_cpp.Angle.fromRadians(0),
                    tbots_cpp.Angle(),
                    tbots_cpp.Timestamp(),
                )
                for index, location in enumerate(enemy_robot_positions)
            ]
        ),
    )

    # construct a pass generator with a max receive speed set
    pass_generator = tbots_cpp.EighteenZoneIdPassGenerator(
        tbots_cpp.EighteenZonePitchDivision(tbots_cpp.Field.createSSLDivisionBField()),
        PassingConfig(),
    )

    # generate the best pass on the world 100 times
    # this improves the passes generated over time
    for index in range(0, 100):
        pass_eval = pass_generator.generatePassEvaluation(world)
        best_pass_eval = pass_eval.getBestPassOnField()
        best_pass = best_pass_eval.pass_value

    # after 100 times, get the best pass we have on the field
    pass_evaluation = pass_generator.generatePassEvaluation(world)
    best_pass_eval = pass_evaluation.getBestPassOnField()
    best_pass = best_pass_eval.pass_value

    kick_vec = tbots_cpp.Vector(
        best_pass.receiverPoint().x() - best_pass.passerPoint().x(),
        best_pass.receiverPoint().y() - best_pass.passerPoint().y(),
    )

    # Setup the passer's tactic
    # We use KickTactic since AttackerTactic shoots towards the goal instead if open
    # KickTactic just does the kick we want
    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[0].kick.CopyFrom(
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
    if receive_pass:
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

        params.assigned_tactics[1].receiver.CopyFrom(ReceiverTactic(**receiver_args))
    simulated_test_runner.set_tactics(params, True)

    # Setup no tactics on the enemy side
    params = AssignedTacticPlayControlParams()
    simulated_test_runner.set_tactics(params, False)

    return best_pass


@pytest.mark.parametrize(
    "ball_initial_position,ball_initial_velocity,attacker_robot_position,"
    "receiver_robot_positions,friendly_orientations,enemy_robot_positions",
    [
        # pass between 2 robots close to each other
        (
            tbots_cpp.Point(-0.85, 0),
            tbots_cpp.Vector(0.0, 0.0),
            tbots_cpp.Point(-1.0, 0.0),
            [tbots_cpp.Point(1.0, 0.0)],
            [0, math.pi],
            [],
        ),
        # pass between 2 robots on opposite ends of the field
        (
            tbots_cpp.Point(-3.35, 0.0),
            tbots_cpp.Vector(0.0, 0.0),
            tbots_cpp.Point(-3.5, 0.0),
            [tbots_cpp.Point(3.5, 0.0)],
            [0, math.pi],
            [],
        ),
        # TODO: Make Interception Better
        # https://github.com/UBC-Thunderbots/Software/issues/2984
        # pass between 2 robots above one another (on the y-axis)
        (
            tbots_cpp.Point(0.0, 0.0),
            tbots_cpp.Vector(0.0, 0.0),
            tbots_cpp.Point(0.0, -1.0),
            [tbots_cpp.Point(0.0, 3.0)],
            [0, math.pi],
            [],
        ),
        # pass between 2 robots on opposite ends of the field's diagonal
        (
            tbots_cpp.Point(-3.35, 2.35),
            tbots_cpp.Vector(0.0, 0.0),
            tbots_cpp.Point(-3.5, 2.5),
            [tbots_cpp.Point(3.5, -2.5)],
            [0, math.pi],
            [],
        ),
        # straight pass with an enemy in between the 2 robots
        (
            tbots_cpp.Point(-0.5, 0),
            tbots_cpp.Vector(0.0, 0.0),
            tbots_cpp.Point(-1.0, 0.0),
            [tbots_cpp.Point(1.5, 0.0)],
            [0, math.pi],
            [tbots_cpp.Point(0.5, 0.0)],
        ),
        # pass with a sparse wall of enemy robots in between the 2 robots
        (
            tbots_cpp.Point(-1.7, 0),
            tbots_cpp.Vector(0.0, 0.0),
            tbots_cpp.Point(-2.0, 0.0),
            [tbots_cpp.Point(2.5, 0.0)],
            [0, math.pi],
            [
                tbots_cpp.Point(0.5, 2.0),
                tbots_cpp.Point(0.5, 1.0),
                tbots_cpp.Point(0.5, 0.0),
                tbots_cpp.Point(0.5, -1.0),
                tbots_cpp.Point(0.5, -2.0),
            ],
        ),
        # pass with a dense wall of enemy robots in between the 2 robots
        (
            tbots_cpp.Point(-1.7, 0),
            tbots_cpp.Vector(0.0, 0.0),
            tbots_cpp.Point(-2.0, 0.0),
            [tbots_cpp.Point(2.5, 0.0)],
            [0, math.pi],
            [
                tbots_cpp.Point(0.5, float(y) / 10)
                for y in range(int(-0.5 * 10), int(0.7 * 10), 2)
            ],
        ),
    ],
    ids=[
        "short_pass",
        "long_pass",
        "pass_vertically",
        "pass_diagonally",
        "pass_with_enemy_in_between",
        "pass_through_sparse_wall_of_enemies",
        "pass_through_dense_wall_of_enemies",
    ],
)
def test_passing_receive_speed(
    ball_initial_position,
    ball_initial_velocity,
    attacker_robot_position,
    receiver_robot_positions,
    friendly_orientations,
    enemy_robot_positions,
    simulated_test_runner,
):
    # Eventually Validation
    eventually_validation_sequence_set = [
        [
            BallEventuallyExitsRegion(
                regions=[tbots_cpp.Circle(ball_initial_position, 0.4)]
            ),
            FriendlyEventuallyHasBallPossession(tolerance=0.05),
        ]
    ]

    # Validate that the ball is always received by the other robot
    # slower than the max receive speed
    always_validation_sequence_set = [
        [FriendlyAlwaysReceivesBallSlow(robot_id=1, max_receive_speed=2.3)],
    ]

    simulated_test_runner.run_test(
        setup=lambda param: setup_pass_and_robots(
            ball_initial_position=ball_initial_position,
            ball_initial_velocity=ball_initial_velocity,
            attacker_robot_position=attacker_robot_position,
            receiver_robot_positions=receiver_robot_positions,
            friendly_orientations=friendly_orientations,
            enemy_robot_positions=enemy_robot_positions,
            simulated_test_runner=simulated_test_runner,
            receive_pass=True,
        ),
        params=[0],
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=10,
        run_till_end=False,
    )


@pytest.mark.parametrize(
    "ball_initial_position,ball_initial_velocity,attacker_robot_position,"
    "receiver_robot_positions,friendly_orientations,enemy_robot_positions",
    [
        (
            tbots_cpp.Point(1.7, -2),
            tbots_cpp.Vector(0.0, 0.0),
            tbots_cpp.Point(2.0, -2.0),
            [tbots_cpp.Point(-2.5, 2.0)],
            [math.pi, 0],
            [],
        ),
        (
            tbots_cpp.Point(0.3, 0),
            tbots_cpp.Vector(0.0, 0.0),
            tbots_cpp.Point(0.5, 0),
            [tbots_cpp.Point(-0.5, 0)],
            [math.pi, 0],
            [],
        ),
        (
            tbots_cpp.Point(0.4, 0),
            tbots_cpp.Vector(0.0, 0.0),
            tbots_cpp.Point(0.6, 0),
            [tbots_cpp.Point(-0.6, 0)],
            [math.pi, 0],
            [],
        ),
        (
            tbots_cpp.Point(0.8, 0),
            tbots_cpp.Vector(0.0, 0.0),
            tbots_cpp.Point(1, 0),
            [
                tbots_cpp.Point(-1, 0),
                tbots_cpp.Point(1, 1),
                tbots_cpp.Point(1, -1),
                tbots_cpp.Point(2, 0),
            ],
            [math.pi, 0],
            [],
        ),
        (
            tbots_cpp.Point(0.8, 0),
            tbots_cpp.Vector(0.0, 0.0),
            tbots_cpp.Point(1, 0),
            [tbots_cpp.Point(-1, 0), tbots_cpp.Point(2.5, 2.5)],
            [math.pi, 0],
            [],
        ),
        (
            tbots_cpp.Point(0.4, 0),
            tbots_cpp.Vector(0.0, 0.0),
            tbots_cpp.Point(0.5, 0),
            [tbots_cpp.Point(-1, 0)],
            [math.pi, 0],
            [
                tbots_cpp.Point(0.5, 0.5),
                tbots_cpp.Point(0.5, -0.5),
                tbots_cpp.Point(1, 0),
            ],
        ),
    ],
    ids=[
        "long_pass_backwards",
        "short_pass_backwards",
        "almost_short_pass_backwards",
        "4_pass_options_with_1_backwards",
        "2_pass_options_with_closest_one_backwards",
        "backwards_pass_forced_by_surrounding_enemies",
    ],
)
def test_passing_no_backwards_passes(
    ball_initial_position,
    ball_initial_velocity,
    attacker_robot_position,
    receiver_robot_positions,
    friendly_orientations,
    enemy_robot_positions,
    simulated_test_runner,
):
    field = tbots_cpp.Field.createSSLDivisionBField()
    best_pass = setup_pass_and_robots(
        ball_initial_position=ball_initial_position,
        ball_initial_velocity=ball_initial_velocity,
        attacker_robot_position=attacker_robot_position,
        receiver_robot_positions=receiver_robot_positions,
        friendly_orientations=friendly_orientations,
        enemy_robot_positions=enemy_robot_positions,
        receive_pass=False,
        simulated_test_runner=simulated_test_runner,
    )

    # Eventually Validation
    eventually_validation_sequence_set = [
        [
            BallEventuallyEntersRegion(
                regions=[tbots_cpp.Circle(best_pass.receiverPoint(), 0.2)]
            )
        ]
    ]

    buffer_x = 0.3
    always_validation_sequence_set = [
        [FriendlyAlwaysReceivesBallSlow(robot_id=1, max_receive_speed=2.5)],
        [
            BallMovesForwardInRegions(
                initial_ball_position=ball_initial_position,
                regions=[
                    tbots_cpp.Rectangle(
                        tbots_cpp.Point(-field.xLength() / 2, field.yLength() / 2),
                        tbots_cpp.Point(0 - buffer_x, -field.yLength() / 2),
                    )
                ],
            )
        ],
    ]

    simulated_test_runner.run_test(
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=10,
        run_till_end=False,
    )


if __name__ == "__main__":
    pytest_main(__file__)
