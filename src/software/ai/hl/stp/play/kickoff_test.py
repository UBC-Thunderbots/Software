import sys

import pytest

import software.python_bindings as tbots
from proto.play_pb2 import Play, PlayName
from software.simulated_tests.friendly_has_ball_possession import *
from software.simulated_tests.robot_enters_region import *
from software.simulated_tests.simulated_test_fixture import simulated_test_runner
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team

kickoff_friendly_always_validation = [
    [
        RobotNeverEntersRegion([tbots.Field.createSSLDivisionBField().enemyHalf()]),
        FriendlyNeverHasBallPossession(),
    ]
]

kickoff_friendly_eventually_validation = [
    [
        # Robot in center circle
        NumberOfRobotsEventuallyEntersRegion(
            tbots.Field.createSSLDivisionBField().centerCircle(), 1
        ),
        # Attacking Robot Placement
        NumberOfRobotsEventuallyEntersRegion(
            tbots.Rectangle(tbots.Point(-0.5, 2.5), tbots.Point(-1.5, -2.5)), 2
        ),
        # Defending Robot Placement
        NumberOfRobotsEventuallyEntersRegion(
            tbots.Rectangle(tbots.Point(-3.2, 1.1), tbots.Point(-3.51, -1.1)), 3
        ),
    ]
]

kickoff_enemy_always_validation = [
    [
        RobotNeverEntersRegion(
            [
                tbots.Field.createSSLDivisionBField().enemyHalf(),
                tbots.Field.createSSLDivisionBField().centerCircle(),
            ]
        ),
        FriendlyNeverHasBallPossession(),
    ]
]

kickoff_enemy_eventually_validation = [
    [
        # Front 3 robots spread out in 3 zones
        NumberOfRobotsEventuallyEntersRegion(
            tbots.Rectangle(tbots.Point(-0.2, 1), tbots.Point(-0.4, 0.25)), 1
        ),
        NumberOfRobotsEventuallyEntersRegion(
            tbots.Rectangle(tbots.Point(-0.2, -1), tbots.Point(-0.4, -0.25)), 1
        ),
        NumberOfRobotsEventuallyEntersRegion(
            tbots.Rectangle(tbots.Point(-0.6, 0.1), tbots.Point(-0.9, -0.1)), 1
        ),
        # 2 robots Defend the box
        NumberOfRobotsEventuallyEntersRegion(
            tbots.Rectangle(tbots.Point(-3.2, 1.1), tbots.Point(-3.5, -1.1)), 2
        ),
    ]
]


@pytest.mark.parametrize(
    "always_validation_list,eventually_validation_list,is_friendly_test",
    [
        (
            kickoff_friendly_always_validation,
            kickoff_friendly_eventually_validation,
            True,
        ),
        (kickoff_enemy_always_validation, kickoff_enemy_eventually_validation, False),
    ],
)
def test_kickoff_play(
    simulated_test_runner,
    always_validation_list,
    eventually_validation_list,
    is_friendly_test,
):

    # starting point must be Point
    ball_initial_pos = tbots.Point(0, 0)

    # Setup Bots
    blue_bots = [
        tbots.Point(-3, 2.5),
        tbots.Point(-3, 1.5),
        tbots.Point(-3, 0.5),
        tbots.Point(-3, -0.5),
        tbots.Point(-3, -1.5),
        tbots.Point(-3, -2.5),
    ]

    yellow_bots = [
        tbots.Point(1, 0),
        tbots.Point(1, 2.5),
        tbots.Point(1, -2.5),
        tbots.Field.createSSLDivisionBField().enemyGoalCenter(),
        tbots.Field.createSSLDivisionBField().enemyDefenseArea().negXNegYCorner(),
        tbots.Field.createSSLDivisionBField().enemyDefenseArea().negXPosYCorner(),
    ]

    blue_play = Play()
    yellow_play = Play()

    # Game Controller Setup
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.STOP, team=Team.UNKNOWN
    )
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.NORMAL_START, team=Team.BLUE
    )
    if is_friendly_test:
        simulated_test_runner.gamecontroller.send_ci_input(
            gc_command=Command.Type.KICKOFF, team=Team.BLUE
        )
        blue_play.name = PlayName.KickoffFriendlyPlay
        yellow_play.name = PlayName.KickoffEnemyPlay
    else:
        simulated_test_runner.gamecontroller.send_ci_input(
            gc_command=Command.Type.KICKOFF, team=Team.YELLOW
        )
        blue_play.name = PlayName.KickoffEnemyPlay
        yellow_play.name = PlayName.KickoffFriendlyPlay

    # Force play override here
    simulated_test_runner.blue_full_system_proto_unix_io.send_proto(Play, blue_play)
    simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(Play, yellow_play)

    # Create world state
    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            yellow_robot_locations=yellow_bots,
            blue_robot_locations=blue_bots,
            ball_location=ball_initial_pos,
            ball_velocity=tbots.Vector(0, 0),
        ),
    )

    # Always Validation
    # TODO- #2753 Validation
    always_validation_sequence_set = [[]]

    # Eventually Validation
    # TODO- #2753 Validation
    eventually_validation_sequence_set = [[]]

    simulated_test_runner.run_test(
        eventually_validation_sequence_set=eventually_validation_sequence_set,
        always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=10,
    )


if __name__ == "__main__":
    # Run the test, -s disables all capturing at -vv increases verbosity
    sys.exit(pytest.main([__file__, "-svv"]))
