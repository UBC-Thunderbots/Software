import sys

import pytest
import itertools

import software.python_bindings as tbots
from software.simulated_tests.robot_enters_region import *
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.ball_moves_forward import *
from software.simulated_tests.friendly_has_ball_possession import *
from software.simulated_tests.ball_speed_threshold import *
from software.simulated_tests.robot_speed_threshold import *
from software.simulated_tests.excessive_dribbling import *
from software.simulated_tests.simulated_test_fixture import simulated_test_runner
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team

# Unit vectors that represent the angles at which the ball will be moving
BALL_UNIT_VELOCITIES = [tbots.Vector.createFromAngle(tbots.Angle.fromDegrees(angle)) for angle in [0,5,20,30]]

# slow, medium, and fast ball speeds
BALL_SPEEDS = [2.0,4.0,6.0]

BALL_VELOCITIES = [s * u for s in BALL_SPEEDS for u in BALL_UNIT_VELOCITIES]

ROBOT_INITIAL_POSITIONS = [tbots.Field.createSSLDivisionBField().centerPoint()]

# positions relative to robots initial position
BALL_INITIAL_POSITIONS =  [tbots.Field.createSSLDivisionBField().centerPoint() + tbots.Vector(*p) for p in [(-1,0), (-2,0), (-4,0)] ]

# cartesian product of parameters
BASIC_TEST_CASE_PARAMETERS = itertools.product(BALL_INITIAL_POSITIONS, BALL_VELOCITIES, ROBOT_INITIAL_POSITIONS)

print(BASIC_TEST_CASE_PARAMETERS)
@pytest.mark.parametrize(
    "ball_initial_position,ball_initial_velocity,robot_initial_position",
    [
        # stationary ball
        (
            tbots.Field.createSSLDivisionBField().centerPoint(),
            tbots.Vector(2.0, 0),
            tbots.Field.createSSLDivisionBField().friendlyGoalpostPos() + tbots.Vector(2,0),
        ),
        #*BASIC_TEST_CASE_PARAMETERS
        # slow ball

    ],
)
def test_robot_intercepts_ball(
    ball_initial_position,
    ball_initial_velocity,
    robot_initial_position,
    simulated_test_runner,
):
    print(WorldState)
    # Setup Robot
    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            [],
            blue_robot_locations=[robot_initial_position],
            ball_location=ball_initial_position,
            ball_velocity=ball_initial_velocity,
        ),
    )

    # These aren't necessary for this test, but this is just an example
    # of how to send commands to the simulator.
    #
    # NOTE: The gamecontroller responses are automatically handled by
    # the gamecontroller context manager class
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.STOP, team=Team.UNKNOWN
    )
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.FORCE_START, team=Team.BLUE
    )

    # Setup Tactic
    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[0].dribble.CopyFrom(
        DribbleTactic(allow_excessive_dribbling=False)
    )
    simulated_test_runner.blue_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    # Setup no tactics on the enemy side
    params = AssignedTacticPlayControlParams()
    simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    # Always Validation
    always_validation_sequence_set = [
        [
            NeverExcessivelyDribbles(),
        ]
    ]

    # Eventually Validation
    eventually_validation_sequence_set = [
        [
            FriendlyHasBallPossession(),
        ]
    ]

    simulated_test_runner.run_test(
        test_timeout_s=5,
        eventually_validation_sequence_set=eventually_validation_sequence_set,
        always_validation_sequence_set=always_validation_sequence_set,
    )


if __name__ == "__main__":
    # Run the test, -s disables all capturing at -vv increases verbosity
    sys.exit(pytest.main([__file__, "-svv"]))
