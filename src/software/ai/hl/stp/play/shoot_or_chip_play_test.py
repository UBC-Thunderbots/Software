import math

import pytest

import software.python_bindings as tbots_cpp
from proto.play_pb2 import Play, PlayName
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.simulated_test_fixture import (
    simulated_test_runner,
    pytest_main,
)
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team
from proto.geometry_pb2 import Point, Vector, Angle, AngularVelocity

from proto.ssl_gc_geometry_pb2 import Vector2


def test_shoot_or_chip_play(simulated_test_runner):
    def setup(*args):
        ball_initial_pos = tbots_cpp.Point(-1.4, 2)
        ball_initial_vel = tbots_cpp.Vector(0, 0)

        field = tbots_cpp.Field.createSSLDivisionBField()

        blue_bots = [
            field.friendlyGoalCenter(),
            tbots_cpp.Point(-1.5, 2),
            tbots_cpp.Point(-2, 1.5),
            tbots_cpp.Point(-2, 0.5),
            tbots_cpp.Point(-2, -0.5),
            tbots_cpp.Point(-2, -1.5),
        ]

        yellow_bots = [
            field.enemyGoalCenter(),
            field.enemyDefenseArea().negXNegYCorner(),
            field.enemyDefenseArea().negXPosYCorner(),
            tbots_cpp.Point(-1, 0),
            tbots_cpp.Point(1, -2.5),
        ]

        world_state = create_world_state(
            yellow_robot_locations=yellow_bots,
            blue_robot_locations=blue_bots,
            ball_location=ball_initial_pos,
            ball_velocity=ball_initial_vel,
        )

        last_robot = RobotState(
            global_position=Point(x_meters=1, y_meters=2),
            global_velocity=Vector(x_component_meters=-4.6, y_component_meters=0),
            global_orientation=Angle(radians=math.pi),
            global_angular_velocity=AngularVelocity(radians_per_second=0),
        )

        world_state.yellow_robots[5].CopyFrom(last_robot)

        # Game Controller Setup
        simulated_test_runner.gamecontroller.send_gc_command(
            gc_command=Command.Type.STOP, team=Team.UNKNOWN
        )
        simulated_test_runner.gamecontroller.send_gc_command(
            gc_command=Command.Type.FORCE_START, team=Team.BLUE
        )

        blue_play = Play()
        blue_play.name = PlayName.ShootOrChipPlay

        simulated_test_runner.blue_full_system_proto_unix_io.send_proto(Play, blue_play)

        simulated_test_runner.simulator_proto_unix_io.send_proto(
            WorldState, world_state
        )

    simulated_test_runner.run_test(
        setup=setup,
        # this array is just so that the test runs 5 times
        # if needed, actual arguments can be passed in to customize each test iteration
        params=[0, 1, 2, 3, 4],
        inv_eventually_validation_sequence_set=[[]],
        inv_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=[[]],
        ag_always_validation_sequence_set=[[]],
        test_timeout_s=10,
    )


if __name__ == "__main__":
    pytest_main(__file__)
