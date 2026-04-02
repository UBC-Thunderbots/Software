import math


import software.python_bindings as tbots_cpp
from proto.play_pb2 import Play, PlayName
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)
from proto.import_all_protos import *
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team
from proto.geometry_pb2 import Point, Vector, Angle, AngularVelocity


def test_shoot_or_chip_play(simulated_test_runner):
    def setup(*args):
        ball_initial_pos = tbots_cpp.Point(-1.4, 2)
        ball_initial_vel = tbots_cpp.Vector(0, 0)

        field = tbots_cpp.Field.createSSLDivisionBField()

        world_state = create_world_state(
            blue_robot_locations=[
                field.friendlyGoalCenter(),
                tbots_cpp.Point(-1.5, 2),
                tbots_cpp.Point(-2, 1.5),
                tbots_cpp.Point(-2, 0.5),
                tbots_cpp.Point(-2, -0.5),
                tbots_cpp.Point(-2, -1.5),
            ],
            yellow_robot_locations=[
                field.enemyGoalCenter(),
                field.enemyDefenseArea().negXNegYCorner(),
                field.enemyDefenseArea().negXPosYCorner(),
                tbots_cpp.Point(-1, 0),
                tbots_cpp.Point(1, -2.5),
            ],
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

        simulated_test_runner.set_world_state(world_state)

        simulated_test_runner.set_plays(blue_play=PlayName.ShootOrChipPlay, yellow_play=PlayName.HaltPlay)

        simulated_test_runner.send_gamecontroller_command(
            gc_command=Command.Type.STOP, team=Team.UNKNOWN
        )
        simulated_test_runner.send_gamecontroller_command(
            gc_command=Command.Type.FORCE_START, team=Team.BLUE
        )

    # TODO (#3632): add validations
    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=[[]],
        test_timeout_s=10,
    )


if __name__ == "__main__":
    pytest_main(__file__)
