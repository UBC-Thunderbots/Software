import math

import proto.import_all_protos as protos
import software.python_bindings as tbots_cpp
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team as SslTeam
from software.gameplay_tests.simulated_test_fixture import (
    pytest_main,
)


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

        last_robot = protos.RobotState(
            global_position=protos.Point(x_meters=1, y_meters=2),
            global_velocity=protos.Vector(
                x_component_meters=-4.6, y_component_meters=0
            ),
            global_orientation=protos.Angle(radians=math.pi),
            global_angular_velocity=protos.AngularVelocity(radians_per_second=0),
        )

        world_state.yellow_robots.robot_states[5].CopyFrom(last_robot)

        simulated_test_runner.set_world_state(world_state)

        simulated_test_runner.set_plays(
            blue_play=protos.PlayName.ShootOrChipPlay,
            yellow_play=protos.PlayName.HaltPlay,
        )

        simulated_test_runner.send_gamecontroller_command(
            gc_command=protos.Command.Type.STOP, team=SslTeam.UNKNOWN
        )
        simulated_test_runner.send_gamecontroller_command(
            gc_command=protos.Command.Type.FORCE_START, team=SslTeam.BLUE
        )

    # TODO (#3651): add validations
    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=[[]],
        test_timeout_s=10,
    )


if __name__ == "__main__":
    pytest_main(__file__)
