import software.python_bindings as tbots_cpp
from software.simulated_tests.robot_enters_region import *
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.ball_moves_in_direction import *
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)
from proto.message_translation.tbots_protobuf import create_world_state

field = tbots_cpp.Field.createSSLDivisionBField()


def test_move_across_field(simulated_test_runner):
    initial_position = tbots_cpp.Point(-3, 1.5)
    destination = tbots_cpp.Point(2.5, -1.1)

    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            blue_robot_locations=[
                tbots_cpp.Point(-3, 2.5),
                initial_position,
            ],
            yellow_robot_locations=[
                tbots_cpp.Point(1, 0),
                tbots_cpp.Point(1, 2.5),
                tbots_cpp.Point(1, -2.5),
                field.enemyGoalCenter(),
                field.enemyDefenseArea().negXNegYCorner(),
                field.enemyDefenseArea().negXPosYCorner(),
            ],
            ball_location=tbots_cpp.Point(4.5, -3),
            ball_velocity=tbots_cpp.Vector(0, 0),
        ),
    )

    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[1].move.CopyFrom(
        MoveTactic(
            destination=tbots_cpp.createPointProto(destination),
            final_orientation=tbots_cpp.createAngleProto(tbots_cpp.Angle.zero()),
            dribbler_mode=DribblerMode.OFF,
            ball_collision_type=BallCollisionType.AVOID,
            auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=0.0),
            max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
            obstacle_avoidance_mode=ObstacleAvoidanceMode.SAFE,
        )
    )
    simulated_test_runner.blue_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    params = AssignedTacticPlayControlParams()
    simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    eventually_validation_sequence_set = [
        [
            RobotEventuallyEntersRegion(regions=[tbots_cpp.Circle(destination, 0.05)]),
        ]
    ]

    # TODO (#2558): should also validate that the robot stays at destination for 1000 ticks after

    # TODO (#2558): check if there is better way to pass these arguments cleaner
    simulated_test_runner.run_test(
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_always_validation_sequence_set=[[]],
    )


def test_autochip_move(simulated_test_runner):
    initial_position = tbots_cpp.Point(-3, 1.5)
    destination = tbots_cpp.Point(0, 1.5)

    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            blue_robot_locations=[
                tbots_cpp.Point(-3, 2.5),
                initial_position,
            ],
            yellow_robot_locations=[
                tbots_cpp.Point(1, 0),
                tbots_cpp.Point(1, 2.5),
                tbots_cpp.Point(1, -2.5),
                field.enemyGoalCenter(),
                field.enemyDefenseArea().negXNegYCorner(),
                field.enemyDefenseArea().negXPosYCorner(),
            ],
            ball_location=destination,
            ball_velocity=tbots_cpp.Vector(0, 0),
        ),
    )

    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[1].move.CopyFrom(
        MoveTactic(
            destination=tbots_cpp.createPointProto(destination),
            final_orientation=tbots_cpp.createAngleProto(tbots_cpp.Angle.zero()),
            dribbler_mode=DribblerMode.OFF,
            ball_collision_type=BallCollisionType.ALLOW,
            auto_chip_or_kick=AutoChipOrKick(
                autochip_distance_meters=2.0,
            ),
            max_allowed_speed_mode=MaxAllowedSpeedMode.COLLISIONS_ALLOWED,
            obstacle_avoidance_mode=ObstacleAvoidanceMode.SAFE,
        )
    )
    simulated_test_runner.blue_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    params = AssignedTacticPlayControlParams()
    simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    eventually_validation_sequence_set = [
        [
            RobotEventuallyEntersRegion(regions=[tbots_cpp.Circle(destination, 0.05)]),
            # TODO (#2558): validate that ball gets kicked at an angle zero
        ]
    ]

    # TODO (#2558): should also validate that the robot stays at destination for 1000 ticks after

    simulated_test_runner.run_test(
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_always_validation_sequence_set=[[]],
        test_timeout_s=10,
    )


def test_autokick_move(simulated_test_runner):
    initial_position = tbots_cpp.Point(-1, -0.5)
    destination = tbots_cpp.Point(-1, -1)

    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            blue_robot_locations=[initial_position],
            blue_robot_orientations=[tbots_cpp.Angle.threeQuarter()],
            yellow_robot_locations=[
                tbots_cpp.Point(1, 0),
                tbots_cpp.Point(1, 2.5),
                tbots_cpp.Point(1, -2.5),
                field.enemyGoalCenter(),
                field.enemyDefenseArea().negXNegYCorner(),
                field.enemyDefenseArea().negXPosYCorner(),
            ],
            ball_location=destination,
            ball_velocity=tbots_cpp.Vector(0, 0),
        ),
    )

    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[0].move.CopyFrom(
        MoveTactic(
            destination=tbots_cpp.createPointProto(destination),
            final_orientation=tbots_cpp.createAngleProto(
                tbots_cpp.Angle.threeQuarter()
            ),
            dribbler_mode=DribblerMode.OFF,
            ball_collision_type=BallCollisionType.ALLOW,
            auto_chip_or_kick=AutoChipOrKick(
                autokick_speed_m_per_s=3.0,
            ),
            max_allowed_speed_mode=MaxAllowedSpeedMode.COLLISIONS_ALLOWED,
            obstacle_avoidance_mode=ObstacleAvoidanceMode.SAFE,
        )
    )
    simulated_test_runner.blue_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    params = AssignedTacticPlayControlParams()
    simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    eventually_validation_sequence_set = [
        [
            RobotEventuallyEntersRegion(regions=[tbots_cpp.Circle(destination, 0.05)]),
            # TODO (#2558): validate that ball gets kicked at an angle threeQuarter
        ]
    ]

    # TODO (#2558): should also validate that the robot stays at destination for 1000 ticks after

    simulated_test_runner.run_test(
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_always_validation_sequence_set=[[]],
    )


def test_spinning_move_clockwise(simulated_test_runner):
    initial_position = tbots_cpp.Point(-4, 2)
    destination = tbots_cpp.Point(4, 2)

    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            blue_robot_locations=[initial_position],
            # TODO (#2558): implement initializing robot angular velocities
            # blue_robot_angular_velocities=[tbots_cpp.AngularVelocity.quarter()],
            yellow_robot_locations=[tbots_cpp.Point(4, 0)],
            ball_location=tbots_cpp.Point(1, 1),
            ball_velocity=tbots_cpp.Vector(0, 0),
        ),
    )

    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[0].move.CopyFrom(
        MoveTactic(
            destination=tbots_cpp.createPointProto(destination),
            final_orientation=tbots_cpp.createAngleProto(tbots_cpp.Angle.zero()),
            dribbler_mode=DribblerMode.OFF,
            ball_collision_type=BallCollisionType.ALLOW,
            auto_chip_or_kick=AutoChipOrKick(),
            max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
            obstacle_avoidance_mode=ObstacleAvoidanceMode.SAFE,
        )
    )
    simulated_test_runner.blue_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    params = AssignedTacticPlayControlParams()
    simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    eventually_validation_sequence_set = [
        [
            RobotEventuallyEntersRegion(regions=[tbots_cpp.Circle(destination, 0.05)]),
            # TODO (#2558): validate robot is at orientation zero
            # TODO (#2558): validate robot is at angular velocity something
            #               (bug with angular velocities so the original c++ test is actual wrong)
        ]
    ]

    simulated_test_runner.run_test(
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_always_validation_sequence_set=[[]],
        test_timeout_s=10,
    )


def test_spinning_move_counter_clockwise(simulated_test_runner):
    initial_position = tbots_cpp.Point(4, 2)
    destination = tbots_cpp.Point(-4, 2)

    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            blue_robot_locations=[initial_position],
            blue_robot_orientations=[tbots_cpp.Angle.quarter()],
            yellow_robot_locations=[tbots_cpp.Point(4, 0)],
            ball_location=tbots_cpp.Point(1, 1),
            ball_velocity=tbots_cpp.Vector(0, 0),
        ),
    )

    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[0].move.CopyFrom(
        MoveTactic(
            destination=tbots_cpp.createPointProto(destination),
            final_orientation=tbots_cpp.createAngleProto(tbots_cpp.Angle.half()),
            dribbler_mode=DribblerMode.OFF,
            ball_collision_type=BallCollisionType.ALLOW,
            auto_chip_or_kick=AutoChipOrKick(),
            max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
            obstacle_avoidance_mode=ObstacleAvoidanceMode.SAFE,
        )
    )
    simulated_test_runner.blue_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    params = AssignedTacticPlayControlParams()
    simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    eventually_validation_sequence_set = [
        [
            RobotEventuallyEntersRegion(regions=[tbots_cpp.Circle(destination, 0.05)]),
            # TODO (#2558): validate robot is at orientation half
            # TODO (#2558): validate robot is at angular velocity something
            #               (bug with angular velocities so the original c++ test is actual wrong)
        ]
    ]

    simulated_test_runner.run_test(
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_always_validation_sequence_set=[[]],
        test_timeout_s=10,
    )


if __name__ == "__main__":
    pytest_main(__file__)
