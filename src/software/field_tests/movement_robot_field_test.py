from proto.import_all_protos import *
from software.field_tests.field_test_fixture import *

from software.simulated_tests.simulated_test_fixture import *
from software.logger.logger import create_logger

logger = create_logger(__name__)
import math
import threading

# this test can only be run on the field
def test_basic_rotation(field_test_runner):
    test_angles = [0, 45, 90, 180, 270, 0]

    world = field_test_runner.world_buffer.get(block=True, timeout=WORLD_BUFFER_TIMEOUT)
    if len(world.friendly_team.team_robots) == 0:
        raise Exception("The first world received had no robots in it!")

    print("Here are the robots:")
    print(
        [
            robot.current_state.global_position
            for robot in world.friendly_team.team_robots
        ]
    )

    id = world.friendly_team.team_robots[0].id
    print(f"Running test on robot {id}")

    robot = world.friendly_team.team_robots[0]
    rob_pos_p = robot.current_state.global_position

    def execute_test():
        # Wait for the user to flip the estop to PLAY
        field_test_runner.wait_for_estop_play()

        # Force start the game automatically
        field_test_runner.gamecontroller.send_gc_command(
            proto.ssl_gc_state_pb2.Command.FORCE_START,
            proto.ssl_gc_common_pb2.Team.UNKNOWN,
            final_ball_placement_point=None,
        )

        for angle in test_angles:
            print(f"Rotating to {angle} degrees")
            move_tactic = MoveTactic()
            move_tactic.destination.CopyFrom(rob_pos_p)
            move_tactic.dribbler_mode = DribblerMode.OFF
            move_tactic.final_orientation.CopyFrom(Angle(radians=angle * math.pi / 180.0))
            move_tactic.ball_collision_type = BallCollisionType.AVOID
            move_tactic.auto_chip_or_kick.CopyFrom(
                AutoChipOrKick(autokick_speed_m_per_s=0.0)
            )
            move_tactic.max_allowed_speed_mode = MaxAllowedSpeedMode.PHYSICAL_LIMIT
            move_tactic.obstacle_avoidance_mode = ObstacleAvoidanceMode.SAFE

            # Setup Tactic
            field_test_runner.set_tactics(
                blue_tactics={id: move_tactic}, yellow_tactics=None
            )
            field_test_runner.run_test(
                always_validation_sequence_set=[[]],
                eventually_validation_sequence_set=[[]],
                test_timeout_s=5,
            )

            # validate by eye
            logger.info(f"robot set to {angle} orientation")
            time.sleep(2)

        # Send a halt tactic after the test finishes
        field_test_runner.set_tactics(blue_tactics={id: HaltTactic()}, yellow_tactics=None)

    test_thread = threading.Thread(target=execute_test, daemon=True)
    test_thread.start()

    # If thunderscope is enabled, show it in the main thread (blocking)
    if field_test_runner.thunderscope:
        field_test_runner.thunderscope.show()
    
    test_thread.join()


def test_one_robots_square(field_test_runner):
    world = field_test_runner.world_buffer.get(block=True, timeout=WORLD_BUFFER_TIMEOUT)
    if len(world.friendly_team.team_robots) == 0:
        raise Exception("The first world received had no robots in it!")

    print("Here are the robots:")
    print(
        [
            robot.current_state.global_position
            for robot in world.friendly_team.team_robots
        ]
    )

    id = world.friendly_team.team_robots[0].id
    print(f"Running test on robot {id}")

    point1 = Point(x_meters=0.5, y_meters=0.4)
    point2 = Point(x_meters=0.5, y_meters=-0.4)
    point3 = Point(x_meters=-0.5, y_meters=-0.4)
    point4 = Point(x_meters=-0.5, y_meters=0.4)

    tactic_0 = MoveTactic(
        destination=point1,
        dribbler_mode=DribblerMode.OFF,
        final_orientation=Angle(radians=-math.pi / 2),
        ball_collision_type=BallCollisionType.AVOID,
        auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=0.0),
        max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
        obstacle_avoidance_mode=ObstacleAvoidanceMode.SAFE,
    )
    tactic_1 = MoveTactic(
        destination=point2,
        dribbler_mode=DribblerMode.OFF,
        final_orientation=Angle(radians=-math.pi / 2),
        ball_collision_type=BallCollisionType.AVOID,
        auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=0.0),
        max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
        obstacle_avoidance_mode=ObstacleAvoidanceMode.SAFE,
    )
    tactic_2 = MoveTactic(
        destination=point3,
        dribbler_mode=DribblerMode.OFF,
        final_orientation=Angle(radians=-math.pi / 2),
        ball_collision_type=BallCollisionType.AVOID,
        auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=0.0),
        max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
        obstacle_avoidance_mode=ObstacleAvoidanceMode.SAFE,
    )
    tactic_3 = MoveTactic(
        destination=point4,
        dribbler_mode=DribblerMode.OFF,
        final_orientation=Angle(radians=-math.pi / 2),
        ball_collision_type=BallCollisionType.AVOID,
        auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=0.0),
        max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
        obstacle_avoidance_mode=ObstacleAvoidanceMode.SAFE,
    )

    tactics = [tactic_0, tactic_1, tactic_2, tactic_3, tactic_0]

    def execute_test():
        # Wait for the user to flip the estop to PLAY
        field_test_runner.wait_for_estop_play()

        # Force start the game automatically
        field_test_runner.gamecontroller.send_gc_command(
            proto.ssl_gc_state_pb2.Command.FORCE_START,
            proto.ssl_gc_common_pb2.Team.UNKNOWN,
            final_ball_placement_point=None,
        )

        for tactic in tactics:
            print(f"Going to {tactic.destination}")

            field_test_runner.set_tactics(
                blue_tactics={
                    id: tactic,
                },
                yellow_tactics=None,
            )
            field_test_runner.run_test(
                always_validation_sequence_set=[[]],
                eventually_validation_sequence_set=[[]],
                test_timeout_s=4,
            )

        # Send a halt tactic after the test finishes
        field_test_runner.set_tactics(blue_tactics={id: HaltTactic()}, yellow_tactics=None)

    test_thread = threading.Thread(target=execute_test, daemon=True)
    test_thread.start()

    # If thunderscope is enabled, show it in the main thread (blocking)
    if field_test_runner.thunderscope:
        field_test_runner.thunderscope.show()
    
    test_thread.join()


if __name__ == "__main__":
    pytest_main(__file__)
