import pytest
import software.python_bindings as tbots
from software.ai.hl.stp.tactic.dribble.dribble_tactic import DribbleTactic
from software.geom import Point, Angle, Vector
from software.simulated_tests.simulated_test_fixture import simulated_test_runner, pytest_main
from software.simulated_tests.validation_functions import *
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Command, Team
from software.world.world import World
from software.time.duration import Duration

@pytest.mark.parametrize(
    "initial_position, dribble_destination, dribble_orientation, ball_state, test_duration",
    [
        # Test case: Close proximity slow moving ball
        (Point(-3, 0), Point(-3, 0.5), Angle.half(), BallState(Point(-3, 0.5), Vector(0, -0.1)), Duration.from_seconds(10)),
        # Test case: Close proximity stationary ball
        (Point(-3, 0), Point(-3, 0.1), Angle.half(), BallState(Point(-3, 0.1), Vector(0, 0)), Duration.from_seconds(10)),
        # Test case: Close proximity fast moving ball
        (Point(-3, 0), Point(-3, -0.1), Angle.half(), BallState(Point(-3, -0.1), Vector(1, 2)), Duration.from_seconds(10)),
    ]
)
def test_dribble_tactic(initial_position, dribble_destination, dribble_orientation, ball_state, test_duration, simulated_test_runner):
    ai_config = TbotsProto.AiConfig()
    motion_constraints = {TbotsProto.MotionConstraint.ENEMY_DEFENSE_AREA}

    # Setup Robot and Ball initial states
    world_state = create_world_state(
        [],
        blue_robot_locations=[initial_position],
        ball_location=ball_state.position,
        ball_velocity=ball_state.velocity
    )
    simulated_test_runner.simulator_proto_unix_io.send_proto(WorldState, world_state)

    # Setup Dribble Tactic
    tactic = DribbleTactic(ai_config)
    tactic.update_control_params(dribble_destination, dribble_orientation, True)
    
    # Setup Validation Sequence
    eventually_validation_sequence_set = [
        lambda: check_possession(tactic, world_ptr, yield_func),
        lambda: ball_at_point(dribble_destination, world_ptr, yield_func),
        lambda: robot_at_orientation(1, world_ptr, dribble_orientation, Angle.from_degrees(5), yield_func),
        lambda: check_possession(tactic, world_ptr, yield_func)
    ]

    # Running the test
    simulated_test_runner.run_test(
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        test_duration=test_duration
    )

if __name__ == "__main__":
    pytest_main(__file__)
