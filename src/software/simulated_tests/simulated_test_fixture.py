import sys
import threading
import time

import pytest
from proto.geometry_pb2 import Angle, AngularVelocity, Point, Vector
from proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
from proto.primitive_pb2 import MaxAllowedSpeedMode
from proto.robot_status_msg_pb2 import RobotStatus
from proto.sensor_msg_pb2 import SensorProto
from proto.tactic_pb2 import AssignedTacticPlayControlParams, GoalieTactic, Tactic
from proto.vision_pb2 import BallState, RobotState
from proto.world_pb2 import SimulatorTick, World, WorldState, ValidationVisualization
from pyqtgraph.Qt import QtCore, QtGui

import software.simulated_tests.python_bindings as py
from software.simulated_tests.full_system_wrapper import FullSystemWrapper
from software.simulated_tests.standalone_simulator_wrapper import (
    StandaloneSimulatorWrapper,
)
from software.thunderscope.thunderscope import Thunderscope


class TacticTestRunner(object):

    """Run a tactic"""

    def __init__(self, launch_delay_s=0.1):
        """Initialize the TacticTestRunner

        :param launch_delay_s: How long to wait after launching 

        """
        self.thunderscope = Thunderscope()
        self.simulator = StandaloneSimulatorWrapper()
        self.yellow_full_system = FullSystemWrapper()
        time.sleep(launch_delay_s)

    def run_test(
        self,
        always_validation=[],
        eventually_validation=[],
        test_timeout_s=3,
        tick_duration_s=0.01,
        open_thunderscope=True,
    ):
        """Run a test

        :param always_validation: Validation functions that should hold on every tick
        :param eventually_validation: Validation that should eventually be true,
                                      before the test ends
        :param test_timeout_s: The timeout for the test, if any eventually_validations
                                remain after the timeout, the test fails.
        :param tick_duration_s: The simulation step duration
        :param open_thunderscope: If true, thunderscope opens and the test runs
                                  in realtime
        """

        def __stopper():
            self.simulator.standalone_simulator_process.kill()
            self.yellow_full_system.full_system_process.kill()
            self.simulator.standalone_simulator_process.wait()
            self.yellow_full_system.full_system_process.wait()
            self.thunderscope.close()

        def __runner():
            time_elapsed_s = 0

            while time_elapsed_s < test_timeout_s:
                time.time()

                ssl_wrapper = self.simulator.get_ssl_wrapper_packet()
                self.simulator.tick(tick_duration_s * 1000)
                time_elapsed_s += tick_duration_s

                # if we wanted to open thunderscope, lets sleep
                # until this ssl wrapper packet is delivered
                # TODO fix this duration
                if open_thunderscope:
                    time.sleep(tick_duration_s)

                if ssl_wrapper is None:
                    continue

                yellow_sensor_proto = self.simulator.get_yellow_sensor_proto(
                    ssl_wrapper
                )
                self.yellow_full_system.send_sensor_proto(yellow_sensor_proto)

                self.simulator.send_yellow_primitive_set_and_vision(
                    self.yellow_full_system.get_vision(),
                    self.yellow_full_system.get_primitive_set(),
                )

            __stopper()

        run_sim_thread = threading.Thread(target=__runner)
        run_sim_thread.start()

        if open_thunderscope:
            self.thunderscope.show()


@pytest.fixture
def tactic_runner():
    runner = TacticTestRunner()
    yield runner


@pytest.mark.parametrize(
    "goalie_starting_position,ball_starting_position",
    [
        ((-4.2, 0), (-2, 1)),
        ((-4.2, 0.4), (-2, -1)),
        ((-4.2, -0.4), (-2, 0.1)),
        ((-4.2, 0.2), (-2, -0.1)),
        ((-4.2, -0.2), (-2, 1)),
        ((-4.2, 0.2), (-2, -0.1)),
        ((-4.2, -0.5), (-2, 2)),
        ((-4.2, -0.5), (-2, -2)),
    ],
)
def test_goalie_blocks_shot(
    goalie_starting_position, ball_starting_position, tactic_runner
):
    tactic_runner.simulator.setup_yellow_robots([goalie_starting_position])
    tactic_runner.simulator.setup_ball(
        ball_position=ball_starting_position,
        # TODO need a vector library here
        ball_velocity=(
            2.5 * (-4.5 - ball_starting_position[0]),
            -2.5 * ball_starting_position[1],
        ),
    )

    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[0].goalie.CopyFrom(
        GoalieTactic(max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT)
    )

    tactic_runner.yellow_full_system.send_tactic_override(params)
    tactic_runner.run_test()


# ball_at_point_validation.cpp
# ball_kicked_validation.h
# friendly_scored_validation.cpp
# robot_halt_validation.cpp
# robot_in_circle.cpp
# robot_in_polygon_validation.cpp
# robot_received_ball_validation.cpp
#   - all dribblers highlighted red
#   - if a robot receives a ball, they all go green
# robot_state_validation.cpp
# robot_stationary_in_polygon_validation.cpp

if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-svv"]))
