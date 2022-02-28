import software.simulated_tests.python_bindings as py

# from hanging_threads import start_monitoring
# start_monitoring(seconds_frozen=10, test_interval=100)
from proto.sensor_msg_pb2 import SensorProto
from proto.world_pb2 import WorldState, SimulatorTick, World
import os
from proto.vision_pb2 import RobotState, BallState
import threading
import logging
from pyqtgraph.Qt import QtCore, QtGui
import sys
from proto.geometry_pb2 import Point, Angle, Vector, AngularVelocity
from proto.primitive_pb2 import MaxAllowedSpeedMode
from proto.robot_status_msg_pb2 import RobotStatus
from typing import List
from proto.tactic_pb2 import AssignedTacticPlayControlParams, GoalieTactic, Tactic
from proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
from software.thunderscope.thunderscope import Thunderscope
import threading
from software.simulated_tests.standalone_simulator_wrapper import (
    StandaloneSimulatorWrapper,
)
from software.simulated_tests.full_system_wrapper import FullSystemWrapper
import time
import pytest


class TacticTestRunner(object):

    """Run a tactic"""

    def __init__(self, launch_delay_s=1):
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
            # self.yellow_full_system.stop()
            self.simulator.standalone_simulator_process.kill()
            self.yellow_full_system.full_system_process.kill()
            self.thunderscope.close()
            self.simulator.standalone_simulator_process.wait()
            self.yellow_full_system.full_system_process.wait()

        def __runner():
            time_elapsed_s = 0

            while time_elapsed_s < test_timeout_s:
                tick_start_time = time.time()

                ssl_wrapper = self.simulator.get_ssl_wrapper_packet()
                self.simulator.tick(tick_duration_s * 1000)
                time_elapsed_s += tick_duration_s

                # if we wanted to open thunderscope, lets sleep
                # until this ssl wrapper packet is delivered
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

        # This will block forever, we rely on the timer above to fire and close
        # thunderscope.
        if open_thunderscope:
            self.thunderscope.show()


@pytest.mark.parametrize("test_input", [(2, 0), (1, 0), (-2, 0), (3, 3), (-1, -1)])
def test_goalie_blocks_shot(test_input):
    tactic_runner = TacticTestRunner()
    tactic_runner.simulator.setup_yellow_robots([test_input])
    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[0].goalie.CopyFrom(
        GoalieTactic(max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT)
    )
    tactic_runner.yellow_full_system.send_tactic_override(params)
    tactic_runner.run_test()
    assert False


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
