import software.simulated_tests.python_bindings as py
from proto.sensor_msg_pb2 import SensorProto
from proto.world_pb2 import WorldState, SimulatorTick, World
from proto.vision_pb2 import RobotState, BallState
import threading
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


def main():

    # visualizer
    thunderscope = Thunderscope()

    def step_simulation():

        # setup simulator
        simulator = StandaloneSimulatorWrapper()
        time.sleep(0.1)  # wait for the sim to start

        simulator.setup_blue_robots([(0, 0), (1, 1)])
        simulator.setup_yellow_robots([(-1, -1), (-2, -2)])

        # setup full_system
        yellow_full_system = FullSystemWrapper()
        time.sleep(0.1)

        params = AssignedTacticPlayControlParams()
        tactic = GoalieTactic(max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT)
        params.assigned_tactics[0].goalie.CopyFrom(tactic)
        time.sleep(1)
        yellow_full_system.send_tactic_override(params)

        while True:
            ssl_wrapper = simulator.get_ssl_wrapper_packet()
            simulator.tick(10)

            if ssl_wrapper is None:
                continue

            yellow_sensor_proto = simulator.get_yellow_sensor_proto(ssl_wrapper)
            yellow_full_system.send_sensor_proto(yellow_sensor_proto)

            simulator.send_yellow_primitive_set_and_vision(
                yellow_full_system.get_vision(), yellow_full_system.get_primitive_set()
            )
            time.sleep(0.01)

    run_sim_thread = threading.Thread(target=step_simulation)
    run_sim_thread.start()

    thunderscope.show()


VALIDATION_1 = 0
VALIDATION_2 = 0


class Validation1(object):
    def __init__(self):
        self.done = False
        self.validation = 0

    def validate(self, world):
        if world == 5:
            self.done = True
        return self.done


class Validation2(object):
    def __init__(self):
        self.done = False

    def validate(self, world):
        if world == 15:
            self.done = True
        return self.done


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


def run_tactic_test(
    simulator,
    full_system,
    # always_validation: List[AlwaysValidation],
    # eventually_validation: List[EventuallyValidation],
):
    main()


def test_attacker_tactic_keepaway():
    """TODO: Docstring for test_attacker_tactic_keepaway.

    :param function: TODO
    :returns: TODO

    """
    main()


if __name__ == "__main__":
    main()
    # sys.exit(pytest.main([__file__, "-svv"]))
