import software.simulated_tests.python_bindings as py
from proto.sensor_msg_pb2 import SensorProto
from proto.world_pb2 import WorldState, SimulatorTick, World
from proto.vision_pb2 import RobotState, BallState
import threading
from pyqtgraph.Qt import QtCore, QtGui
import sys
from proto.geometry_pb2 import Point, Angle, Vector, AngularVelocity
from proto.robot_status_msg_pb2 import RobotStatus
from typing import List
from proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
from software.thunderscope.thunderscope import Thunderscope
from software.simulated_tests.standalone_simulator_wrapper import (
    StandaloneSimulatorWrapper,
)
import time
import pytest


def main():

    thunderscope = Thunderscope()

    # setup simulator
    sim_wrapper = StandaloneSimulatorWrapper()
    # create tactics
    attacker_tactic = py.AttackerTactic(py.AttackerTacticConfig())
    goalie_tactic = py.GoalieTactic(py.GoalieTacticConfig())

    # create tactic steppers
    attacker_tactic_stepper = py.TacticStepper(
        attacker_tactic, set(), py.ThunderbotsConfig(),
    )

    goalie_tactic_stepper = py.TacticStepper(
        goalie_tactic, set(), py.ThunderbotsConfig(),
    )

    # create sensor fusions
    yellow_sensor_fusion_config = py.SensorFusionConfig()
    yellow_sensor_fusion = py.SensorFusion(yellow_sensor_fusion_config)

    blue_sensor_fusion_config = py.SensorFusionConfig()
    blue_sensor_fusion_config.getMutableFriendlyColorYellow().setValue(False)
    blue_sensor_fusion_config.getMutableDefendingPositiveSide().setValue(True)
    blue_sensor_fusion = py.SensorFusion(blue_sensor_fusion_config)

    # Visualize currently active validation
    # Multiple streams of validation
    # AlwaysValidation
    # EventuallyValidation
    # 
    
    def run_lol():
        sim_wrapper.tick(5)
        ssl_wrapper = sim_wrapper.get_ssl_wrapper_packet()
        if ssl_wrapper is None:
            return

        yellow_sensor_proto = sim_wrapper.get_yellow_sensor_proto(ssl_wrapper)
        yellow_sensor_fusion.processSensorProto(yellow_sensor_proto)
        yellow_world = yellow_sensor_fusion.getWorld()

        blue_sensor_proto = sim_wrapper.get_blue_sensor_proto(ssl_wrapper)
        blue_sensor_fusion.processSensorProto(blue_sensor_proto)
        blue_world = blue_sensor_fusion.getWorld()

        thunderscope.world_layer.cached_world = py.createWorld(yellow_world)

        yellow_primitives = attacker_tactic_stepper.getPrimitives(yellow_world, 0)
        blue_primitives = goalie_tactic_stepper.getPrimitives(blue_world, 0)

        sim_wrapper.send_yellow_primitive_set_and_vision(
            py.createVision(yellow_world), yellow_primitives
        )

        sim_wrapper.send_blue_primitive_set_and_vision(
            py.createVision(blue_world), blue_primitives
        )

    thunderscope.schedule_something(5, run_lol)
    thunderscope.show()


# if __name__ == "__main__":
    # main()

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

def validation_sequence():

    val1 = Validation1()
    val2 = Validation2()

    while not val1.done:
        yield val1
    while not val2.done:
        yield val2

    return


@pytest.fixture()
def simulator():
    print("simulator test fixture")
    print("creating simulator")
    er_force_sim = None
    yield er_force_sim
    print("deleting simulator")
    del er_force_sim

def full_system():
    print("simulator test fixture")
    print("creating full_system")
    full_system = None
    yield full_system
    print("deleting full_system")
    del full_system


class AlwaysValidation(object):

    """Docstring for AlwaysValidation. """

    def validate(self, world):
        assert True

        
class EventuallyValidation(object):

    """Docstring for EventuallyValidation. """

    def __init__(self):
        EventuallyValidation.__init__(self)

def run_tactic_test(simulator, full_system,
             always_validation : List[AlwaysValidation],
             eventually_validation : List[EventuallyValidation]):

    # step sim + sensor_fusion here
    new_world = World()

    while True:
        for validation in always_validation:
            validation.validate(new_world)

def test_attacker_tactic_keepaway(simulator):
    """TODO: Docstring for test_attacker_tactic_keepaway.

    :param function: TODO
    :returns: TODO

    """
    # simulator = StandaloneSimulatorWrapper()
    # simulator.setup_blue_robots([(1, 1)])
    # simulator.setup_yellow_robots([(2, 2)])
    # simulator.setup_ball(ball_position=(1, -2))

    run_tactic_test(None, None, [AlwaysValidation()], [])


def test_attacker_tactic_shoots_on_net():
    """TODO: Docstring for test_attacker_tactic_shoots_on_net.

    :param function: TODO
    :returns: TODO

    """
    pass


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-svv"]))
