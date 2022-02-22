import software.simulated_tests.python_bindings as py
from proto.sensor_msg_pb2 import SensorProto
from proto.world_pb2 import WorldState, SimulatorTick
from proto.vision_pb2 import RobotState, BallState
import threading
from pyqtgraph.Qt import QtCore, QtGui
from proto.geometry_pb2 import Point, Angle, Vector, AngularVelocity
from proto.robot_status_msg_pb2 import RobotStatus
from proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
from software.thunderscope.thunderscope import Thunderscope
from software.simulated_tests.standalone_simulator_interface import (
    StandaloneErForceSimulator,
)
import time
import pytest


@pytest.fixture()
def thunderscope():
    print("setup")
    yield "resource"
    print("teardown")


@pytest.fixture()
def simulator():
    print("setup")
    yield "resource"
    print("teardown")


@pytest.fixture()
def tactic_stepper():
    print("setup")
    yield "resource"
    print("teardown")


def main():

    # setup simulator
    sim_interface = StandaloneErForceSimulator()
    sim_interface.setup_blue_robots([(1, 1)])
    sim_interface.setup_yellow_robots([(2, 2)])
    sim_interface.setup_ball(ball_position=(1, -2))

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

    def run_lol():
        sim_interface.tick(5)
        ssl_wrapper = sim_interface.get_ssl_wrapper_packet()
        if ssl_wrapper is None:
            return

        yellow_sensor_proto = sim_interface.get_yellow_sensor_proto(ssl_wrapper)
        yellow_sensor_fusion.processSensorProto(yellow_sensor_proto)
        yellow_world = yellow_sensor_fusion.getWorld()

        blue_sensor_proto = sim_interface.get_blue_sensor_proto(ssl_wrapper)
        blue_sensor_fusion.processSensorProto(blue_sensor_proto)
        blue_world = blue_sensor_fusion.getWorld()

        thunderscope.world_layer.cached_world = py.createWorld(yellow_world)

        yellow_primitives = attacker_tactic_stepper.getPrimitives(yellow_world, 0)
        blue_primitives = goalie_tactic_stepper.getPrimitives(blue_world, 0)

        sim_interface.send_yellow_primitive_set_and_vision(
            py.createVision(yellow_world), yellow_primitives
        )

        sim_interface.send_blue_primitive_set_and_vision(
            py.createVision(blue_world), blue_primitives
        )

    thunderscope = Thunderscope()
    thunderscope.schedule_something(5, run_lol)
    thunderscope.show()


if __name__ == "__main__":
    main()
