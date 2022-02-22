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
    sim_interface = StandaloneErForceSimulator()

    sim_interface.setup_blue_robots([(1, 1)])
    sim_interface.setup_yellow_robots([(2, 2)])
    sim_interface.setup_ball()

    attacker_tactic = py.AttackerTactic(py.AttackerTacticConfig())

    tactic_stepper = py.TacticStepper(
        attacker_tactic,
        set([py.MotionConstraint.FRIENDLY_DEFENSE_AREA]),
        py.ThunderbotsConfig(),
    )

    sensor_fusion_config = py.SensorFusionConfig()
    sensor_fusion = py.SensorFusion(sensor_fusion_config)

    def run_lol():
        sim_interface.tick(15)
        ssl_wrapper = sim_interface.get_ssl_wrapper_packet()
        if ssl_wrapper is None:
            return
        yellow_sensor_proto = sim_interface.get_yellow_sensor_proto(ssl_wrapper)
        sensor_fusion.processSensorProto(yellow_sensor_proto)
        world = sensor_fusion.getWorld()
        thunderscope.world_layer.cached_world = py.createWorld(world)
        primitives = tactic_stepper.getPrimitives(world, 0)
        sim_interface.send_yellow_primitive_set_and_vision(
            py.createVision(world), primitives
        )

    thunderscope = Thunderscope()
    thunderscope.schedule_something(16, run_lol)
    thunderscope.show()


if __name__ == "__main__":
    main()
