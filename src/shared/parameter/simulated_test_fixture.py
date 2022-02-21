import shared.parameter.python_bindings as py
from proto.sensor_msg_pb2 import SensorProto
from proto.world_pb2 import WorldState, SimulatorTick
from proto.vision_pb2 import RobotState, BallState
import threading
from pyqtgraph.Qt import QtCore, QtGui
from proto.geometry_pb2 import Point, Angle, Vector, AngularVelocity
from proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
from software.thunderscope.thunderscope import Thunderscope
from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.networking.threaded_unix_sender import ThreadedUnixSender
import time


def main():
    sensor_proto = SensorProto()
    sensor_fusion_config = py.SensorFusionConfig()
    sensor_fusion = py.SensorFusion(sensor_fusion_config)
    sensor_fusion.processSensorProto(sensor_proto)
    world = sensor_fusion.getWorld()

    attacker_tactic = py.AttackerTactic(py.AttackerTacticConfig())

    tactic_stepper = py.TacticStepper(
        attacker_tactic,
        set([py.MotionConstraint.FRIENDLY_DEFENSE_AREA]),
        py.ThunderbotsConfig(),
    )

    world_state_sender = ThreadedUnixSender("/tmp/tbots/world_state")

    blue_vision = ThreadedUnixSender("/tmp/tbots/blue_vision")
    yellow_vision = ThreadedUnixSender("/tmp/tbots/yellow_vision")

    blue_primitive_set = ThreadedUnixSender("/tmp/tbots/blue_primitive_set")
    yellow_primitive_set = ThreadedUnixSender("/tmp/tbots/yellow_primitive_set")

    world_state_init = ThreadedUnixSender("/tmp/tbots/world_state_init")
    sim_tick_sender = ThreadedUnixSender("/tmp/tbots/simulation_tick")

    listener = ThreadedUnixListener(
        "/tmp/tbots/ssl_wrapper_packet", SSL_WrapperPacket, convert_from_any=False
    )

    tick = SimulatorTick()
    tick.milliseconds = 16

    world_state = WorldState()

    world_state.blue_robots[0].CopyFrom(
        RobotState(
            global_position=Point(x_meters=1, y_meters=1),
            global_orientation=Angle(radians=1),
            global_velocity=Vector(x_component_meters=1, y_component_meters=1),
            global_angular_velocity=AngularVelocity(radians_per_second=1),
        )
    )

    world_state.yellow_robots[0].CopyFrom(
        RobotState(
            global_position=Point(x_meters=1, y_meters=1),
            global_orientation=Angle(radians=1),
            global_velocity=Vector(x_component_meters=1, y_component_meters=1),
            global_angular_velocity=AngularVelocity(radians_per_second=1),
        )
    )

    world_state.ball_state.CopyFrom(
        BallState(
            global_position=Point(x_meters=-2, y_meters=-2),
            global_velocity=Vector(x_component_meters=-2, y_component_meters=-2),
            distance_from_ground=0,
        )
    )

    world_state_sender.send(world_state)

    def run_lol():
        start_time = time.time()
        sim_tick_sender.send(tick)
        time.sleep(0.016)
        wrapper = listener.maybe_pop()
        sense = SensorProto()
        sense.ssl_vision_msg.CopyFrom(wrapper)
        sensor_fusion.processSensorProto(sense)
        world = sensor_fusion.getWorld()
        thunderscope.world_layer.cached_world = py.createWorld(world)
        vision = py.createVision(world)
        primitives = tactic_stepper.getPrimitives(world, 0)
        yellow_vision.send(vision)
        yellow_primitive_set.send(primitives)

    thunderscope = Thunderscope()
    thunderscope.schedule_something(16, run_lol)
    thunderscope.show()


if __name__ == "__main__":
    main()
