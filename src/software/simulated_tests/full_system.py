from subprocess import Popen

from proto.geometry_pb2 import Angle, AngularVelocity, Point, Vector
from proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
from proto.robot_status_msg_pb2 import RobotStatus
from proto.sensor_msg_pb2 import SensorProto
from proto.tbots_software_msgs_pb2 import PrimitiveSet, Vision
from proto.vision_pb2 import BallState, RobotState
from proto.world_pb2 import SimulatorTick, WorldState

from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.networking.threaded_unix_sender import ThreadedUnixSender

VISION_INPUT_PATH = "/vision"
PRIMITIVE_INPUT_PATH = "/primitive"

ROBOT_STATUS_OUTPUT_PATH = "/robot_status"
SSL_WRAPPER_OUTPUT_PATH = "/ssl_wrapper"
SSL_REFEREE_OUTPUT_PATH = "/ssl_referee"
SENSOR_PROTO_OUTPUT_PATH = "/sensor_proto"

TACTIC_OVERRIDE_PATH = "/tactic_override"


class FullSystem(object):
    def __init__(self, base_unix_path="/tmp/tbots"):

        """Runs our standalone er-force simulator binary and sets up the unix
        sockets to communicate with it

        """
        # TODO I don't think we need these
        self.robot_status_sender = ThreadedUnixSender(
            base_unix_path + ROBOT_STATUS_OUTPUT_PATH
        )
        self.ssl_wrapper_sender = ThreadedUnixSender(
            base_unix_path + SSL_WRAPPER_OUTPUT_PATH
        )
        self.ssl_referee_sender = ThreadedUnixSender(
            base_unix_path + SSL_REFEREE_OUTPUT_PATH
        )
        self.tactic_override = ThreadedUnixSender(base_unix_path + TACTIC_OVERRIDE_PATH)

        self.sensor_proto_sender = ThreadedUnixSender(
            base_unix_path + SENSOR_PROTO_OUTPUT_PATH
        )

        self.vision_listener = ThreadedUnixListener(
            base_unix_path + VISION_INPUT_PATH, Vision, convert_from_any=False,
        )
        self.primitive_listener = ThreadedUnixListener(
            base_unix_path + PRIMITIVE_INPUT_PATH, PrimitiveSet, convert_from_any=False,
        )

        self.full_system_process = Popen(["software/full_system"])

    def send_robot_status(self, robot_status):
        self.robot_status_sender.send(robot_status)

    def send_ssl_wrapper(self, ssl_wrapper_packet):
        self.ssl_wrapper_sender.send(ssl_wrapper_packet)

    def send_ssl_referee(self, ssl_referee_packet):
        self.ssl_referee_sender.send(ssl_referee_packet)

    def send_sensor_proto(self, sensor_proto):
        self.sensor_proto_sender.send(sensor_proto)

    def get_vision(self):
        return self.vision_listener.maybe_pop()

    def get_primitive_set(self):
        return self.primitive_listener.maybe_pop()

    def send_tactic_override(self, assigned_tactic_play_control_params):
        return self.tactic_override.send(assigned_tactic_play_control_params)

    def stop():
        self.robot_status_sender.force_stop()
        self.ssl_wrapper_sender.force_stop()
        self.ssl_referee_sender.force_stop()
        self.tactic_override.force_stop()
        self.sensor_proto_sender.force_stop()
        self.vision_listener.force_stop()
        self.primitive_listener.force_stop()
