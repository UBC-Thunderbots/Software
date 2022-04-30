from software.py_constants import *
from proto.import_all_protos import *
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.python_bindings import *


class RobotCommunication(object):

    """ Communicate with the robots """

    def __init__(
        self,
        proto_unix_io,
        multicast_channel,
        interface,
        estop_path="/dev/ttyACM0",
        estop_buadrate=115200,
    ):
        """Initialize the communication with the robots

        :param proto_unix_io: proto_unix_io object
        :param multicast_channel: The multicast channel to use
        :param interface: The interface to use
        :param estop_path: The path to the estop
        :param estop_baudrate: The baudrate of the estop

        """
        self.proto_unix_io = proto_unix_io
        self.multicast_channel = multicast_channel
        self.interface = interface
        self.estop_path = estop_path
        self.estop_buadrate = estop_buadrate

        self.world_buffer = ThreadSafeBuffer(1, World)
        self.primitive_buffer = ThreadSafeBuffer(1, PrimitiveSet)

        self.proto_unix_io.register_observer(World, self.world_buffer)
        self.proto_unix_io.register_observer(PrimitiveSet, self.primitive_buffer)

        try:
            self.estop_reader = ThreadedEstopReader(
                self.estop_path, self.estop_buadrate
            )
        except Exception:
            raise Exception("Could not find estop, make sure its plugged in")

    def __enter__(self):

        """ Enter RobotCommunication context manager """

        # Create the multicast channels
        self.receive_robot_status = networking.RobotStatusProtoListener(
            self.multicast_channel + "%" + self.interface,
            ROBOT_STATUS_PORT,
            lambda data: self.proto_unix_io.send_proto(RobotStatus, data),
            True,
        )

        self.send_primitive_set = networking.PrimitiveSetProtoSender(
            self.multicast_channel + "%" + self.interface, PRIMITIVE_PORT, True
        )

        self.send_world = networking.WorldProtoSender(
            self.multicast_channel + "%" + self.interface, VISION_PORT, True
        )

    def __exit__(self):
        """ Exit RobotCommunication context manager """
