from software.py_constants import *
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.python_bindings import *
from proto.import_all_protos import *


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
        self.multicast_channel = str(multicast_channel)
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

        self.run_thread = Thread(target=self.run)

    def run(self):
        """Run the robot communication

        Only sends the primitive set if the estop is not pressed
        Always sends the latest world

        NOTE: This loop runs as fast as we receive worlds and primitive sets.
        We send out the new world and new primitive set

        """

        world = self.world_buffer.get(block=True)
        primitive_set = self.primitive_buffer.get(block=True)

        if self.estop_reader.isEstopPlay():
            self.primitive_set_mcast_sender.send(primitive_set)

        self.world_mcast_sender.send(world)

    def __enter__(self):

        """ Enter RobotCommunication context manager """

        # Create the multicast channels
        self.receive_robot_status = RobotStatusProtoListener(
            self.multicast_channel + "%" + self.interface,
            ROBOT_STATUS_PORT,
            lambda data: self.proto_unix_io.send_proto(RobotStatus, data),
            True,
        )

        self.send_primitive_mcast_sender = PrimitiveSetProtoSender(
            self.multicast_channel + "%" + self.interface, PRIMITIVE_PORT, True
        )

        self.send_world_mcast_sender = WorldProtoSender(
            self.multicast_channel + "%" + self.interface, VISION_PORT, True
        )

    def __exit__(self):

        """ Exit RobotCommunication context manager """

        self.run_thread.join()
