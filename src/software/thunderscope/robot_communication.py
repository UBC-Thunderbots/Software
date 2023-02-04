from software.py_constants import *
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.python_bindings import *
from proto.import_all_protos import *
from enum import Enum
import threading
import time


class RobotCommunicationMode(Enum):
    """
    Enum for the 3 different modes robot communications can run in + 1 default mode as a null value
    """

    FULLSYSTEM = 1
    DIAGNOSTICS = 2
    BOTH = 3
    NONE = 4


class RobotCommunication(object):

    """ Communicate with the robots """

    def __init__(
        self,
        current_proto_unix_io,
        current_mode,
        multicast_channel,
        interface,
        estop_path="/dev/ttyACM0",
        estop_buadrate=115200,
    ):
        """Initialize the communication with the robots

        :param current_proto_unix_io: the current proto unix io object
        :param current_mode: one of RobotCommunicationMode, indicates which mode is currently running
        :param multicast_channel: The multicast channel to use
        :param interface: The interface to use
        :param estop_path: The path to the estop
        :param estop_baudrate: The baudrate of the estop

        """
        self.sequence_number = 0
        self.last_time = time.time()
        self.current_proto_unix_io = current_proto_unix_io
        self.multicast_channel = str(multicast_channel)
        self.interface = interface
        self.estop_path = estop_path
        self.estop_buadrate = estop_buadrate
        self.current_mode = current_mode

        self.world_buffer = ThreadSafeBuffer(1, World)
        self.primitive_buffer = ThreadSafeBuffer(1, PrimitiveSet)

        self.motor_control_diagnostics_buffer = ThreadSafeBuffer(1, MotorControl)
        self.power_control_diagnostics_buffer = ThreadSafeBuffer(1, PowerControl)

        self.current_proto_unix_io.register_observer(World, self.world_buffer)

        # if fullsystem is running, all robots are connected to it
        if (
            current_mode == RobotCommunicationMode.BOTH
            or current_mode == RobotCommunicationMode.FULLSYSTEM
        ):
            self.robots_connected_to_fullsystem = {
                robot_id for robot_id in range(MAX_ROBOT_IDS_PER_SIDE)
            }

        # if diagnostics is running, no robots are connected to it initially (empty set)
        if current_mode != RobotCommunicationMode.FULLSYSTEM:
            self.robots_connected_to_diagnostics = set()

        self.robots_to_be_disconnected = set()

        self.current_proto_unix_io.register_observer(
            PrimitiveSet, self.primitive_buffer
        )

        self.current_proto_unix_io.register_observer(
            MotorControl, self.motor_control_diagnostics_buffer
        )
        self.current_proto_unix_io.register_observer(
            PowerControl, self.power_control_diagnostics_buffer
        )

        self.send_estop_state_thread = threading.Thread(target=self.__send_estop_state)
        self.run_thread = threading.Thread(target=self.run)

        # try:
        #     self.estop_reader = ThreadedEstopReader(
        #         self.estop_path, self.estop_buadrate
        #     )
        # except Exception:
        #     raise Exception("Could not find estop, make sure its plugged in")

    def __send_estop_state(self):
        print('yea')
        # while True:
        #     self.current_proto_unix_io.send_proto(
        #         EstopState, EstopState(is_playing=self.estop_reader.isEstopPlay())
        #     )
        #     time.sleep(0.1)

    def run(self):
        """Forward World and PrimitiveSet protos from fullsystem to the robots.

        If the emergency stop is tripped, the PrimitiveSet will not be sent so
        that the robots timeout and stop.

        NOTE: If disconnect_fullsystem_from_robots is called, then the packets
        will not be forwarded to the robots.

        send_override_primitive_set can be used to send a primitive set, which
        is useful to dip in and out of robot diagnostics.

        """
        while True:
            # total primitives for all robots
            robot_primitives = dict()

            # fullsystem is running, so world data is being received
            if (
                self.current_mode == RobotCommunicationMode.FULLSYSTEM
                or self.current_mode == RobotCommunicationMode.BOTH
            ):
                # Get the world
                world = self.world_buffer.get(block=True)

                # send the world proto
                self.send_world.send_proto(world)

                # Get the primitives
                primitive_set = self.primitive_buffer.get(block=False)

                # Filter the primitives to only include robots not connected to diagnostics
                for robot_id in self.robots_connected_to_diagnostics:
                    del primitive_set[robot_id]

                robot_primitives = primitive_set

            # diagnostics is running
            if (
                self.current_mode == RobotCommunicationMode.DIAGNOSTICS
                or self.current_mode == RobotCommunicationMode.BOTH
            ):

                # get the manual control primitive
                diagnostics_primitive = DirectControlPrimitive(
                    motor_control=self.motor_control_diagnostics_buffer.get(
                        block=False
                    ),
                    power_control=self.power_control_diagnostics_buffer.get(
                        block=False
                    ),
                )

                # for all robots connected to diagnostics, set their primitive
                for robot_id in self.robots_connected_to_diagnostics:
                    robot_primitives[robot_id] = Primitive(
                        direct_control=diagnostics_primitive
                    )

            # sends a final stop primitive to all disconnected robots and removes them from list
            # in order to prevent robots acting on cached old primitives
            while self.robots_to_be_disconnected:
                robot_primitives[self.robots_to_be_disconnected.pop()] = Primitive(
                    stop=StopPrimitive()
                )

            # initialize total primitive set and send it
            primitive_set = PrimitiveSet(
                time_sent=Timestamp(epoch_timestamp_seconds=time.time()),
                stay_away_from_ball=False,
                robot_primitives=robot_primitives,
                sequence_number=self.sequence_number,
            )

            self.sequence_number += 1

            if self.estop_reader.isEstopPlay():
                self.last_time = primitive_set.time_sent.epoch_timestamp_seconds
                self.send_primitive_set.send_proto(primitive_set)

            time.sleep(0.01)

    def toggle_robot_connection(self, robot_id):
        """
        Connects a robot to or disconnects a robot from diagnostics
        :param robot_id: the id of the robot to be added or removed from the diagnostics set
        """
        if robot_id in self.robots_connected_to_diagnostics:
            self.robots_connected_to_diagnostics.remove(robot_id)
            self.robots_to_be_disconnected.add(robot_id)
        else:
            self.robots_connected_to_diagnostics.add(robot_id)

    def __enter__(self):
        """Enter RobotCommunication context manager. Setup multicast listener
        for RobotStatus and multicast senders for World and PrimitiveSet

        """
        # Create the multicast listeners
        self.receive_robot_status = RobotStatusProtoListener(
            self.multicast_channel + "%" + self.interface,
            ROBOT_STATUS_PORT,
            lambda data: self.current_proto_unix_io.send_proto(RobotStatus, data),
            True,
        )

        self.receive_robot_log = RobotLogProtoListener(
            self.multicast_channel + "%" + self.interface,
            ROBOT_LOGS_PORT,
            lambda data: self.current_proto_unix_io.send_proto(RobotLog, data),
            True,
        )

        if (
            self.current_mode == RobotCommunicationMode.FULLSYSTEM
            or self.current_mode == RobotCommunicationMode.BOTH
        ):
            self.receive_ssl_wrapper = SSLWrapperPacketProtoListener(
                SSL_VISION_ADDRESS,
                SSL_VISION_PORT,
                lambda data: self.current_proto_unix_io.send_proto(
                    SSL_WrapperPacket, data
                ),
                True,
            )

            self.send_world = WorldProtoSender(
                self.multicast_channel + "%" + self.interface, VISION_PORT, True
            )

        # Create multicast senders
        self.send_primitive_set = PrimitiveSetProtoSender(
            self.multicast_channel + "%" + self.interface, PRIMITIVE_PORT, True
        )

        self.send_estop_state_thread.start()
        self.run_thread.start()

        return self

    def __exit__(self):
        """Exit RobotCommunication context manager

        """
        self.run_thread.join()
