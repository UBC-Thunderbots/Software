from software.py_constants import *
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.python_bindings import *
from proto.import_all_protos import *
import threading
import time


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

        :param proto_unix_io: The input proto_unix_io object
        :param multicast_channel: The multicast channel to use
        :param interface: The interface to use
        :param estop_path: The path to the estop
        :param estop_baudrate: The baudrate of the estop

        """
        print('init')
        self.sequence_number = 0
        self.last_time = time.time()
        self.proto_unix_io = proto_unix_io
        self.multicast_channel = str(multicast_channel)
        self.interface = interface
        self.estop_path = estop_path
        self.estop_buadrate = estop_buadrate

        self.robots_connected_to_diagnostics = set()

        self.world_buffer = ThreadSafeBuffer(1, World)
        self.primitive_buffer = ThreadSafeBuffer(1, PrimitiveSet)

        self.motor_control_diagnostics_buffer = ThreadSafeBuffer(1, MotorControl)
        self.power_control_diagnostics_buffer = ThreadSafeBuffer(1, PowerControl)

        self.proto_unix_io.register_observer(World, self.world_buffer)
        self.proto_unix_io.register_observer(PrimitiveSet, self.primitive_buffer)
        self.proto_unix_io.register_observer(
            MotorControl, self.motor_control_diagnostics_buffer
        )
        self.proto_unix_io.register_observer(
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
        #     self.proto_unix_io.send_proto(
        #         EstopState, EstopState(is_playing=self.estop_reader.isEstopPlay())
        #     )
        #     time.sleep(0.1)

    def run(self):
        """Forward World and PrimitiveSet protos from fullsystem and diagnostics to the robots.

        If the emergency stop is tripped, the PrimitiveSet will not be sent so
        that the robots timeout and stop.

        Robots can be connected to and disconnected from diagnostics and primitive sets can be
        sent that way.

        """
        while True:
            # Initialise empty total primitives
            robot_primitives = {}

            # # Get the world
            # world = self.world_buffer.get(block=False)
            #
            # # try sending the world proto
            # try:
            #     self.world_mcast_sender.send_proto(world)
            #
            #     # if world is valid, get fullsystem primitives and add them to total primitives
            #     fullsystem_primitive_set = self.primitive_buffer.get(block=False)
            #
            #     robot_primitives = fullsystem_primitive_set.robot_primitives
            #
            #     # filter fullsystem primitives to remove ones for robots connected to diagnostics
            #     for robot_id in self.robots_connected_to_diagnostics:
            #         del robot_primitives[robot_id]
            # except RuntimeError:
            #     # world is not valid, which means no vision / simulator data is being received
            #     pass

            # get the diagnostics primitive
            diagnostics_primitive = DirectControlPrimitive(
                motor_control=self.motor_control_diagnostics_buffer.get(
                    block=False
                ),
                power_control=self.power_control_diagnostics_buffer.get(
                    block=False
                ),
            )

            print(diagnostics_primitive)

            # for robots connected to diagnostics, add the diagnostics primitive to total primitives
            for robot_id in self.robots_connected_to_diagnostics:
                robot_primitives[robot_id] = Primitive(direct_control=diagnostics_primitive)

            # initialize total primitive set
            robot_primitive_set = PrimitiveSet(
                time_sent=Timestamp(epoch_timestamp_seconds=time.time()),
                stay_away_from_ball=False,
                robot_primitives=robot_primitives,
                sequence_number=self.sequence_number,
            )

            print(robot_primitives)

            self.sequence_number += 1

            # send the total primitive set
            if True:
                self.last_time = (
                    robot_primitive_set.time_sent.epoch_timestamp_seconds
                )
                self.send_primitive_set.send_proto(robot_primitive_set)

            time.sleep(0.001)

    def connect_fullsystem_to_robots(self):
        """ Connect the robots to fullsystem """

        self.fullsystem_connected_to_robots = True
        self.robots_connected_to_diagnostics = set()

    def disconnect_fullsystem_from_robots(self):
        """ Disconnect the robots from fullsystem """

        self.fullsystem_connected_to_robots = False

    def toggle_robot_connection(self, robot_id):
        """
        Connects a robot to or disconnects a robot from diagnostics
        :param robot_id: the id of the robot to be added or removed from the diagnostics set
        """
        print(robot_id)
        if robot_id in self.robots_connected_to_diagnostics:
            self.robots_connected_to_diagnostics.remove(robot_id)
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
            lambda data: self.full_system_proto_unix_io.send_proto(RobotStatus, data),
            True,
        )

        self.send_primitive_mcast_sender = PrimitiveSetProtoSender(
            self.multicast_channel + "%" + self.interface, PRIMITIVE_PORT, True
        )

        self.receive_robot_log = RobotLogProtoListener(
            self.multicast_channel + "%" + self.interface,
            ROBOT_LOGS_PORT,
            lambda data: self.full_system_proto_unix_io.send_proto(RobotLog, data),
            True,
        )

        self.receive_ssl_wrapper = SSLWrapperPacketProtoListener(
            SSL_VISION_ADDRESS,
            SSL_VISION_PORT,
            lambda data: self.full_system_proto_unix_io.send_proto(
                SSL_WrapperPacket, data
            ),
            True,
        )

        self.receive_ssl_referee_proto = SSLRefereeProtoListener(
            SSL_REFEREE_ADDRESS,
            SSL_REFEREE_PORT,
            lambda data: self.full_system_proto_unix_io.send_proto(Referee, data),
            True,
        )

        # Create multicast senders
        self.send_primitive_set = PrimitiveSetProtoSender(
            self.multicast_channel + "%" + self.interface, PRIMITIVE_PORT, True
        )

        self.world_mcast_sender = WorldProtoSender(
            self.multicast_channel + "%" + self.interface, VISION_PORT, True
        )

        self.connect_fullsystem_to_robots()

        # TODO (#2741): we might not want to support robot diagnostics in tscope
        # make a ticket here to create a widget to call these functions to detach
        # from AI and connect to robots/or remove
        # self.disconnect_fullsystem_from_robots()

        self.send_estop_state_thread.start()
        self.run_thread.start()

    def __exit__(self):
        """Exit RobotCommunication context manager

        """
        self.run_thread.join()
