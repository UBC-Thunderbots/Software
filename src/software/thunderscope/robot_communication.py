from software.py_constants import *
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.python_bindings import *
from proto.import_all_protos import *
from proto.visualization_pb2 import HRVOVisualization
import threading
import time

from software.logger.logger import createLogger

logger = createLogger(__name__)

# todo remove
IGNORE_ESTOP = True


class RobotCommunication(object):

    def process_data(self, data):
        viz = HRVOVisualization(robot_id=4)
        str = viz.ParseFromString(b"CjB0eXBlLmdvb2dsZWFwaXMuY29tL1Rib3RzUHJvdG8uSFJWT1Zpc3VhbGl6YXRpb24SXD")
        print(viz.robot_id)
        print(data.velocity_obstacles)
        self.full_system_proto_unix_io.send_proto(HRVOVisualization, data)

    """ Communicate with the robots """

    def __init__(
            self,
            full_system_proto_unix_io,
            multicast_channel,
            interface,
            estop_path="/dev/ttyACM0",
            estop_buadrate=115200,
    ):
        """Initialize the communication with the robots

        :param full_system_proto_unix_io: full_system_proto_unix_io object
        :param multicast_channel: The multicast channel to use
        :param interface: The interface to use
        :param estop_path: The path to the estop
        :param estop_baudrate: The baudrate of the estop

        """
        self.sequence_number = 0
        self.last_time = time.time()
        self.full_system_proto_unix_io = full_system_proto_unix_io
        self.multicast_channel = str(multicast_channel)
        self.interface = interface
        self.estop_path = estop_path
        self.estop_buadrate = estop_buadrate

        self.robots_connected_to_handheld_controllers = set()
        self.robots_connected_to_diagnostics = set()

        self.world_buffer = ThreadSafeBuffer(1, World)
        self.primitive_buffer = ThreadSafeBuffer(1, PrimitiveSet)

        self.motor_control_diagnostics_buffer = ThreadSafeBuffer(1, MotorControl)
        self.power_control_diagnostics_buffer = ThreadSafeBuffer(1, PowerControl)

        self.full_system_proto_unix_io.register_observer(World, self.world_buffer)
        self.full_system_proto_unix_io.register_observer(
            PrimitiveSet, self.primitive_buffer
        )
        self.full_system_proto_unix_io.register_observer(
            MotorControl, self.motor_control_diagnostics_buffer
        )
        self.full_system_proto_unix_io.register_observer(
            PowerControl, self.power_control_diagnostics_buffer
        )

        self.send_estop_state_thread = threading.Thread(target=self.__send_estop_state)
        self.run_thread = threading.Thread(target=self.run)

        try:
            self.estop_reader = ThreadedEstopReader(
                self.estop_path, self.estop_buadrate
            )
        except Exception:
            pass

    def __send_estop_state(self):
        while True:
            # self.full_system_proto_unix_io.send_proto(
            #    EstopState, EstopState(is_playing=self.estop_reader.isEstopPlay())
            # )
            time.sleep(0.1)

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
            if self.fullsystem_connected_to_robots:

                # Send the world
                world = self.world_buffer.get(block=True)
                self.world_mcast_sender.send_proto(world)
                # Send the primitive set
                primitive_set = self.primitive_buffer.get(block=False)

                if IGNORE_ESTOP or self.estop_reader.isEstopPlay():
                    # primitive_set.time_sent = Timestamp(epoch_timestamp_seconds=time.time())
                    print(primitive_set)
                    self.send_primitive_set.send_proto(primitive_set)
                    # logger.info(primitive_set)

            else:

                diagnostics_primitive = DirectControlPrimitive(
                    motor_control=self.motor_control_diagnostics_buffer.get(
                        block=False
                    ),
                    power_control=self.power_control_diagnostics_buffer.get(
                        block=False
                    ),
                )

                primitive_set = PrimitiveSet(
                    time_sent=Timestamp(epoch_timestamp_seconds=time.time()),
                    stay_away_from_ball=False,
                    robot_primitives={
                        robot_id: Primitive(direct_control=diagnostics_primitive)
                        for robot_id in self.robots_connected_to_diagnostics
                    },
                    sequence_number=self.sequence_number,
                )

                self.sequence_number += 1

                if self.estop_reader.isEstopPlay():
                    self.last_time = primitive_set.time_sent.epoch_timestamp_seconds
                    self.send_primitive_set.send_proto(primitive_set)

                time.sleep(0.001)

    def connect_fullsystem_to_robots(self):
        """ Connect the robots to fullsystem """

        self.fullsystem_connected_to_robots = True
        self.robots_connected_to_handheld_controllers = set()
        self.robots_connected_to_diagnostics = set()

    def disconnect_fullsystem_from_robots(self):
        """ Disconnect the robots from fullsystem """

        self.fullsystem_connected_to_robots = False

    def connect_robot_to_diagnostics(self, robot_id):
        self.robots_connected_to_diagnostics.add(robot_id)

    def discconnect_robot_from_diagnostics(self, robot_id):
        self.robots_connected_to_diagnostics.remove(robot_id)

    def __enter__(self):
        """Enter RobotCommunication context manager. Setup multicast listener
        for RobotStatus and multicast senders for World and PrimitiveSet

        """
        # Create the multicast listeners
        print("channel", (self.multicast_channel))
        print(self.interface)
        print(ROBOT_STATUS_PORT)
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
            lambda data:  # print("received robot log"),
            self.full_system_proto_unix_io.send_proto(RobotLog, data),
            True,
        )

        self.receive_hrvo_vis = HRVOVisualizationProtoListener(
            self.multicast_channel + "%" + self.interface,
            SERIALIZED_PROTO_LOGS_PORT,
            lambda data: self.process_data(data),
            # print("received proto log " + viz.ParseFromString(data.SerializeToString())),
            # print("received proto log " + data),
            # self.full_system_proto_unix_io.send_proto(
            #     HRVOVisualization, data
            # ),
            True,
        )
        print(
            SSL_VISION_ADDRESS, SSL_VISION_PORT, SSL_REFEREE_ADDRESS, SSL_REFEREE_PORT
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

        # self.disconnect_fullsystem_from_robots()
        # self.connect_robot_to_diagnostics(0)
        # self.connect_robot_to_diagnostics(1)
        # self.connect_robot_to_diagnostics(1)
        # self.connect_robot_to_diagnostics(3)
        # self.connect_robot_to_diagnostics(4)
        # self.connect_robot_to_diagnostics(5)
        # self.connect_robot_to_diagnostics(6)
        # self.connect_robot_to_diagnostics(7)
        self.connect_fullsystem_to_robots()

        self.send_estop_state_thread.start()
        self.run_thread.start()

    def __exit__(self):
        """Exit RobotCommunication context manager

        """
        self.run_thread.join()
