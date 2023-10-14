from software.py_constants import *
from software.thunderscope.constants import ROBOT_COMMUNICATIONS_TIMEOUT_S
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.constants import IndividualRobotMode
from software.python_bindings import *
from queue import Empty
from proto.import_all_protos import *
from pyqtgraph.Qt import QtCore
from software.thunderscope.proto_unix_io import ProtoUnixIO
import threading
import time
import os
from typing import Any
from enum import Enum
from google.protobuf.message import Message


class RobotCommunication(object):

    """ Communicate with the robots """

    def __init__(
        self,
        current_proto_unix_io: ProtoUnixIO,
        multicast_channel: str,
        interface: str,
        disable_estop: bool,
        estop_path: os.PathLike,
        estop_buadrate: int = 115200,
    ) -> None:
        """Initialize the communication with the robots

        :param current_proto_unix_io: the current proto unix io object
        :param multicast_channel: The multicast channel to use
        :param interface: The interface to use
        :param disable_estop: whether to disable estop checks (ONLY FOR TESTING LOCALLY)
        :param estop_path: The path to the estop
        :param estop_baudrate: The baudrate of the estop

        """
        self.sequence_number = 0
        self.last_time = time.time()
        self.current_proto_unix_io = current_proto_unix_io
        self.multicast_channel = str(multicast_channel)
        self.interface = interface
        self.disable_estop = disable_estop
        self.estop_path = estop_path
        self.estop_buadrate = estop_buadrate

        self.running = False

        self.world_buffer = ThreadSafeBuffer(1, World)
        self.primitive_buffer = ThreadSafeBuffer(1, PrimitiveSet)

        self.motor_control_diagnostics_buffer = ThreadSafeBuffer(1, MotorControl)
        self.power_control_diagnostics_buffer = ThreadSafeBuffer(1, PowerControl)

        self.current_proto_unix_io.register_observer(World, self.world_buffer)

        self.robots_connected_to_fullsystem = set()
        self.robots_connected_to_manual = set()
        self.robots_to_be_disconnected = {}

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
        self.run_world_thread = threading.Thread(target=self.run_world, daemon=True)
        self.run_primitive_set_thread = threading.Thread(
            target=self.run_primitive_set, daemon=True
        )

        # only checks for estop if checking is not disabled
        if not self.disable_estop:
            try:
                self.estop_reader = ThreadedEstopReader(
                    self.estop_path, self.estop_buadrate
                )
            except Exception:
                raise Exception("Could not find estop, make sure its plugged in")

    def __send_estop_state(self) -> None:
        if not self.disable_estop:
            while self.running:
                self.current_proto_unix_io.send_proto(
                    EstopState, EstopState(is_playing=self.estop_reader.isEstopPlay())
                )
                time.sleep(0.1)

    def run_world(self) -> None:
        """
        Forward World protos from fullsystem to the robots
        Blocks if no world is available and does not return a cached world
        :return:
        """
        while self.running:
            world = None
            try:
                world = self.world_buffer.get(
                    block=True,
                    timeout=ROBOT_COMMUNICATIONS_TIMEOUT_S,
                    return_cached=False,
                )
            except Empty:
                # if empty do nothing
                pass
            if (
                world
                and not self.disable_estop
                and self.estop_reader.isEstopPlay()
                and (
                    self.robots_connected_to_fullsystem
                    or self.robots_connected_to_manual
                )
            ):
                # send the world proto
                self.send_world.send_proto(world)

    def run_primitive_set(self) -> None:
        """Forward PrimitiveSet protos from fullsystem to the robots.

        For AI protos, blocks for 10ms if no proto is available, and then returns a cached proto

        For Diagnostics protos, does not block and returns cached message if none available
        Sleeps for 10ms for diagnostics

        If the emergency stop is tripped, the PrimitiveSet will not be sent so
        that the robots timeout and stop.

        NOTE: If disconnect_fullsystem_from_robots is called, then the packets
        will not be forwarded to the robots.

        send_override_primitive_set can be used to send a primitive set, which
        is useful to dip in and out of robot diagnostics.

        """
        while self.running:
            # total primitives for all robots
            robot_primitives = {}

            # fullsystem is running, so world data is being received
            if self.robots_connected_to_fullsystem:
                # Get the primitives
                primitive_set = self.primitive_buffer.get(
                    block=True, timeout=ROBOT_COMMUNICATIONS_TIMEOUT_S
                )

                fullsystem_primitives = dict(primitive_set.robot_primitives)
                for robot_id in fullsystem_primitives.keys():
                    if robot_id in self.robots_connected_to_fullsystem:
                        robot_primitives[robot_id] = fullsystem_primitives[robot_id]

            # get the manual control primitive
            diagnostics_primitive = DirectControlPrimitive(
                motor_control=self.motor_control_diagnostics_buffer.get(block=False),
                power_control=self.power_control_diagnostics_buffer.get(block=False),
            )

            # diagnostics is running
            if self.robots_connected_to_manual:
                # for all robots connected to diagnostics, set their primitive
                for robot_id in self.robots_connected_to_manual:
                    robot_primitives[robot_id] = Primitive(
                        direct_control=diagnostics_primitive
                    )

            # sends a final stop primitive to all disconnected robots and removes them from list
            # in order to prevent robots acting on cached old primitives
            for robot_id, num_times_to_stop in self.robots_to_be_disconnected.items():
                if num_times_to_stop > 0:
                    robot_primitives[robot_id] = Primitive(stop=StopPrimitive())
                    self.robots_to_be_disconnected[robot_id] = num_times_to_stop - 1

            # initialize total primitive set and send it
            primitive_set = PrimitiveSet(
                time_sent=Timestamp(epoch_timestamp_seconds=time.time()),
                stay_away_from_ball=False,
                robot_primitives=robot_primitives,
                sequence_number=self.sequence_number,
            )

            self.sequence_number += 1

            if (
                not self.disable_estop
                and self.estop_reader.isEstopPlay()
                and (
                    self.robots_connected_to_fullsystem
                    or self.robots_connected_to_manual
                )
            ):
                self.send_primitive_set.send_proto(primitive_set)

            # sleep if not running fullsystem
            if not self.robots_connected_to_fullsystem:
                time.sleep(ROBOT_COMMUNICATIONS_TIMEOUT_S)

    def toggle_robot_connection(self, mode: IndividualRobotMode, robot_id: int) -> None:
        """
        Connects a robot to or disconnects a robot from diagnostics

        :param mode: the mode of input for this robot's primitives
        :param robot_id: the id of the robot to be added or removed from the diagnostics set
        """
        self.robots_connected_to_fullsystem.discard(robot_id)
        self.robots_connected_to_manual.discard(robot_id)
        self.robots_to_be_disconnected.pop(robot_id, None)

        if mode == IndividualRobotMode.NONE:
            self.robots_to_be_disconnected[robot_id] = NUM_TIMES_SEND_STOP
        elif mode == IndividualRobotMode.MANUAL:
            self.robots_connected_to_manual.add(robot_id)
        elif mode == IndividualRobotMode.AI:
            self.robots_connected_to_fullsystem.add(robot_id)

    def __forward_to_proto_unix_io(self, type: Message, data: Enum) -> None:
        """
        Forwards to proto unix IO iff running is true
        :param data: the data to be passed through
        :param type: the proto type
        """
        if self.running:
            self.current_proto_unix_io.send_proto(type, data)

    def setup_for_fullsystem(self) -> None:
        """
        Sets up a world sender, a listener for SSL vision data, and connects all robots to fullsystem as default
        """

        self.receive_ssl_wrapper = SSLWrapperPacketProtoListener(
            SSL_VISION_ADDRESS,
            SSL_VISION_PORT,
            lambda data: self.__forward_to_proto_unix_io(SSL_WrapperPacket, data),
            True,
        )

        self.receive_ssl_referee_proto = SSLRefereeProtoListener(
            SSL_REFEREE_ADDRESS,
            SSL_REFEREE_PORT,
            lambda data: self.current_proto_unix_io.send_proto(Referee, data),
            True,
        )

        self.send_world = WorldProtoSender(
            self.multicast_channel + "%" + self.interface, VISION_PORT, True
        )

        self.robots_connected_to_fullsystem = {
            robot_id for robot_id in range(MAX_ROBOT_IDS_PER_SIDE)
        }

        self.run_world_thread.start()

    def __enter__(self) -> "self":
        """Enter RobotCommunication context manager. Setup multicast listener
        for RobotStatus and multicast senders for World and PrimitiveSet

        """
        # Create the multicast listeners
        self.receive_robot_status = RobotStatusProtoListener(
            self.multicast_channel + "%" + self.interface,
            ROBOT_STATUS_PORT,
            lambda data: self.__forward_to_proto_unix_io(RobotStatus, data),
            True,
        )

        self.receive_robot_log = RobotLogProtoListener(
            self.multicast_channel + "%" + self.interface,
            ROBOT_LOGS_PORT,
            lambda data: self.__forward_to_proto_unix_io(RobotLog, data),
            True,
        )

        self.receive_log_visualize = HRVOVisualizationProtoListener(
            self.multicast_channel + "%" + self.interface,
            HRVO_VISUALIZATION_PORT,
            lambda data: self.current_proto_unix_io.send_proto(HRVOVisualization, data),
            True,
        )

        self.receive_robot_crash = RobotCrashProtoListener(
            self.multicast_channel + "%" + self.interface,
            ROBOT_CRASH_PORT,
            lambda data: self.current_proto_unix_io.send_proto(RobotCrash, data),
            True,
        )

        # Create multicast senders
        self.send_primitive_set = PrimitiveSetProtoSender(
            self.multicast_channel + "%" + self.interface, PRIMITIVE_PORT, True
        )

        self.running = True

        self.send_estop_state_thread.start()
        self.run_primitive_set_thread.start()

        return self

    def __exit__(self, type, value, traceback) -> None:
        """Exit RobotCommunication context manager

        Ends all currently running loops and joins all currently active threads

        """
        self.running = False
        self.receive_ssl_wrapper.close()
        self.receive_robot_log.close()
        self.receive_robot_status.close()
        self.run_world_thread.join()
        self.run_primitive_set_thread.join()
