import threading
import time
import os

from software.thunderscope.constants import (
    ROBOT_COMMUNICATIONS_TIMEOUT_S,
    IndividualRobotMode,
    EstopMode,
)
from software.thunderscope.robot_diagnostics.diagnostics_input_widget import ControlMode
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.proto_unix_io import ProtoUnixIO
import software.python_bindings as tbots_cpp
from software.py_constants import *
from google.protobuf.message import Message
from proto.import_all_protos import *
from pyqtgraph.Qt import QtCore
from typing import Type
from queue import Empty

NUM_ROBOTS = 6


class RobotCommunication(object):

    """ Communicate with the robots """

    def __init__(
        self,
        current_proto_unix_io: ProtoUnixIO,
        multicast_channel: str,
        interface: str,
        estop_mode: EstopMode,
        estop_path: os.PathLike = None,
        estop_baudrate: int = 115200,
    ):
        """Initialize the communication with the robots

        :param current_proto_unix_io: the current proto unix io object
        :param multicast_channel: The multicast channel to use
        :param interface: The interface to use
        :param estop_mode: what estop mode we are running right now, of type EstopMode
        :param estop_path: The path to the estop
        :param estop_baudrate: The baudrate of the estop

        """
        self.receive_ssl_referee_proto = None
        self.receive_ssl_wrapper = None
        self.sequence_number = 0
        self.last_time = time.time()
        self.current_proto_unix_io = current_proto_unix_io
        self.multicast_channel = str(multicast_channel)
        self.interface = interface
        self.estop_mode = estop_mode

        self.estop_path = estop_path
        self.estop_buadrate = estop_baudrate

        self.running = False

        self.primitive_buffer = ThreadSafeBuffer(1, PrimitiveSet)

        self.motor_control_diagnostics_buffer = ThreadSafeBuffer(1, MotorControl)
        self.power_control_diagnostics_buffer = ThreadSafeBuffer(1, PowerControl)

        self.current_proto_unix_io.register_observer(
            PrimitiveSet, self.primitive_buffer
        )

        self.current_proto_unix_io.register_observer(
            MotorControl, self.motor_control_diagnostics_buffer
        )
        self.current_proto_unix_io.register_observer(
            PowerControl, self.power_control_diagnostics_buffer
        )

        self.send_estop_state_thread = threading.Thread(
            target=self.__send_estop_state, daemon=True
        )
        self.run_primitive_set_thread = threading.Thread(
            target=self.__run_primitive_set, daemon=True
        )

        self.robot_id_individual_mode_dict = {
            rid: IndividualRobotMode.NONE for rid in range(NUM_ROBOTS)
        }

        self.robot_id_estop_send_count_dict = {
            rid: 0 for rid in range(NUM_ROBOTS) # TODO
        }

        # default to diagnostics control.
        self.__input_mode = ControlMode.DIAGNOSTICS

        # initialising the estop
        # tries to access a plugged in estop. if not found, throws an exception
        # if using keyboard estop, skips this step
        self.estop_reader = None
        self.estop_is_playing = False
        # when the estop has just been stopped,
        # we want to send a stop primitive once to all currently connected robots
        self.should_send_stop = False

        # only checks for estop if we are in physical estop mode
        if self.estop_mode == EstopMode.PHYSICAL_ESTOP:
            try:
                self.estop_reader = tbots_cpp.ThreadedEstopReader(
                    self.estop_path, self.estop_buadrate
                )
            except Exception:
                raise Exception(f"Invalid Estop found at location {self.estop_path}")

    def setup_for_fullsystem(self) -> None:
        """
        Sets up a listener for SSL vision and referee data, and connects all robots to fullsystem as default
        """
        self.receive_ssl_wrapper = tbots_cpp.SSLWrapperPacketProtoListener(
            SSL_VISION_ADDRESS,
            SSL_VISION_PORT,
            lambda data: self.__forward_to_proto_unix_io(SSL_WrapperPacket, data),
            True,
        )

        self.receive_ssl_referee_proto = tbots_cpp.SSLRefereeProtoListener(
            SSL_REFEREE_ADDRESS,
            SSL_REFEREE_PORT,
            lambda data: self.current_proto_unix_io.send_proto(Referee, data),
            True,
        )

        for key in self.robot_id_individual_mode_dict:
            self.robot_id_individual_mode_dict[key] = IndividualRobotMode.AI


    def close_for_fullsystem(self) -> None:
        if self.receive_ssl_wrapper:
            self.receive_ssl_wrapper.close()

        if self.receive_ssl_referee_proto:
            self.receive_ssl_referee_proto.close()

    def toggle_keyboard_estop(self) -> None:
        """
        If keyboard estop is being used, toggles the estop state
        And sends a message to the console
        """
        if self.estop_mode == EstopMode.KEYBOARD_ESTOP:
            self.estop_is_playing = not self.estop_is_playing

            print(
                "Keyboard Estop changed to "
                + (
                    f"\x1b[32mPLAY \x1b[0m"
                    if self.estop_is_playing
                    else f"\x1b[31;20mSTOP \x1b[0m"
                )
            )

    def toggle_robot_control_mode(self, robot_id: int, mode: IndividualRobotMode):
        """
        Changes the input mode for a robot between None, Manual, or AI
        If changing from anything to None, add robot to disconnected map
        So we can send multiple stop primitives to make sure it stops

        :param mode: the mode of input for this robot's primitives
        :param robot_id: the id of the robot whose mode we're changing
        """
        self.robot_id_individual_mode_dict[robot_id] = mode

        self.robot_id_estop_send_count_dict.pop(robot_id, None)

        if mode == IndividualRobotMode.NONE:
            self.robot_id_estop_send_count_dict[robot_id] = NUM_TIMES_SEND_STOP

    def toggle_input_mode(self, mode: ControlMode):
        """
        Changes the diagnostics input mode for all robots between Xbox and Diagnostics.

        :param mode: Control mode to use when sending diagnostics primitives
        """
        self.__input_mode = mode

    def __send_estop_state(self) -> None:
        """
        Constant loop which sends the current estop status proto if estop is not disabled
        Uses the keyboard estop value for keyboard estop mode
        If we're in physical estop mode, uses the physical estop value
        If estop has just changed from playing to stop, set flag to send stop primitive once to connected robots
        """
        previous_estop_is_playing = True
        if self.estop_mode != EstopMode.DISABLE_ESTOP:
            while True:
                if self.estop_mode == EstopMode.PHYSICAL_ESTOP:
                    self.estop_is_playing = self.estop_reader.isEstopPlay()

                # Send stop primitive once when estop is paused
                if previous_estop_is_playing and not self.estop_is_playing:
                    self.should_send_stop = True
                else:
                    self.should_send_stop = False

                previous_estop_is_playing = self.estop_is_playing

                self.current_proto_unix_io.send_proto(
                    EstopState, EstopState(is_playing=self.estop_is_playing)
                )
                time.sleep(0.1)

    def __should_send_packet(self) -> bool:
        """
        Returns True if the proto sending threads should send a proto
        :return: boolean
        """
        return (
                self.estop_mode != EstopMode.DISABLE_ESTOP
                and self.estop_is_playing
                and (IndividualRobotMode.AI or
                     IndividualRobotMode.MANUAL
                     in self.robot_id_individual_mode_dict.values())
        )

    def __run_primitive_set(self) -> None:
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

            # diagnostics is running. Only send diagnostics packets if using diagnostics control mode
            if self.__input_mode == ControlMode.DIAGNOSTICS:
                # get the manual control primitive
                diagnostics_primitive = DirectControlPrimitive(
                    motor_control=self.motor_control_diagnostics_buffer.get(block=False),
                    power_control=self.power_control_diagnostics_buffer.get(block=False),
                )

                manually_controlled_robots = filter(
                    lambda robot_mode: robot_mode[1] == IndividualRobotMode.MANUAL,
                           self.robot_id_individual_mode_dict)

                for robot_id in manually_controlled_robots:
                    robot_primitives[robot_id] = Primitive(
                        direct_control=diagnostics_primitive)

            # fullsystem is running, so world data is being received
            ai_controlled_robots: dict[int, IndividualRobotMode] = (
                filter(lambda robot_mode: robot_mode[1] == IndividualRobotMode.AI,
                       self.robot_id_individual_mode_dict)
            )

            if ai_controlled_robots:
                # Get the primitives
                primitive_set = self.primitive_buffer.get(
                    block=True, timeout=ROBOT_COMMUNICATIONS_TIMEOUT_S
                )

                fullsystem_primitives = dict(primitive_set.robot_primitives)
                for robot_id in fullsystem_primitives.keys():
                    if robot_id in ai_controlled_robots:
                        robot_primitives[robot_id] = fullsystem_primitives[robot_id]

            # sends a final stop primitive to all disconnected robots and removes them from list
            # in order to prevent robots acting on cached old primitives
            for robot_id, num_times_to_stop in self.robot_id_estop_send_count_dict.items():
                if num_times_to_stop > 0:
                    robot_primitives[robot_id] = Primitive(stop=StopPrimitive())
                    self.robot_id_estop_send_count_dict[robot_id] = num_times_to_stop - 1

            # initialize total primitive set and send it
            primitive_set = PrimitiveSet(
                time_sent=Timestamp(epoch_timestamp_seconds=time.time()),
                stay_away_from_ball=False,
                robot_primitives=robot_primitives
                if not self.should_send_stop
                else {
                    robot_id: Primitive(stop=StopPrimitive())
                    for robot_id in robot_primitives.keys()
                },
                sequence_number=self.sequence_number,
            )

            self.sequence_number += 1

            if self.__should_send_packet() or self.should_send_stop:
                self.send_primitive_set.send_proto(primitive_set)
                self.should_send_stop = False

            # sleep if not running fullsystem
            if not ai_controlled_robots:
                time.sleep(ROBOT_COMMUNICATIONS_TIMEOUT_S)

    def __forward_to_proto_unix_io(self, type: Type[Message], data: Message) -> None:
        """
        Forwards to proto unix IO iff running is true
        :param data: the data to be passed through
        :param type: the proto type
        """
        if self.running:
            self.current_proto_unix_io.send_proto(type, data)

    def __enter__(self) -> "self":
        """Enter RobotCommunication context manager. Setup multicast listeners
        for RobotStatus, RobotLogs, and RobotCrash msgs, and multicast sender for PrimitiveSet

        """
        # Create the multicast listeners
        self.receive_robot_status = tbots_cpp.RobotStatusProtoListener(
            self.multicast_channel + "%" + self.interface,
            ROBOT_STATUS_PORT,
            lambda data: self.__forward_to_proto_unix_io(RobotStatus, data),
            True,
        )

        self.receive_robot_log = tbots_cpp.RobotLogProtoListener(
            self.multicast_channel + "%" + self.interface,
            ROBOT_LOGS_PORT,
            lambda data: self.__forward_to_proto_unix_io(RobotLog, data),
            True,
        )

        self.receive_robot_crash = tbots_cpp.RobotCrashProtoListener(
            self.multicast_channel + "%" + self.interface,
            ROBOT_CRASH_PORT,
            lambda data: self.current_proto_unix_io.send_proto(RobotCrash, data),
            True,
        )

        # Create multicast senders
        self.send_primitive_set = tbots_cpp.PrimitiveSetProtoSender(
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

        self.close_for_fullsystem()

        self.receive_robot_log.close()
        self.receive_robot_status.close()
        self.run_primitive_set_thread.join()
