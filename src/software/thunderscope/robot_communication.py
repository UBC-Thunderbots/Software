from software.py_constants import *
from software.thunderscope.constants import ROBOT_COMMUNICATIONS_TIMEOUT_S
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.constants import IndividualRobotMode, EstopMode
import software.python_bindings as tbots_cpp
from software.thunderscope.proto_unix_io import ProtoUnixIO
from proto.import_all_protos import *
from pyqtgraph.Qt import QtCore
from software.thunderscope.proto_unix_io import ProtoUnixIO
from colorama import Fore, Style
from typing import Type
import threading
import time
import os
from google.protobuf.message import Message

DISCONNECTED = "DISCONNECTED"
"""A constant to represent a disconnected interface"""


class RobotCommunication(object):

    """ Communicate with the robots """

    def __init__(
        self,
        current_proto_unix_io: ProtoUnixIO,
        multicast_channel: str,
        estop_mode: EstopMode,
        estop_path: os.PathLike = None,
        estop_baudrate: int = 115200,
        enable_radio: bool = False,
    ):
        """Initialize the communication with the robots

        :param current_proto_unix_io: the current proto unix io object
        :param multicast_channel: The multicast channel to use
        :param estop_mode: what estop mode we are running right now, of type EstopMode
        :param estop_path: The path to the estop
        :param estop_baudrate: The baudrate of the estop
        :param enable_radio: Whether to use radio to send primitives to robots

        """
        self.is_setup_for_fullsystem = False
        self.receive_ssl_referee_proto = None
        self.receive_ssl_wrapper = None

        self.receive_robot_status = None
        self.receive_robot_log = None
        self.receive_robot_crash = None
        self.send_primitive_set = None

        self.sequence_number = 0
        self.last_time = time.time()
        self.current_proto_unix_io = current_proto_unix_io
        self.multicast_channel = str(multicast_channel)
        self.estop_mode = estop_mode

        self.estop_path = estop_path
        self.estop_buadrate = estop_baudrate

        self.enable_radio = enable_radio

        self.running = False

        self.primitive_buffer = ThreadSafeBuffer(1, PrimitiveSet)

        self.motor_control_diagnostics_buffer = ThreadSafeBuffer(1, MotorControl)
        self.power_control_diagnostics_buffer = ThreadSafeBuffer(1, PowerControl)

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

        self.current_network_config = NetworkConfig(
            robot_status_interface=DISCONNECTED,
            vision_interface=DISCONNECTED,
            referee_interface=DISCONNECTED,
        )
        self.network_config_buffer = ThreadSafeBuffer(1, NetworkConfig)
        self.current_proto_unix_io.register_observer(
            NetworkConfig, self.network_config_buffer
        )

        self.send_estop_state_thread = threading.Thread(
            target=self.__send_estop_state, daemon=True
        )
        self.run_primitive_set_thread = threading.Thread(
            target=self.__run_primitive_set, daemon=True
        )

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

    def setup_for_fullsystem(
        self,
        referee_interface: str = DISCONNECTED,
        vision_interface: str = DISCONNECTED,
    ) -> None:
        """
        Sets up a listener for SSL vision and referee data, and connects all robots to fullsystem as default

        :param referee_interface: the interface to listen for referee data
        :param vision_interface: the interface to listen for vision data
        """
        change_referee_interface = (
            referee_interface != self.current_network_config.referee_interface
        ) and (referee_interface != DISCONNECTED)
        change_vision_interface = (
            vision_interface != self.current_network_config.vision_interface
        ) and (vision_interface != DISCONNECTED)

        if change_vision_interface:

            (
                self.receive_ssl_wrapper,
                error,
            ) = tbots_cpp.createSSLWrapperPacketProtoListener(
                SSL_VISION_ADDRESS,
                SSL_VISION_PORT,
                vision_interface,
                lambda data: self.__forward_to_proto_unix_io(SSL_WrapperPacket, data),
                True,
            )

            if error:
                print(f"Error setting up vision interface: {error}")

            self.current_network_config.vision_interface = (
                vision_interface if not error else DISCONNECTED
            )

        if change_referee_interface:
            (
                self.receive_ssl_referee_proto,
                error,
            ) = tbots_cpp.createSSLRefereeProtoListener(
                SSL_REFEREE_ADDRESS,
                SSL_REFEREE_PORT,
                referee_interface,
                lambda data: self.__forward_to_proto_unix_io(Referee, data),
                True,
            )

            if error:
                print(f"Error setting up referee interface: {error}")

            self.current_network_config.referee_interface = (
                referee_interface if not error else DISCONNECTED
            )

        if not self.is_setup_for_fullsystem:
            self.robots_connected_to_fullsystem = {
                robot_id for robot_id in range(MAX_ROBOT_IDS_PER_SIDE)
            }

            self.is_setup_for_fullsystem = True

    def __setup_for_robot_communication(
        self, robot_status_interface: str = "lo"
    ) -> None:
        """
        Set up senders and listeners for communicating with the robots

        :param robot_status_interface: the interface to listen/send for robot status data. Ignored for sending
        primitives if using radio
        """
        if (
            robot_status_interface == self.current_network_config.robot_status_interface
            or robot_status_interface == DISCONNECTED
        ):
            return

        # Create the multicast listeners
        self.receive_robot_status, error = tbots_cpp.createRobotStatusProtoListener(
            self.multicast_channel,
            ROBOT_STATUS_PORT,
            robot_status_interface,
            self.__receive_robot_status,
            True,
        )

        self.receive_robot_log, error = tbots_cpp.createRobotLogProtoListener(
            self.multicast_channel,
            ROBOT_LOGS_PORT,
            robot_status_interface,
            lambda data: self.__forward_to_proto_unix_io(RobotLog, data),
            True,
        )

        self.receive_robot_crash, error = tbots_cpp.createRobotCrashProtoListener(
            self.multicast_channel,
            ROBOT_CRASH_PORT,
            robot_status_interface,
            lambda data: self.current_proto_unix_io.send_proto(RobotCrash, data),
            True,
        )

        # Create multicast senders
        if self.enable_radio:
            self.send_primitive_set = tbots_cpp.PrimitiveSetProtoRadioSender()
        else:
            self.send_primitive_set, error = tbots_cpp.createPrimitiveSetProtoUdpSender(
                self.multicast_channel, PRIMITIVE_PORT, robot_status_interface, True
            )

        if error:
            print(f"Error setting up robot status interface: {error}")

        self.current_network_config.robot_status_interface = (
            robot_status_interface if not error else DISCONNECTED
        )

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

    def toggle_robot_connection(self, mode: IndividualRobotMode, robot_id: int):
        """
        Changes the input mode for a robot between None, Manual, or AI
        If changing from anything to None, add robot to disconnected map
        So we can send multiple stop primitives to make sure it stops

        :param mode: the mode of input for this robot's primitives
        :param robot_id: the id of the robot whose mode we're changing
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
            and (self.robots_connected_to_fullsystem or self.robots_connected_to_manual)
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
        network_config = self.network_config_buffer.get(
            block=False if self.send_primitive_set else True, return_cached=False
        )
        while self.running:
            if network_config is not None:
                print(f"[RobotCommunication] Received new NetworkConfig")
                if self.is_setup_for_fullsystem:
                    self.setup_for_fullsystem(
                        referee_interface=network_config.referee_interface,
                        vision_interface=network_config.vision_interface,
                    )
                self.__setup_for_robot_communication(
                    robot_status_interface=network_config.robot_status_interface
                )
                self.__print_current_network_config()

            # Set up network on the next tick
            network_config = self.network_config_buffer.get(
                block=False, return_cached=False
            )

            # total primitives for all robots
            robot_primitives = {}

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
            if not self.robots_connected_to_fullsystem:
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
        self.running = True

        self.send_estop_state_thread.start()
        self.run_primitive_set_thread.start()

        return self

    def __receive_robot_status(self, robot_status: Message) -> None:
        """
        Forwards the given robot status to the full system along with the round-trip time
        :param robot_status: RobotStatus to forward to fullsystem
        """
        round_trip_time_seconds = time.time() - (
            robot_status.adjusted_time_sent.epoch_timestamp_seconds
        )
        self.__forward_to_proto_unix_io(
            RobotStatistic,
            RobotStatistic(round_trip_time_seconds=round_trip_time_seconds),
        )
        self.__forward_to_proto_unix_io(RobotStatus, robot_status)

    def __exit__(self, type, value, traceback) -> None:
        """Exit RobotCommunication context manager

        Ends all currently running loops and joins all currently active threads

        """
        self.running = False

        self.close_for_fullsystem()

        self.receive_robot_log.close()
        self.receive_robot_status.close()
        self.run_primitive_set_thread.join()

    def __print_current_network_config(self) -> None:
        """
        Prints the current network configuration to the console
        """

        def output_string(comm_name: str, status: str) -> str:
            """
            Returns a formatted string with the communication name and status

            Any status other than DISCONNECTED will be coloured green, otherwise red

            :param comm_name: the name of the communication
            :param status: the status of the communication
            """
            colour = Fore.RED if status == DISCONNECTED else Fore.GREEN
            return f"{comm_name} {colour}{status} {Style.RESET_ALL}"

        print(
            output_string(
                "Robot Status\t", self.current_network_config.robot_status_interface
            )
        )
        print(output_string("Vision\t\t", self.current_network_config.vision_interface))
        print(
            output_string("Referee\t\t", self.current_network_config.referee_interface)
        )
