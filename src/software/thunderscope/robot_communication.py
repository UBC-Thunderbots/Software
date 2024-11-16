from __future__ import annotations

from colorama import Fore, Style
import logging
import threading
from typing import Any, Callable, Tuple, Type
import time
from typing import Any, Callable, Tuple, Type
import os
import software.python_bindings as tbots_cpp

from google.protobuf.message import Message
from proto.import_all_protos import *
from software.logger.logger import create_logger
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.py_constants import *
from software.thunderscope.constants import (
    ROBOT_COMMUNICATIONS_TIMEOUT_S,
    IndividualRobotMode,
    EstopMode,
)

DISCONNECTED = "DISCONNECTED"
"""A constant to represent a disconnected interface"""

logger = logging.getLogger(__name__)


class RobotCommunication:
    """Communicate with the robots"""

    def __init__(
        self,
        current_proto_unix_io: ProtoUnixIO,
        multicast_channel: str,
        estop_mode: EstopMode,
        interface: str = None,
        estop_path: os.PathLike = None,
        estop_baudrate: int = 115200,
        enable_radio: bool = False,
        referee_port: int = SSL_REFEREE_PORT,
    ):
        """Initialize the communication with the robots

        :param current_proto_unix_io: the current proto unix io object
        :param multicast_channel: The multicast channel to use
        :param estop_mode: what estop mode we are running right now, of type EstopMode
        :param interface: The interface to use for communication with the robots
        :param estop_path: The path to the estop
        :param estop_baudrate: The baudrate of the estop
        :param enable_radio: Whether to use radio to send primitives to robots
        :param referee_port: the referee port that we are using. If this is None, the default port is used
        """
        self.is_setup_for_fullsystem = False

        self.referee_port = referee_port
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

        self.interface = interface
        self.running = False
        self.enable_radio = enable_radio

        self.logger = create_logger("RobotCommunication")

        self.fullsystem_primitive_set_buffer = ThreadSafeBuffer(1, PrimitiveSet)

        self.motor_control_primitive_buffer = ThreadSafeBuffer(1, MotorControl)
        self.power_control_primitive_buffer = ThreadSafeBuffer(1, PowerControl)

        # dynamic map of robot id to the individual control mode
        self.robot_control_mode_map: dict[int, IndividualRobotMode] = {}

        # static map of robot id to stop primitive
        self.robot_stop_primitives_map: dict[int, StopPrimitive] = {}

        # dynamic map of robot id to the number of times to send a stop primitive
        self.robot_stop_primitive_send_count_map: dict[int, int] = {}

        self.current_proto_unix_io.register_observer(
            PrimitiveSet, self.fullsystem_primitive_set_buffer
        )

        self.current_proto_unix_io.register_observer(
            MotorControl, self.motor_control_primitive_buffer
        )

        self.current_proto_unix_io.register_observer(
            PowerControl, self.power_control_primitive_buffer
        )

        # dynamic map of robot id to the individual control mode
        self.robot_control_mode_map: dict[int, IndividualRobotMode] = {}

        # static map of robot id to stop primitive
        self.robot_stop_primitives_map: dict[int, StopPrimitive] = {}

        # dynamic map of robot id to the number of times to send a stop primitive
        self.robot_stop_primitive_send_count_map: dict[int, int] = {}

        # Whether to accept the next configuration update. We will be provided a proto configuration from the
        # ProtoConfigurationWidget. If the user provides an interface, we will accept it as the first network
        # configuration and ignore the provided one from the widget. If not, we will wait for this first configuration
        self.accept_next_network_config = True
        self.current_network_config = NetworkConfig(
            robot_communication_interface=DISCONNECTED,
            referee_interface=DISCONNECTED,
            vision_interface=DISCONNECTED,
        )
        if interface:
            self.accept_next_network_config = False
            self.__setup_for_robot_communication(interface)
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

        # load control mode and stop primitive maps with default values
        for robot_id in range(MAX_ROBOT_IDS_PER_SIDE):
            self.robot_control_mode_map[robot_id] = IndividualRobotMode.NONE
            self.robot_stop_primitives_map[robot_id] = Primitive(stop=StopPrimitive())
            self.robot_stop_primitive_send_count_map[robot_id] = 0

        # TODO: (#3174): move estop state management out of robot_communication
        self.estop_mode = estop_mode
        self.estop_path = estop_path
        self.estop_baudrate = estop_baudrate

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
                    self.estop_path, self.estop_baudrate
                )
            except Exception:
                raise Exception(f"Invalid Estop found at location {self.estop_path}")

        self.print_current_network_config()

    def setup_for_fullsystem(
        self,
        referee_interface: str,
        vision_interface: str,
    ) -> None:
        """Sets up a listener for SSL vision and referee data

        :param referee_interface: the interface to listen for referee data
        :param vision_interface: the interface to listen for vision data
        """
        # Check cache to see if we're already connected
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
                logger.error(f"Error setting up vision interface:\n{error}")

            self.current_network_config.vision_interface = (
                vision_interface if not error else DISCONNECTED
            )

        if change_referee_interface:
            (
                self.receive_ssl_referee_proto,
                error,
            ) = tbots_cpp.createSSLRefereeProtoListener(
                SSL_REFEREE_ADDRESS,
                self.referee_port,
                referee_interface,
                lambda data: self.__forward_to_proto_unix_io(Referee, data),
                True,
            )

            if error:
                logger.error(f"Error setting up referee interface:\n{error}")

            self.current_network_config.referee_interface = (
                referee_interface if not error else DISCONNECTED
            )

        if not self.is_setup_for_fullsystem:
            self.robots_connected_to_fullsystem = {
                robot_id for robot_id in range(MAX_ROBOT_IDS_PER_SIDE)
            }

            self.is_setup_for_fullsystem = True

    def __setup_for_robot_communication(
        self, robot_communication_interface: str
    ) -> None:
        """Set up senders and listeners for communicating with the robots

        :param robot_communication_interface: the interface to listen/send for robot status data. Ignored for sending
        primitives if using radio
        """
        if (
            robot_communication_interface
            == self.current_network_config.robot_communication_interface
        ) or (robot_communication_interface == DISCONNECTED):
            return

        is_listener_setup_successfully = True

        def setup_listener(listener_creator: Callable[[], Tuple[Any, str]]) -> Any:
            """Sets up a listener with the given creator function. Logs any errors that occur.

            :param listener_creator: the function to create the listener. It must return a type of
            (listener object, error)
            """
            listener, error = listener_creator()
            if error:
                logger.error(f"Error setting up robot status interface:\n{error}")

            return listener

        # Create the multicast listeners
        self.receive_robot_status = setup_listener(
            lambda: tbots_cpp.createRobotStatusProtoListener(
                self.multicast_channel,
                ROBOT_STATUS_PORT,
                robot_communication_interface,
                self.__receive_robot_status,
                True,
            )
        )

        self.receive_robot_log = setup_listener(
            lambda: tbots_cpp.createRobotLogProtoListener(
                self.multicast_channel,
                ROBOT_LOGS_PORT,
                robot_communication_interface,
                lambda data: self.__forward_to_proto_unix_io(RobotLog, data),
                True,
            )
        )

        self.receive_robot_crash = setup_listener(
            lambda: tbots_cpp.createRobotCrashProtoListener(
                self.multicast_channel,
                ROBOT_CRASH_PORT,
                robot_communication_interface,
                lambda data: self.current_proto_unix_io.send_proto(RobotCrash, data),
                True,
            )
        )

        # Create multicast senders
        if self.enable_radio:
            self.send_primitive_set = tbots_cpp.PrimitiveSetProtoRadioSender()
        else:
            self.send_primitive_set, error = tbots_cpp.createPrimitiveSetProtoUdpSender(
                self.multicast_channel,
                PRIMITIVE_PORT,
                robot_communication_interface,
                True,
            )

            if error:
                is_listener_setup_successfully = False
                logger.error(f"Error setting up primitive set sender:\n{error}")

        self.current_network_config.robot_communication_interface = (
            robot_communication_interface
            if is_listener_setup_successfully
            else DISCONNECTED
        )

        # set all robots to AI control in the control mode map
        self.robot_control_mode_map.update(
            (robot_id, IndividualRobotMode.AI)
            for robot_id in self.robot_control_mode_map.keys()
        )


    def toggle_keyboard_estop(self) -> None:
        """If keyboard estop is being used, toggles the estop state
        And sends a message to the console
        """
        if self.estop_mode == EstopMode.KEYBOARD_ESTOP:
            self.estop_is_playing = not self.estop_is_playing
            self.logger.debug(
                "Keyboard Estop changed to "
                + (
                    "\x1b[32mPLAY \x1b[0m"
                    if self.estop_is_playing
                    else "\x1b[31;20mSTOP \x1b[0m"
                )
            )

    def toggle_individual_robot_control_mode(
        self, robot_id: int, mode: IndividualRobotMode
    ):
        """Changes the input mode for a robot between NONE, MANUAL, or AI
        If changing from MANUAL OR AI to NONE, add robot id to stop primitive
        map so that multiple stop primitives are sent - safety number one priority

        :param mode: the mode of input for this robot's primitives
        :param robot_id: the id of the robot whose mode we're changing
        """
        self.robot_control_mode_map[robot_id] = mode
        self.robot_stop_primitive_send_count_map[robot_id] = (
            NUM_TIMES_SEND_STOP if mode == IndividualRobotMode.NONE else 0
        )

    def __send_estop_state(self) -> None:
        """Constant loop which sends the current estop status proto if estop is not disabled
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
        """Returns True if the proto sending threads should send a proto

        :return: boolean
        """
        return (
            self.estop_mode != EstopMode.DISABLE_ESTOP
            and self.estop_is_playing
            and (
                IndividualRobotMode.AI
                or IndividualRobotMode.MANUAL
                in self.robot_id_individual_mode_dict.values()
            )
        )

    def __run_primitive_set(self) -> None:
        """Forward PrimitiveSet protos from Fullsystem and MotorControl/PowerControl
        protos from Robot Diagnostics to the robots.

        For AI protos, blocks for 10ms if no proto is available, and then returns a cached proto

        For Diagnostics protos, does not block and returns cached message if none available
        Sleeps for 10ms for diagnostics

        If the emergency stop is tripped, the PrimitiveSet will not be sent so
        that the robots timeout and stop.
        """
        network_config = self.network_config_buffer.get(
            block=True if self.accept_next_network_config else False,
            return_cached=False,
        )
        while self.running:
            if network_config is not None and self.accept_next_network_config:
                logging.info("[RobotCommunication] Received new NetworkConfig")

                if self.is_setup_for_fullsystem:
                    self.setup_for_fullsystem(
                        referee_interface=network_config.referee_interface,
                        vision_interface=network_config.vision_interface,
                    )
                self.__setup_for_robot_communication(
                    robot_communication_interface=network_config.robot_communication_interface
                )
                self.print_current_network_config()
            elif network_config is not None:
                logger.warning(
                    "[RobotCommunication] We received a proto configuration update with a newer network"
                    " configuration. We will ignore this update, likely because the interface was provided at startup but"
                    " the next update will be accepted."
                )
                self.accept_next_network_config = True
                self.print_current_network_config()

            # Set up network on the next tick
            network_config = self.network_config_buffer.get(
                block=False, return_cached=False
            )

            # map of robot id to diagnostics/fullsystem primitive map
            robot_primitives_map = {}

            # get the most recent diagnostics primitive
            motor_control = self.motor_control_primitive_buffer.get(block=False)
            power_control = self.power_control_primitive_buffer.get(block=False)

            diagnostics_primitive = Primitive(
                direct_control=DirectControlPrimitive(
                    motor_control=motor_control,
                    power_control=power_control,
                )
            )

            # filter for diagnostics controlled robots
            diagnostics_robots = list(
                robot_id
                for robot_id, mode in self.robot_control_mode_map.items()
                if mode == IndividualRobotMode.MANUAL
            )

            # set diagnostics primitives for diagnostics robots
            for robot_id in diagnostics_robots:
                robot_primitives_map[robot_id] = diagnostics_primitive

            # get the most recent fullsystem primitives
            fullsystem_primitive_set = self.fullsystem_primitive_set_buffer.get(
                block=True, timeout=ROBOT_COMMUNICATIONS_TIMEOUT_S
            )

            # filter for fullsystem controlled robots
            fullsystem_robots = list(
                robot_id
                for robot_id, mode in self.robot_control_mode_map.items()
                if mode == IndividualRobotMode.AI
            )

            # set fullsystem primitives for fullsystem robots
            for robot_id in fullsystem_robots:
                robot_primitives_map[robot_id] = (
                    fullsystem_primitive_set.robot_primitives[robot_id]
                )

            # sends a final stop primitive to all disconnected robots and removes them from list
            # in order to prevent robots acting on cached old primitives
            for (
                robot_id,
                num_times_to_stop,
            ) in self.robot_stop_primitive_send_count_map.items():
                if num_times_to_stop > 0:
                    robot_primitives_map[robot_id] = Primitive(stop=StopPrimitive())
                    self.robot_stop_primitive_send_count_map[robot_id] = (
                        num_times_to_stop - 1
                    )

            # initialize total primitive set and send it
            primitive_set = PrimitiveSet(
                time_sent=Timestamp(epoch_timestamp_seconds=time.time()),
                stay_away_from_ball=False,
                robot_primitives=self.robot_stop_primitives_map
                if self.should_send_stop
                else robot_primitives_map,
                sequence_number=self.sequence_number,
            )

            self.sequence_number += 1

            if self.__should_send_packet() or self.should_send_stop:
                self.send_primitive_set.send_proto(primitive_set)
                self.should_send_stop = False

            # sleep if not running fullsystem
            if IndividualRobotMode.AI not in self.robot_control_mode_map.values():
                time.sleep(ROBOT_COMMUNICATIONS_TIMEOUT_S)

    def __forward_to_proto_unix_io(self, type: Type[Message], data: Message) -> None:
        """Forwards to proto unix IO iff running is true

        :param data: the data to be passed through
        :param type: the proto type
        """
        if self.running:
            self.current_proto_unix_io.send_proto(type, data)

    def __enter__(self) -> RobotCommunication:
        """Enter RobotCommunication context manager. Setup multicast listeners
        for RobotStatus, RobotLogs, and RobotCrash msgs, and multicast sender for PrimitiveSet
        """
        self.running = True

        self.send_estop_state_thread.start()
        self.run_primitive_set_thread.start()

        return self

    def __receive_robot_status(self, robot_status: Message) -> None:
        """Forwards the given robot status to the full system along with the round-trip time

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

        self.run_primitive_set_thread.join()

    def print_current_network_config(self) -> None:
        """Prints the current network configuration to the console"""

        def output_string(comm_name: str, status: str) -> str:
            """Returns a formatted string with the communication name and status

            Any status other than DISCONNECTED will be coloured green, otherwise red

            :param comm_name: the name of the communication
            :param status: the status of the communication
            """
            colour = Fore.RED if status == DISCONNECTED else Fore.GREEN
            return f"{comm_name} {colour}{status} {Style.RESET_ALL}"

        logging.info(
            output_string(
                "Robot Status\t",
                self.current_network_config.robot_communication_interface,
            )
        )
        logging.info(
            output_string(
                "Vision\t\t",
                self.current_network_config.vision_interface
                if self.is_setup_for_fullsystem
                else DISCONNECTED,
            )
        )
        logging.info(
            output_string(
                "Referee\t\t",
                self.current_network_config.referee_interface
                if self.is_setup_for_fullsystem
                else DISCONNECTED,
            )
        )
