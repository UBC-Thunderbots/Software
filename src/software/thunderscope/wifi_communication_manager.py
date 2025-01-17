from typing import Any, Callable, Self, Type

from proto.import_all_protos import *
from software.logger.logger import create_logger
import software.python_bindings as tbots_cpp
from software.py_constants import *
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

from google.protobuf.message import Message

from colorama import Fore, Style
import logging
from threading import Lock, Thread
import time

DISCONNECTED = "DISCONNECTED"
"""A constant to represent a disconnected interface"""

logger = create_logger(name=__name__, log_level=logging.DEBUG)


class WifiCommunicationManager:
    """Manages WiFi communication between different modules of the system."""

    BROADCAST_HZ = 1.0
    """The frequency at which to broadcast the full system IP address"""

    def __init__(
        self,
        current_proto_unix_io: ProtoUnixIO,
        multicast_channel: str,
        should_setup_full_system: bool = False,
        interface: str | None = None,
        referee_port: int = SSL_REFEREE_PORT,
    ):
        """Sets up WiFi communication between this computer and the robots, SSL Vision, and SSL Referee

        :param current_proto_unix_io: the current proto unix io object
        :param multicast_channel: The multicast channel to use
        :param interface: The interface to use for communication with the robots
        :param referee_port: the referee port that we are using. If this is None, the default port is used
        """
        ## Robot IP address tracking ##
        self.robot_ip_addresses: list[tuple[Lock, str | None]] = [
            (Lock(), None) for _ in range(MAX_ROBOT_IDS_PER_SIDE)
        ]

        ## Senders and listeners ##
        self.primitive_senders: list[tuple[Lock, tbots_cpp.PrimitiveSender | None]] = [
            (Lock(), None) for _ in range(MAX_ROBOT_IDS_PER_SIDE)
        ]
        self.receive_robot_status: tbots_cpp.RobotStatusProtolistener | None = None
        self.receive_robot_log: tbots_cpp.RobotLogProtolistener | None = None
        self.receive_robot_crash: tbots_cpp.RobotCrashProtolistener | None = None
        self.receive_ssl_referee_proto: tbots_cpp.SSLRefereeProtolistener | None = None
        self.receive_ssl_wrapper: tbots_cpp.SSLWrapperPacketProtolistener | None = None
        self.robot_ip_listener: tbots_cpp.RobotIpNotificationProtolistener | None = None
        self.fullsystem_ip_broadcaster: tuple[
            Lock, tbots_cpp.FullsystemIpBroadcast | None, IpNotification
        ] = (Lock(), None, IpNotification())

        ## ProtoUnixIO ##
        self.current_proto_unix_io = current_proto_unix_io

        ## Network Configuration ##
        self.multicast_channel = multicast_channel
        self.network_config_buffer = ThreadSafeBuffer(1, NetworkConfig)
        self.current_proto_unix_io.register_observer(
            NetworkConfig, self.network_config_buffer
        )
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
            self.__setup_robot_communication(interface)
        self.referee_port = referee_port

        self.should_setup_full_system = should_setup_full_system
        if interface and should_setup_full_system:
            self.__setup_full_system(
                referee_interface=interface, vision_interface=interface
            )

        ## Thread Management ##
        self.running = True
        self.broadcast_ip: Thread | None = None

        logger.debug("[WifiCommunicationManager] Initialized")
        self.__print_current_network_config()

    def __enter__(self) -> Self:
        self.broadcast_ip = Thread(target=self.__broadcast_fullsystem_ip, daemon=True)
        self.broadcast_ip.start()
        return self

    def __exit__(self, type, value, traceback) -> None:
        self.running = False
        self.broadcast_ip.join()

    def __broadcast_fullsystem_ip(self) -> None:
        """Notify the robots of this computer's IP address"""
        while self.running:
            with self.fullsystem_ip_broadcaster[0]:
                fullsystem_ip_broadcaster = self.fullsystem_ip_broadcaster[1]
                if fullsystem_ip_broadcaster is not None:
                    fullsystem_ip_broadcaster.send_proto(
                        self.fullsystem_ip_broadcaster[2], True
                    )
            time.sleep(1.0 / WifiCommunicationManager.BROADCAST_HZ)

    def __connect_to_robot(self, robot_id: int) -> None:
        """Setup a primitive sender for the robot with the given id.

        :param robot_id: the id of the robot to connect to
        """
        if self.current_network_config.robot_communication_interface == DISCONNECTED:
            logger.warning("Robot communication interface isn't set")
            return

        ip_address = str()
        with self.robot_ip_addresses[robot_id][0]:
            ip_address = self.robot_ip_addresses[robot_id][1]

        if ip_address is None:
            return

        with self.primitive_senders[robot_id][0]:
            primitive_sender = self.primitive_senders[robot_id][1]
            if primitive_sender is None or (
                primitive_sender.get_ip_address() != ip_address
                or primitive_sender.get_interface()
                != self.current_network_config.robot_communication_interface
            ):
                try:
                    primitive_sender = tbots_cpp.PrimitiveProtoUdpSender(
                        ip_address,
                        PRIMITIVE_PORT,
                        self.current_network_config.robot_communication_interface,
                        False,
                    )
                    self.primitive_senders[robot_id] = (
                        self.primitive_senders[robot_id][0],
                        primitive_sender,
                    )
                    logger.info(f"Connected to robot {robot_id} at {ip_address}")
                except tbots_cpp.TbotsNetworkException:
                    logger.error(f"Error connecting to robot {robot_id}: {error}")


    def __forward_to_proto_unix_io(self, type: Type[Message], data: Message) -> None:
        """Forwards to proto unix IO iff running is true

        :param data: the data to be passed through
        :param type: the proto type
        """
        if self.running:
            self.current_proto_unix_io.send_proto(type, data)

    def __print_current_network_config(self) -> None:
        """Prints the current network configuration to the console."""

        def output_string(comm_name: str, status: str) -> str:
            """Returns a formatted string with the given module we are communicating to, coloured based on the status.

            Any status other than DISONNECTED will be coloured green, otherwise red.

            :param comm_name: Whether we are connectng to the robot, SSL Vision or SSL Referee
            :param status: The status to report to the user
            """
            colour = Fore.RED if status == DISCONNECTED else Fore.GREEN

            return f"{comm_name}: {colour}{status} {Style.RESET_ALL}"

        logging.info(
            output_string(
                "Robot Status\t",
                self.current_network_config.robot_communication_interface,
            )
        )
        logging.info(
            output_string("Vision\t\t", self.current_network_config.vision_interface)
        )
        logging.info(
            output_string("Referee\t", self.current_network_config.referee_interface)
        )

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

    def __setup_full_system(self, referee_interface: str, vision_interface: str) -> None:
        """Connect to the SSL Referee and SSL Vision interfaces.

        :param referee_interface: the interface to listen for SSL Referee data
        :param vision_interface: the interface to listen for SSL Vision data
        """
        change_referee_interface = (
            referee_interface != self.current_network_config.referee_interface
        ) and (referee_interface != DISCONNECTED)

        change_vision_interface = (
            vision_interface != self.current_network_config.vision_interface
        ) and (vision_interface != DISCONNECTED)

        if change_referee_interface:
            try:
                self.receive_ssl_referee_proto = tbots_cpp.SSLRefereeProtoListener(
                    SSL_REFEREE_ADDRESS,
                    self.referee_port,
                    referee_interface,
                    lambda data: self.__forward_to_proto_unix_io(SSL_Referee, data),
                    True,
                )
                self.current_network_config.referee_interface = referee_interface
            except tbots_cpp.TbotsNetworkException as e:
                logger.error(f"Error setting up referee interface:\n{e}")
                self.current_network_config.referee_interface = DISCONNECTED

        if change_vision_interface:
            try:
                self.receive_ssl_wrapper = tbots_cpp.SSLWrapperPacketProtoListener(
                    SSL_VISION_ADDRESS,
                    SSL_VISION_PORT,
                    vision_interface,
                    lambda data: self.__forward_to_proto_unix_io(SSL_WrapperPacket, data),
                    True,
                )
                self.current_network_config.vision_interface = vision_interface
            except tbots_cpp.TbotsNetworkException as e:
                logger.error(f"Error setting up vision interface:\n{e}")
                self.current_network_config.vision_interface = DISCONNECTED


    def __setup_robot_communication(self, robot_communication_interface: str) -> None:
        """Set up senders and listeners for communicating with the robots

        :param robot_communication_interface: the interface to listen/send for robot status data. Ignored for sending
        primitives if using radio
        """
        is_setup_successfully = True

        def setup_network_resource(creator: Callable[[], tuple[Any, str]]) -> Any:
            """Sets up a network node with the given creator function. Logs any errors that occur.

            :param creator: the function to create the resource.
            """
            try:
                resource = creator()
            except tbots_cpp.TbotsNetworkException:
                logger.error(f"Error setting up robot status interface:\n{error}")
                is_setup_successfully = False
            return resource

        # Create unicast listeners for RobotStatus and RobotLog. These are all binded to all interfaces
        if self.receive_robot_status is None:
            self.receive_robot_status = setup_network_resource(
                lambda: tbots_cpp.RobotStatusProtoListener(
                    ROBOT_STATUS_PORT, self.__receive_robot_status
                )
            )

        if self.receive_robot_log is None:
            self.receive_robot_log = setup_network_resource(
                lambda: tbots_cpp.RobotLogProtoListener(
                    ROBOT_LOGS_PORT,
                    lambda data: self.__forward_to_proto_unix_io(RobotLog, data),
                )
            )

        should_change_robot_communication_interface = (
            robot_communication_interface
            != self.current_network_config.robot_communication_interface
        )

        if (
            robot_communication_interface == DISCONNECTED
            or not should_change_robot_communication_interface
        ):
            return

        # The following listeners and senders use multicast and are binded to a specific interface
        self.receive_robot_crash = setup_network_resource(
            lambda: tbots_cpp.RobotCrashProtoListener(
                self.multicast_channel,
                ROBOT_CRASH_PORT,
                robot_communication_interface,
                lambda data: self.__forward_to_proto_unix_io(RobotCrash, data),
                True,
            )
        )

        self.robot_ip_listener = setup_network_resource(
            lambda: tbots_cpp.RobotIpNotificationProtoListener(
                self.multicast_channel,
                ROBOT_TO_FULL_SYSTEM_IP_NOTIFICATION_PORT,
                robot_communication_interface,
                self.__update_robot_ip,
                True,
            )
        )

        fullsystem_ip_broadcaster = setup_network_resource(
            lambda: tbots_cpp.FullsystemIpBroadcastProtoUdpSender(
                self.multicast_channel,
                FULL_SYSTEM_TO_ROBOT_IP_NOTIFICATION_PORT,
                robot_communication_interface,
                True,
            )
        )

        local_ip = tbots_cpp.get_local_ip(robot_communication_interface, True)
        if local_ip:
            with self.fullsystem_ip_broadcaster[0]:
                self.fullsystem_ip_broadcaster = (
                    self.fullsystem_ip_broadcaster[0],
                    fullsystem_ip_broadcaster,
                    IpNotification(ip_address=local_ip),
                )

        self.current_network_config.robot_communication_interface = (
            robot_communication_interface if is_setup_successfully else DISCONNECTED
        )

        for robot_id in range(MAX_ROBOT_IDS_PER_SIDE):
            self.__connect_to_robot(robot_id)

    def __update_robot_ip(self, robot_ip_notification: IpNotification) -> None:
        """Given a received discovery message from a robot, update the robot's IP address

        :param robot_ip_notification: IP notification message from a robot with its IP address
        """
        should_reconnect = False
        with self.robot_ip_addresses[robot_ip_notification.robot_id][0]:
            old_ip_address = self.robot_ip_addresses[robot_ip_notification.robot_id][1]
            if (
                old_ip_address is None
                or old_ip_address != robot_ip_notification.ip_address
            ):
                logging.info(
                    f"Robot {robot_ip_notification.robot_id} has IP address {robot_ip_notification.ip_address}"
                )
                self.robot_ip_addresses[robot_ip_notification.robot_id] = (
                    self.robot_ip_addresses[robot_ip_notification.robot_id][0],
                    robot_ip_notification.ip_address,
                )
                should_reconnect = True

        if should_reconnect:
            self.__connect_to_robot(robot_ip_notification.robot_id)

    def poll(self) -> None:
        """Polls and updates the network senders and listeners if a new network configuration is available"""
        # Set up the network on the next tick
        network_config = self.network_config_buffer.get(
            block=False, return_cached=False
        )

        if network_config is not None and self.accept_next_network_config:
            logging.info("Updating network configuration")

            if self.should_setup_full_system:
                self.__setup_full_system(
                    network_config.referee_interface, network_config.vision_interface
                )
            self.__setup_robot_communication(
                network_config.robot_communication_interface
            )
            self.__print_current_network_config()
        elif network_config is not None:
            logger.warning(
                "[RobotCommunication] We received a proto configuration update with a newer network "
                "configuration. We will ignore this update, likely because the interface was provided at "
                "startup. The next update will be accepted."
            )
            self.accept_next_network_config = True
            self.__print_current_network_config()

    def send_primitive(self, robot_id: int, primitive: Primitive) -> None:
        """Send the given primitive to the robot with the given id

        :param robot_id: the id of the robot to send the primitive to
        :param primitive: the primitive to send
        """
        logger.debug(f"Sending primitive to robot {robot_id}")
        with self.primitive_senders[robot_id][0]:
            primitive_sender = self.primitive_senders[robot_id][1]
            if primitive_sender is not None:
                primitive_sender.send_proto(primitive, True)
            else:
                logger.warning(
                    f"Robot {robot_id} is not connected. Unable to send a primitive to it."
                )
