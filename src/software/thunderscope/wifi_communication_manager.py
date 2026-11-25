import logging
import statistics
import time
from collections import deque
from threading import Lock, Thread
from typing import Any, Callable, Self, Type

import proto.import_all_protos as protos
import software.python_bindings as tbots_cpp
from colorama import Fore, Style
from google.protobuf.message import Message
from software.logger.logger import create_logger
from software.py_constants import (
    FULL_SYSTEM_TO_ROBOT_IP_NOTIFICATION_PORT,
    MAX_ROBOT_IDS_PER_SIDE,
    PRIMITIVE_PORT,
    ROBOT_CRASH_PORT,
    ROBOT_LOGS_PORT,
    ROBOT_STATUS_PORT,
    ROBOT_TO_FULL_SYSTEM_IP_NOTIFICATION_PORT,
    SSL_REFEREE_ADDRESS,
    SSL_REFEREE_PORT,
    SSL_VISION_ADDRESS,
    SSL_VISION_PORT,
)
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

DISCONNECTED = "DISCONNECTED"
"""A constant to represent a disconnected interface"""

logger = create_logger(name=__name__, log_level=logging.DEBUG)


class RollingMin:
    """Thread-safe minimum of the samples added within a sliding time window."""

    def __init__(self, window_s: float):
        """Creates an empty rolling minimum

        :param window_s: how long, in seconds, a sample is kept for
        """
        self.window_s = window_s
        # (time added, value) with strictly increasing values, so the front is the min
        self.samples: deque[tuple[float, float]] = deque()
        self.lock = Lock()

    def add(self, value: float, now: float) -> None:
        """Adds a sample

        :param value: the sample value
        :param now: the current time in seconds
        """
        with self.lock:
            while self.samples and self.samples[-1][1] >= value:
                self.samples.pop()
            self.samples.append((now, value))
            self.__expire(now)

    def get(self, now: float) -> float | None:
        """Returns the minimum of the samples in the window, or None if there are none

        :param now: the current time in seconds
        """
        with self.lock:
            self.__expire(now)
            return self.samples[0][1] if self.samples else None

    def clear(self) -> None:
        """Removes all samples"""
        with self.lock:
            self.samples.clear()

    def __expire(self, now: float) -> None:
        while self.samples and self.samples[0][0] < now - self.window_s:
            self.samples.popleft()


class WifiCommunicationManager:
    """Manages WiFi communication between different modules of the system."""

    BROADCAST_HZ = 1.0
    """The frequency at which to broadcast the full system IP address"""

    VISION_CLOCK_OFFSET_WINDOW_S = 5.0
    """How long to keep vision clock offset samples. The minimum over this window
    approximates the offset between the SSL Vision clock and this computer's clock"""

    VISION_CLOCK_JUMP_THRESHOLD_S = 1.0
    """If a new vision clock offset sample differs from the current estimate by more
    than this, the vision clock was reset and the old samples are discarded"""

    MAX_VALID_VISION_AGE_S = 0.5
    """Vision ages outside (0, this] are treated as invalid and not sent"""

    RTT_SAMPLE_COUNT = 20
    """Number of recent round-trip time samples per robot to take the median of"""

    RTT_STALE_TIMEOUT_S = 2.0
    """Round-trip time estimates older than this are not sent"""

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
            Lock, tbots_cpp.FullsystemIpBroadcast | None, protos.IpNotification
        ] = (Lock(), None, protos.IpNotification())

        ## Latency estimation ##
        # Offset from the SSL Vision clock to this computer's clock
        self.vision_clock_offset = RollingMin(self.VISION_CLOCK_OFFSET_WINDOW_S)
        # Per robot: (lock, recent round-trip times, time of the latest sample)
        self.robot_rtts: list[tuple[Lock, deque[float], list[float]]] = [
            (Lock(), deque(maxlen=self.RTT_SAMPLE_COUNT), [0.0])
            for _ in range(MAX_ROBOT_IDS_PER_SIDE)
        ]

        ## ProtoUnixIO ##
        self.current_proto_unix_io = current_proto_unix_io

        ## Network Configuration ##
        self.multicast_channel = multicast_channel
        self.network_config_buffer = ThreadSafeBuffer(1, protos.NetworkConfig)
        self.current_proto_unix_io.register_observer(
            protos.NetworkConfig, self.network_config_buffer
        )
        # Whether to accept the next configuration update. We will be provided a proto configuration from the
        # ProtoConfigurationWidget. If the user provides an interface, we will accept it as the first network
        # configuration and ignore the provided one from the widget. If not, we will wait for this first configuration
        self.accept_next_network_config = True
        self.current_network_config = protos.NetworkConfig(
            robot_communication_interface=DISCONNECTED,
            referee_interface=DISCONNECTED,
            vision_interface=DISCONNECTED,
        )

        ## Thread Management ##
        self.running = True
        self.broadcast_ip: Thread | None = None

        if interface:
            self.accept_next_network_config = False
            self.__setup_robot_communication(interface)
        self.referee_port = referee_port

        self.should_setup_full_system = should_setup_full_system
        if interface and should_setup_full_system:
            self.__setup_full_system(
                referee_interface=interface, vision_interface=interface
            )

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
                except tbots_cpp.TbotsNetworkException as error:
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
        now = time.time()
        round_trip_time_seconds = now - (
            robot_status.adjusted_time_sent.epoch_timestamp_seconds
        )
        if (
            robot_status.HasField("adjusted_time_sent")
            and 0 <= robot_status.robot_id < MAX_ROBOT_IDS_PER_SIDE
            and 0.0 < round_trip_time_seconds < self.MAX_VALID_VISION_AGE_S
        ):
            lock, rtts, last_sample_time = self.robot_rtts[robot_status.robot_id]
            with lock:
                rtts.append(round_trip_time_seconds)
                last_sample_time[0] = now
        self.__forward_to_proto_unix_io(
            protos.RobotStatistic,
            protos.RobotStatistic(
                robot_id=robot_status.robot_id,
                round_trip_time_seconds=round_trip_time_seconds,
            ),
        )
        self.__forward_to_proto_unix_io(protos.RobotStatus, robot_status)

    def __receive_ssl_wrapper(self, ssl_wrapper: Message) -> None:
        """Updates the vision clock offset estimate and forwards the given SSL wrapper
        packet to the full system

        :param ssl_wrapper: SSL_WrapperPacket received from SSL Vision
        """
        if ssl_wrapper.HasField("detection"):
            now = time.time()
            # Local receive time minus vision send time is the clock offset plus the
            # network delay, so the minimum over a window approximates the offset
            offset_sample = now - ssl_wrapper.detection.t_sent
            current_offset = self.vision_clock_offset.get(now)
            if (
                current_offset is not None
                and abs(offset_sample - current_offset)
                > self.VISION_CLOCK_JUMP_THRESHOLD_S
            ):
                self.vision_clock_offset.clear()
            self.vision_clock_offset.add(offset_sample, now)

        self.__forward_to_proto_unix_io(protos.SSL_WrapperPacket, ssl_wrapper)

    def __stamp_latency_estimates(
        self, robot_id: int, primitive: protos.Primitive
    ) -> None:
        """Fills in the vision age and network latency estimates of the given primitive,
        right before it is sent. Estimates that are unknown are set to 0.

        :param robot_id: the id of the robot the primitive is sent to
        :param primitive: the primitive to update
        """
        now = time.time()

        primitive.vision_age_at_send_s = 0.0
        vision_clock_offset = self.vision_clock_offset.get(now)
        if primitive.vision_capture_time_s > 0.0 and vision_clock_offset is not None:
            vision_age_s = now - (primitive.vision_capture_time_s + vision_clock_offset)
            if 0.0 < vision_age_s <= self.MAX_VALID_VISION_AGE_S:
                primitive.vision_age_at_send_s = vision_age_s

        primitive.estimated_one_way_latency_s = 0.0
        lock, rtts, last_sample_time = self.robot_rtts[robot_id]
        with lock:
            if rtts and now - last_sample_time[0] < self.RTT_STALE_TIMEOUT_S:
                primitive.estimated_one_way_latency_s = statistics.median(rtts) / 2

    def __setup_full_system(
        self, referee_interface: str, vision_interface: str
    ) -> None:
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
                    lambda data: self.__forward_to_proto_unix_io(protos.Referee, data),
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
                    self.__receive_ssl_wrapper,
                    True,
                )
                self.current_network_config.vision_interface = vision_interface
            except tbots_cpp.TbotsNetworkException as e:
                logger.error(f"Error setting up vision interface:\n{e}")
                self.current_network_config.vision_interface = DISCONNECTED

    def __setup_robot_communication(self, robot_communication_interface: str) -> None:
        """Set up senders and listeners for communicating with the robots

        :param robot_communication_interface: the interface to listen/send for robot status data
        """
        is_setup_successfully = True

        def setup_network_resource(creator: Callable[[], tuple[Any, str]]) -> Any:
            """Sets up a network node with the given creator function. Logs any errors that occur.

            :param creator: the function to create the resource.
            """
            try:
                resource = creator()
            except tbots_cpp.TbotsNetworkException as error:
                logger.error(f"Error setting up robot status interface:\n{error}")
                return None
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
                    lambda data: self.__forward_to_proto_unix_io(protos.RobotLog, data),
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
                lambda data: self.__forward_to_proto_unix_io(protos.RobotCrash, data),
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
                    protos.IpNotification(ip_address=local_ip),
                )

        self.current_network_config.robot_communication_interface = (
            robot_communication_interface if is_setup_successfully else DISCONNECTED
        )

        for robot_id in range(MAX_ROBOT_IDS_PER_SIDE):
            self.__connect_to_robot(robot_id)

    def __update_robot_ip(self, robot_ip_notification: protos.IpNotification) -> None:
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

    def send_primitive(self, robot_id: int, primitive: protos.Primitive) -> None:
        """Send the given primitive to the robot with the given id

        :param robot_id: the id of the robot to send the primitive to
        :param primitive: the primitive to send
        """
        logger.debug(f"Sending primitive to robot {robot_id}")
        self.__stamp_latency_estimates(robot_id, primitive)
        with self.primitive_senders[robot_id][0]:
            primitive_sender = self.primitive_senders[robot_id][1]
            if primitive_sender is not None:
                primitive_sender.send_proto(primitive, True)
            else:
                logger.warning(
                    f"Robot {robot_id} is not connected. Unable to send a primitive to it."
                )
