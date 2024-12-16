import software.python_bindings as tbots_cpp
from software.py_constants import MAX_ROBOT_IDS_PER_SIDE

from threading import Lock

DISCONNECTED = "DISCONNECTED"
"""A constant to represent a disconnected interface"""

def WifiCommunicationManager():
    BROADCAST_HZ = 1.0

    def __init__(self, current_proto_unix_io: ProtoUnixIO, interface: Union[str, None] = None):
        ## Robot IP address tracking ##
        self.robot_ip_addresses: List[Tuple[Lock, Union[None, str]]] = (
                [(Lock(), None)] for _ in range(MAX_ROBOT_IDS_PER_SIDE)
        )


        ## Senders and Listeners ##
        self.primitive_senders: List[Tuple[Lock, Union[None, tbots_cpp.PrimitiveSender]]] = (
                [(Lock(), None)] for _ in range(MAX_ROBOT_IDS_PER_SIDE)
        )
        self.receive_robot_status: Union[None, tbots_cpp.RobotStatusProtoListener] = None
        self.receive_robot_log: Union[None, tbots_cpp.RobotLogProtoListener] = None
        self.receive_robot_crash: Union[None, tbots_cpp.RobotCrashProtoListener] = None
        self.robot_ip_listener: Union[None, tbots_cpp.RobotIpNotificationProtoListener] = None 
        self.fullsystem_ip_broadcaster: Tuple[Lock, Union[None, tbots_cpp.FullsystemIpBroadcast], IpNotification] = (
                Lock(), None, IpNotification()
        )

        ## ProtoUnixIO ##
        self.current_proto_unix_io = current_proto_unix_io

        ## Network Configuration ##
        self.network_config_buffer = ThreadSafeBuffer(1, NetworkConfig)
        self.current_proto_unix_io.register_observer(NetworkConfig, self.network_config_buffer)
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
        

        ## Thread Management ##
        self.running = True
        self.broadcast_ip: Union[None, threading.Thread] = None

    def __enter__(self):
        self.broadcast_ip = threading.thread(target=self.__broadcast_fullsystem_ip, daemon=True)


    def __broadcast_fullsystem_ip(self):
        """Notify the robots of this computer's IP address"""
        while self.running:
            time.sleep(1.0 / BROADCAST_HZ)


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

        error = None
        with self.primitive_senders[robot_id][0]:
            primitive_sender = self.primitive_senders[robot_id][1]
            if primitive_sender is None or (
                    primitive_sender.get_ip_address() != ip_address or
                    primitive_sender.get_interface() != self.current_network_config.robot_communication_interface
                    ):
                self.primitive_senders[robot_id][1], error = tbots_cpp.createPrimitiveSender(
                    ip_address,
                    PRIMITIVE_PORT,
                    self.current_network_config.robot_communication_interface,
                    False,
                )

        if error:
            logger.error(f"Error connecting to robot {robot_id}: {error}")
        else:
            logger.info(f"Connected to robot {robot_id} at {ip_address}")
        

    def __forward_to_proto_unix_io(self, type: Type[Message], data: Message) -> None:
        """Forwards to proto unix IO iff running is true

        :param data: the data to be passed through
        :param type: the proto type
        """
        if self.running:
            self.current_proto_unix_io.send_proto(type, data)


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


    def __setup_robot_communication(self, robot_communication_interface: str):
        """
        Set up senders and listeners for communicating with the robots

        :param robot_communication_interface: the interface to listen/send for robot status data. Ignored for sending
        primitives if using radio
        """
        is_setup_successfully = True

        def setup_network_resource(creator: Callable[[], Tuple[Any, str]]) -> Any:
            """Sets up a network node with the given creator function. Logs any errors that occur.
                                                                                                   
            :param creator: the function to create the resource. It must return a type of
            (resource object, error)
            """
            resource, error = creator()
            if error:
                logger.error(f"Error setting up robot status interface:\n{error}")
                is_setup_successfully = False
            return resource

        # Create unicast listeners for RobotStatus and RobotLog. These are all binded to all interfaces
        if self.receive_robot_status is None:
            self.receive_robot_status = setup_network_resource(
                lambda: tbots_cpp.createRobotStatusProtoListener(ROBOT_STATUS_PORT, self.__receive_robot_status))

        if self.receive_robot_log is None:
            self.receive_robot_log = setup_network_resource(
                    lambda: tbots_cpp.createRobotLogProtoListener(ROBOT_LOGS_PORT,
                                                                  lambda data: self.__forward_to_proto_unix_io(
                                                                      RobotLog, data)))

        should_change_robot_communication_interface = (
                robot_communication_interface
                != self.current_network_config.robot_communication_interface
        )

        if (robot_communication_interface == DISCONNECTED or not should_change_robot_communication_interface):
            return

        # The following listeners and senders use multicast and are binded to a specific interface
        self.receive_robot_crash = setup_network_resource(
                lambda: tbots_cpp.createRobotCrashProtoListener(self.multicast_channel,
                                                                ROBOT_CRASH_PORT,
                                                                robot_communication_interface,
                                                                lambda data: self.__forward_to_proto_unix_io(
                                                                    RobotCrash, data),
                                                                True))


        self.robot_ip_listener = setup_network_resource(
                lambda: tbots_cpp.createRobotIpNotificationProtoListener(
                    self.multicast_channel,
                    ROBOT_TO_FULL_SYSTEM_IP_NOTIFICATION_PORT,
                    robot_communication_interface,
                    self.__update_robot_ip,
                    True)

        fullsystem_ip_broadcaster = setup_network_resource(
            lambda: tbots_cpp.createFullsystemIpBroadcast(
                self.multicast_channel,
                FULL_SYSTEM_TO_ROBOT_IP_NOTIFICATION_PORT,
                robot_communication_interface,
                True,
            )

        found_ip, local_ip = tbots_cpp.get_local_ip(robot_communication_interface, True)
        if found_ip:
            with self.fullsystem_ip_broadcaster[0]:
                self.fullsystem_ip_broadcaster[1] = fullsystem_ip_broadcaster
                self.fullsystem_ip_broadcaster[2].ip_address = local_ip

        for robot_id in range(MAX_ROBOT_IDS_PER_SIDE):
            self.__connect_to_robot(robot_id)

        self.current_network_config.robot_communication_interface = (
            robot_communication_interface if is_setup_successfully else DISCONNECTED
        )

    def send_primitive(self, robot_id: int, primitive: Primitive) -> None:
        """Send the given primitive to the robot with the given id

        :param robot_id: the id of the robot to send the primitive to
        :param primitive: the primitive to send
        """
        with self.primitive_senders[robot_id][0]:
            primitive_sender = self.primitive_senders[robot_id][1]
            if primitive_sender is not None:
                primitive_sender.send_proto(primitive, True)
            else:
                logger.warning(f"Robot {robot_id} is not connected. Unable to send a primitive to it.")
