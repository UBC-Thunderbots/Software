import software.python_bindings as tbots_cpp
from software.py_constants import MAX_ROBOT_IDS_PER_SIDE

DISCONNECTED = "DISCONNECTED"
"""A constant to represent a disconnected interface"""

def WifiCommunicationManager():
    def __init__(self, current_proto_unix_io: ProtoUnixIO, interface: Union[str, None] = None):
        ## Robot IP address tracking ##
        self.robot_ip_addresses: List[Union[None, str]] = [None] * MAX_ROBOT_IDS_PER_SIDE


        ## Senders and Listeners ##
        self.primitive_senders: List[Union[None, tbots_cpp.PrimitiveSender]] = [None] * MAX_ROBOT_IDS_PER_SIDE
        self.receive_robot_status: Union[None, tbots_cpp.RobotStatusProtoListener] = None
        

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
        

        ## Threads ##
        self.running = True
        self.broadcast_ip = threading.thread(target=self.__broadcast_fullsystem_ip, daemon=True)


    def __broadcast_fullsystem_ip(self):
        """Notify the robots of this computer's IP address"""
        pass


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
                                                                                                   
            :param creator: the function to create the listener. It must return a type of
            (listener object, error)
            """
            listener, error = creator()
            if error:
                logger.error(f"Error setting up robot status interface:\n{error}")
                is_setup_successfully = False
            return listener

        # Create unicast listeners for RobotStatus, RobotLog and RobotCrash. These are all binded to all interfaces
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

        self.fullsystem_ip_broadcaster = setup_network_resource(
            lambda: tbots_cpp.createFullsystemIpBroadcast(
                self.multicast_channel,
                FULL_SYSTEM_TO_ROBOT_IP_NOTIFICATION_PORT,
                robot_communication_interface,
                True,
            )

        for robot_ip in self.robot_ids:
            if robot_ip is not None:
                self.__connect_to_robot(robot_id)

                if self.primitive_sender[robot_id] is None:
                    self.is_setup_successfully = False

        self.current_network_config.robot_communication_interface = (
            robot_communication_interface if is_setup_successfully else DISCONNECTED
        )

