from proto.import_all_protos import *
from embedded_data import EmbeddedData
from google.protobuf.message import Message
from software.embedded.constants.py_constants import get_estop_config, EstopMode
from rich.progress import track
from threading import Thread
import software.python_bindings as tbots_cpp
from software.py_constants import *
import time


class EmbeddedCommunication:
    """Communication class for sending/executing protos with the robots"""

    BROADCAST_HZ = 1.0
    LOCALHOST_IP = "127.0.0.1"

    def __init__(self):
        """Establishes multicast connections and e-stop information"""
        self.primitive_sender: tbots_cpp.PrimitiveSender | None = None
        self.receive_robot_status: tbots_cpp.RobotStatusProtolistener | None = None
        self.embedded_data = EmbeddedData()
        self.channel_id = int(self.embedded_data.get_channel_id())
        self.robot_id = int(self.embedded_data.get_robot_id())

        self.sequence_number = 0
        self.command_duration_seconds = 2.0
        self.send_primitive_interval_s = 0.01

        # total primitives for this robots
        self.robot_primitives = {}
        self.running = True
        # Set the primitive of the local robot
        self.robot_primitives[self.robot_id] = Primitive(stop=StopPrimitive())

        # Localhost IP Broadcaster
        self.fullsystem_ip_broadcaster = tbots_cpp.FullsystemIpBroadcastProtoUdpSender(
            getRobotMulticastChannel(int(self.embedded_data.get_channel_id())),
            FULL_SYSTEM_TO_ROBOT_IP_NOTIFICATION_PORT,
            self.embedded_data.get_network_interface(),
            True,
        )

        # initialising the estop
        self.estop_reader = None
        self.should_send_stop = False
        self.estop_mode, self.estop_path = get_estop_config(
            keyboard_estop=False, disable_communication=False
        )

        # Always in PHYSICAL_ESTOP mode within CLI
        try:
            self.estop_reader = tbots_cpp.ThreadedEstopReader(self.estop_path, 115200)
            self.__update_estop_state()
        except Exception as e:
            raise Exception(f"Invalid Estop found at location {self.estop_path} as {e}")

    def __receive_robot_status(self, robot_status: Message) -> None:
        """Updates the dynamic information with the new RobotStatus message.
        :param robot_status: The incoming RobotStatus proto from the robot to read
        """
        self.embedded_data.epoch_timestamp_seconds = (
            robot_status.time_sent.epoch_timestamp_seconds
        )
        self.embedded_data.battery_voltage = robot_status.power_status.battery_voltage
        self.embedded_data.primitive_packet_loss_percentage = (
            robot_status.network_status.primitive_packet_loss_percentage
        )
        self.embedded_data.primitive_executor_step_time_ms = (
            robot_status.thunderloop_status.primitive_executor_step_time_ms
        )

    def __should_send_packet(self) -> bool:
        """Returns True if a proto should be sent
        :return: boolean
        """
        return (
            self.estop_mode != EstopMode.DISABLE_ESTOP
        ) and not self.should_send_stop

    def send_primitive(self, primitive: Primitive) -> None:
        """Forward PrimitiveSet protos from diagnostics to the robots."""
        primitive.sequence_number = self.sequence_number
        primitive.time_sent.epoch_timestamp_seconds = time.time()
        self.sequence_number += 1

        if self.__should_send_packet() or self.should_send_stop:
            self.primitive_sender.send_proto(primitive, True)
            self.should_send_stop = False

    def __update_estop_state(self) -> None:
        """Synchronously updates the current estop status proto. Always in physical estop mode."""
        if self.estop_mode == EstopMode.DISABLE_ESTOP:
            self.should_send_stop = False
        elif self.estop_mode == EstopMode.PHYSICAL_ESTOP:
            self.should_send_stop = not self.estop_reader.isEstopPlay()
        else:
            self.should_send_stop = True

    def run_primitive(self, diagnostics_primitive: Primitive) -> None:
        """Wrapper class that forwards PrimitiveSet protos from diagnostics to the robots.

        If the emergency stop is tripped, the PrimitiveSet will not be sent so
        that the robots timeout and stop.
        """
        self.__update_estop_state()
        if self.should_send_stop:
            raise KeyboardInterrupt
        else:
            self.send_primitive(diagnostics_primitive)

    def run_primitive_over_time(
        self, duration_seconds: float, primitive_set: Primitive, description: str
    ) -> None:
        """Executes a primitive set synchronously for the specified amount of time in the console.
        :param duration_seconds: Duration to execute in seconds
        :param primitive_set: Primitive set to execute
        :param description: The UI description to display in the console
        """
        for _ in track(
            range(int(duration_seconds / self.send_primitive_interval_s)),
            description=description,
        ):
            self.run_primitive(primitive_set)
            time.sleep(self.send_primitive_interval_s)

    def __broadcast_fullsystem_ip(self) -> None:
        while self.running:
            self.fullsystem_ip_broadcaster.send_proto(
                IpNotification(ip_address=EmbeddedCommunication.LOCALHOST_IP), True
            )
            time.sleep(1.0 / EmbeddedCommunication.BROADCAST_HZ)

    def __enter__(self) -> EmbeddedData:
        """Enter RobotDiagnosticsCLI context manager. Setup multicast listeners
        for RobotStatus msgs, and multicast sender for PrimitiveSet
        """
        # Broadcast IP
        self.broadcast_ip = Thread(target=self.__broadcast_fullsystem_ip, daemon=True)
        self.broadcast_ip.start()

        # Unicast Receiver
        self.receive_robot_status = tbots_cpp.RobotStatusProtoListener(
            ROBOT_STATUS_PORT, self.__receive_robot_status
        )

        # Unicast Sender
        self.primitive_sender = tbots_cpp.PrimitiveProtoUdpSender(
            EmbeddedCommunication.LOCALHOST_IP,
            PRIMITIVE_PORT,
            self.embedded_data.get_network_interface(),
            False,
        )
        return self

    def __exit__(self, type, value, traceback) -> None:
        """Exit RobotDiagnosticsCLI context manager
        Ends all currently running loops and joins all currently active threads
        """
        self.running = False
        self.receive_robot_status.close()
        self.broadcast_ip.join()
