from proto.import_all_protos import *
from embedded_data import EmbeddedData
from google.protobuf.message import Message
from software.embedded.constants.py_constants import (get_estop_config, EstopMode)
from rich.progress import track
import software.python_bindings as tbots_cpp
from software.py_constants import *
import time

class EmbeddedCommunication:
    def __init__(self):
        self.send_primitive_set = None
        self.receive_robot_status = None
        self.embedded_data = EmbeddedData()
        self.channel_id = self.embedded_data.get_robot_id()
        self.robot_id = self.embedded_data.get_robot_id()

        self.sequence_number = 0
        self.command_duration_seconds = 2.0
        self.send_primitive_interval_s = 0.01

        # total primitives for all robots
        self.robot_primitives = {}

        # for all robots connected to diagnostics, set their primitive
        self.robot_primitives[self.robot_id] = Primitive(stop=StopPrimitive())

        # initialising the estop
        self.estop_reader = None
        self.estop_is_playing = False
        self.should_send_stop = False
        self.estop_mode, self.estop_path = (
            get_estop_config(keyboard_estop=False, disable_communication=False))

        # only checks for estop if we are in physical estop mode
        if self.estop_mode == EstopMode.PHYSICAL_ESTOP:
            try:
                self.estop_reader = tbots_cpp.ThreadedEstopReader(
                    self.estop_path, 115200
                )
                self.__update_estop_state()
            except Exception as e:
                raise Exception(f"Invalid Estop found at location {self.estop_path} as {e}")

    def __receive_robot_status(self, robot_status: Message) -> None:
        """Forwards the given robot status to the full system along with the round-trip time

        :param robot_status: RobotStatus to extract information from
        """
        self.epoch_timestamp_seconds = robot_status.time_sent.epoch_timestamp_seconds
        self.battery_voltage = robot_status.power_status.battery_voltage
        self.primitive_packet_loss_percentage = robot_status.network_status.primitive_packet_loss_percentage
        self.primitive_executor_step_time_ms = robot_status.thunderloop_status.primitive_executor_step_time_ms

    def __should_send_packet(self) -> bool:
        """Returns True if should send a proto

        :return: boolean
        """
        return ((self.estop_mode != EstopMode.DISABLE_ESTOP) and self.estop_is_playing)

    def __send_primitive_set(self, primitive: Primitive) -> None:
        """Forward PrimitiveSet protos from diagnostics to the robots."""
        self.robot_primitives[self.robot_id] = primitive
        # initialize total primitive set and send it
        primitive_set = PrimitiveSet(
            time_sent=Timestamp(epoch_timestamp_seconds=time.time()),
            stay_away_from_ball=False,
            robot_primitives=(
                self.robot_primitives
                if not self.should_send_stop
                else {
                    robot_id: Primitive(stop=StopPrimitive())
                    for robot_id in self.robot_primitives.keys()
                }
            ),
            sequence_number=self.sequence_number,
        )

        self.sequence_number += 1

        if self.__should_send_packet() or self.should_send_stop:
            self.send_primitive_set.send_proto(primitive_set)
            self.should_send_stop = False

    def __update_estop_state(self) -> None:
        """Updates the current estop status proto if estop is not disabled
        Always in physical estop mode, uses the physical estop value
        """
        if self.estop_mode == EstopMode.DISABLE_ESTOP:
            self.should_send_stop = False
        elif self.estop_mode == EstopMode.PHYSICAL_ESTOP:
            self.should_send_stop = not self.estop_reader.isEstopPlay()
        else:
            self.should_send_stop = True

    def run_primitive_set(self, diagnostics_primitive: Primitive) -> None:
        """Forward PrimitiveSet protos from diagnostics to the robots.

        Updates/polls the Estop state

        If the emergency stop is tripped, the PrimitiveSet will not be sent so
        that the robots timeout and stop.
        """
        self.__update_estop_state()
        if self.should_send_stop:
            raise KeyboardInterrupt
        else:
            self.__send_primitive_set(diagnostics_primitive)

    def run_primitive_over_time(self, duration_seconds: float, primitive_set: Primitive, description: str):
        """Executes a primitive set synchronously for the specified amount of time in the console.
        :param duration_seconds: Duration to execute in seconds
        :param primitive_set: Primitive set to execute
        :param description: The UI description to display in the console
        """
        for _ in track(range(int(duration_seconds / self.send_primitive_interval_s)),
                       description=description):
            self.run_primitive_set(primitive_set)
            time.sleep(self.send_primitive_interval_s)

    def __enter__(self):
        """Enter RobotDiagnosticsCLI context manager. Setup multicast listeners
        for RobotStatus, RobotLogs, and RobotCrash msgs, and multicast sender for PrimitiveSet
        """
        # Receiver
        self.receive_robot_status = tbots_cpp.RobotStatusProtoListener(
            str(getRobotMulticastChannel(
                self.channel_id)) + "%" + f"{self.embedded_data.get_network_interface()}",
            ROBOT_STATUS_PORT,
            self.__receive_robot_status,
            True,
            )

        # Sender
        self.send_primitive_set = tbots_cpp.PrimitiveSetProtoUdpSender(
            str(getRobotMulticastChannel(
                self.channel_id)) + "%" + f"{self.embedded_data.get_network_interface()}", PRIMITIVE_PORT, True
        )

        return self

    def __exit__(self, type, value, traceback) -> None:
        """Exit RobotDiagnosticsCLI context manager
        Ends all currently running loops and joins all currently active threads
        """
        self.receive_robot_status.close()