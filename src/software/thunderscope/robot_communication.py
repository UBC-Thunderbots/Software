from typing import Self

import threading
import time
import os
import software.python_bindings as tbots_cpp

from proto.import_all_protos import *
from software.logger.logger import create_logger
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.wifi_communication_manager import WifiCommunicationManager
from software.py_constants import *
from software.thunderscope.constants import (
    ROBOT_COMMUNICATIONS_TIMEOUT_S,
    IndividualRobotMode,
    EstopMode,
)

logger = create_logger(__name__)


class RobotCommunication:
    """Communicate with the robots"""

    def __init__(
        self,
        current_proto_unix_io: ProtoUnixIO,
        estop_mode: EstopMode,
        communication_manager: WifiCommunicationManager,
        estop_path: os.PathLike = None,
        estop_baudrate: int = 115200,
    ):
        """Initialize the communication with the robots

        :param current_proto_unix_io: the current proto unix io object
        :param estop_mode: what estop mode we are running right now, of type EstopMode
        :param communication_manager: Handles WiFi communication to the robots
        :param estop_path: The path to the estop
        :param estop_baudrate: The baudrate of the estop
        """
        self.sequence_number = 0
        self.current_proto_unix_io = current_proto_unix_io

        self.communication_manager = communication_manager

        self.running = False

        self.fullsystem_primitive_set_buffer = ThreadSafeBuffer(1, PrimitiveSet)

        self.motor_control_primitive_buffer = ThreadSafeBuffer(1, MotorControl)
        self.power_control_primitive_buffer = ThreadSafeBuffer(1, PowerControl)

        # dynamic map of robot id to the individual control mode
        self.robot_control_mode_map: dict[int, IndividualRobotMode] = {}

        # links robot id to the number of times to send a stop primitive
        self.robot_stop_primitive_send_count: list[int] = [0] * MAX_ROBOT_IDS_PER_SIDE

        self.current_proto_unix_io.register_observer(
            PrimitiveSet, self.fullsystem_primitive_set_buffer
        )

        self.current_proto_unix_io.register_observer(
            MotorControl, self.motor_control_primitive_buffer
        )

        self.current_proto_unix_io.register_observer(
            PowerControl, self.power_control_primitive_buffer
        )

        self.send_estop_state_thread = threading.Thread(
            target=self.__send_estop_state, daemon=True
        )
        self.run_primitive_set_thread = threading.Thread(
            target=self.__run_primitive_set, daemon=True
        )

        # load control mode map with default values
        for robot_id in range(MAX_ROBOT_IDS_PER_SIDE):
            self.robot_control_mode_map[robot_id] = IndividualRobotMode.NONE

        # TODO: (#3174): move estop state management out of robot_communication
        self.estop_mode = estop_mode
        self.estop_path = estop_path
        self.estop_baudrate = estop_baudrate

        # initialising the estop
        # tries to access a plugged in estop. if not found, throws an exception
        # if using keyboard estop, skips this step
        self.estop_reader = None
        self.estop_is_playing = False
        self.prev_estop_is_playing = False

        # only checks for estop if we are in physical estop mode
        if self.estop_mode == EstopMode.PHYSICAL_ESTOP:
            try:
                self.estop_reader = tbots_cpp.ThreadedEstopReader(
                    self.estop_path, self.estop_baudrate
                )
            except Exception:
                raise Exception(f"Invalid Estop found at location {self.estop_path}")

        self.robots_connected_to_fullsystem = {
            robot_id for robot_id in range(MAX_ROBOT_IDS_PER_SIDE)
        }

    def toggle_keyboard_estop(self) -> None:
        """Toggle the keyboard estop state, queue stop primitives for all robots,
        and log the new state. No-op unless in keyboard estop mode.
        """
        if self.estop_mode == EstopMode.KEYBOARD_ESTOP:
            self.prev_estop_is_playing = self.estop_is_playing
            self.estop_is_playing = not self.estop_is_playing
            logger.debug(
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
        """Change a robot's input mode (NONE, MANUAL, or AI).

        Switching to NONE queues stop primitives for the robot so it stops
        instead of acting on cached primitives.

        :param robot_id: the id of the robot whose mode we're changing
        :param mode: the mode of input for this robot's primitives
        """
        self.robot_control_mode_map[robot_id] = mode
        self.robot_stop_primitive_send_count[robot_id] = (
            NUM_TIMES_SEND_STOP if mode == IndividualRobotMode.NONE else 0
        )

    def __queue_stop_for_all_robots(self) -> None:
        """Queue NUM_TIMES_SEND_STOP stop primitives for every robot."""
        self.robot_stop_primitive_send_count = [
            NUM_TIMES_SEND_STOP for _ in range(MAX_ROBOT_IDS_PER_SIDE)
        ]

    def __send_estop_state(self) -> None:
        """Continuously broadcast the estop state, refreshing the physical estop
        reading each iteration. On a play->stop transition, queue stop primitives
        for all robots. No-op when estop is disabled.
        """
        if self.estop_mode == EstopMode.DISABLE_ESTOP:
            return

        while True:
            if self.estop_mode == EstopMode.PHYSICAL_ESTOP:
                self.prev_estop_is_playing = self.estop_is_playing
                self.estop_is_playing = self.estop_reader.isEstopPlay()

            if self.prev_estop_is_playing and not self.estop_is_playing:
                self.__queue_stop_for_all_robots()

            self.current_proto_unix_io.send_proto(
                EstopState, EstopState(is_playing=self.estop_is_playing)
            )
            time.sleep(0.1)

    def __should_send_packet(self, robot_id) -> bool:
        """Returns True if the proto should be sent to the robot with the given id

        :param robot_id: the id of the robot to potentially send the proto to

        :return: boolean
        """
        return (
            self.estop_mode != EstopMode.DISABLE_ESTOP
            and self.estop_is_playing
            and self.robot_control_mode_map[robot_id] != IndividualRobotMode.NONE
        )

    def __run_primitive_set(self) -> None:
        """Forward AI primitives (PrimitiveSet) and diagnostics MotorControl/
        PowerControl protos to the robots.

        Blocks up to ROBOT_COMMUNICATIONS_TIMEOUT_S for an AI proto, falling back
        to the cached one; diagnostics protos are read non-blocking.

        While the estop is tripped, regular primitives are withheld and a stop
        primitive is sent to every robot instead so they stop immediately.
        """
        while self.running:
            self.communication_manager.poll()

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

            # force a stop primitive to estopped/uncontrolled robots so they
            # don't keep acting on their last primitive
            force_stop_robot_ids = set()
            for robot_id, num_times_to_send_stop in enumerate(
                self.robot_stop_primitive_send_count
            ):
                if num_times_to_send_stop > 0:
                    robot_primitives_map[robot_id] = Primitive(stop=StopPrimitive())
                    self.robot_stop_primitive_send_count[robot_id] = (
                        num_times_to_send_stop - 1
                    )
                    force_stop_robot_ids.add(robot_id)

            for robot_id, primitive in robot_primitives_map.items():
                if (
                    robot_id not in force_stop_robot_ids
                    and not self.__should_send_packet(robot_id=robot_id)
                ):
                    continue
                primitive.sequence_number = self.sequence_number
                primitive.time_sent.CopyFrom(
                    Timestamp(epoch_timestamp_seconds=time.time())
                )
                self.communication_manager.send_primitive(
                    robot_id=robot_id, primitive=primitive
                )

            self.sequence_number += 1

            # sleep if not running fullsystem
            if IndividualRobotMode.AI not in self.robot_control_mode_map.values():
                time.sleep(ROBOT_COMMUNICATIONS_TIMEOUT_S)

    def __enter__(self) -> Self:
        """Enter RobotCommunication context manager. Setup multicast listeners
        for RobotStatus, RobotLogs, and RobotCrash msgs, and multicast sender for PrimitiveSet
        """
        self.running = True

        self.send_estop_state_thread.start()
        self.run_primitive_set_thread.start()

        return self

    def __exit__(self, type, value, traceback) -> None:
        """Exit RobotCommunication context manager

        Ends all currently running loops and joins all currently active threads
        """
        self.running = False

        self.run_primitive_set_thread.join()
