import os
from typing import Callable

from evdev import util

from software.thunderscope.constants import IndividualRobotMode, HANDHELD_PATH
from software.thunderscope.controller_diagnostics import ControllerDiagnostics
from software.thunderscope.robot_diagnostics.diagnostics_input_widget import ControlMode


class RobotInputControlManager:
    """
    Sets up a manager that handles the control input source for each robot,
    and forwards that to communication

    @:param xbox_enabled: If xbox control is enabled
    """

    def __init__(
        self,
        controller_device_path: str,
        perform_move: Callable[[int, int, int, int], None],
        perform_kick: Callable[[int], None],
        perform_chip: Callable[[int], None],
    ):
        self.__perform_move = perform_move
        self.__perform_kick = perform_kick
        self.__perform_chip = perform_chip
        self.__input_mode = ControlMode.DIAGNOSTICS
        self.__controller_diagnostics = None

        self.setup_controller(controller_device_path)

        self.__robot_id_input_source_map: set[(int, IndividualRobotMode)] = set()

        self.__robots_to_be_disconnected: set[int] = {}

    def setup_controller(self, controller_device_path: str):
        if controller_device_path:
            if os.path.exists(controller_device_path) and util.is_device(
                controller_device_path
            ):
                self.__input_mode = ControlMode.XBOX
                self.__controller_diagnostics = ControllerDiagnostics(
                    controller_device_path,
                    self.__perform_move,
                    self.__perform_kick,
                    self.__perform_chip,
                )
            else:
                self.__input_mode = ControlMode.DIAGNOSTICS

    def set_full_system_control(self):
        self.__robot_id_input_source_map = set(
            map(
                lambda robot_control: (robot_control[0], IndividualRobotMode.AI),
                self.__robot_id_input_source_map,
            )
        )

    def get_diagnostics_input_mode(self) -> ControlMode:
        """
        Get the diagnostics input mode
        :return:
        """
        return self.__input_mode

    def toggle_input_mode(self, mode: ControlMode):
        """
        Process and Qt signal indicating the diagnostics control mode has changed
        :param mode:
        :return:
        """
        if mode == ControlMode.XBOX and self.__controller_diagnostics is not None:
            self.__input_mode = mode
        else:
            self.__input_mode = ControlMode.DIAGNOSTICS

    def toggle_control_mode_for_robot(self, robot_id: int, mode: IndividualRobotMode):
        """
        Updates the control mode for the given robot id to the given mode
        :param robot_id: The robot id to update
        :param mode: The input mode this robot should use - one of AI, MANUAL, or NONE
        """
        self.__robots_to_be_disconnected[robot_id] = NUM_TIMES_SEND_STOP
        self.__robot_id_input_source_map = set(
            map(
                # TODO figure way to destruct params or make callback maybe
                lambda robot_control: (robot_id, mode)
                if robot_id == robot_control[0]
                else robot_control,
                self.__robot_id_input_source_map,
            )
        )
