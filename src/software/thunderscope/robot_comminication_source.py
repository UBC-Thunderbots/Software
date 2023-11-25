from enum import IntEnum

from software.thunderscope.proto_unix_io import ProtoUnixIO


class ControlMode(IntEnum):
    """
    Enum for the 3 modes representing the source of input controls from the robot
    """

    DIAGNOSTICS = 0
    XBOX = 1
    NONE = 2


class DiagnosticsControlManager:
    """
    Sets up a manager that handles the control input source for each robot,
    and forwards that to communication
    """
    def __init__(
            self,
    ):
        self.diagnostics_input_widget_signal = 0
        self.robot_id_input_source_map: set[(int, ControlMode)] = set()
        self.robots_connected_to_fullsystem = set()


    def get_fullsystem_robots_ids(self):
        return
    def get_xbox_robot_ids(self):
        return
    def get_diagnostic_robots_ids(self):
        return
    def connect_control_mode_signal(self, signal):
        return

    def connect_control_mode_signal(self, signal):
        return

