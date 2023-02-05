import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from software.py_constants import *
import software.thunderscope.common.common_widgets as common_widgets
from proto.import_all_protos import *
from software.thunderscope.robot_diagnostics.robot_info import RobotInfo

from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class RobotView(QWidget):
    """Class to show a snapshot of the robot's current state.

    Displays the vision pattern, capacitor/battery voltages,
    and other information about the robot state.

    """

    PINK = QtGui.QColor(255, 0, 255)
    GREEN = QtGui.QColor(0, 255, 0)

    # There is no pattern to this so we just have to create
    # mapping from robot id to the four corners of the vision pattern
    #
    # robot-id: top-right, top-left, bottom-left, bottom-right
    #
    # https://robocup-ssl.github.io/ssl-rules/sslrules.html
    VISION_PATTERN_LOOKUP = {
        0: [PINK, PINK, GREEN, PINK],
        1: [PINK, GREEN, GREEN, PINK],
        2: [GREEN, GREEN, GREEN, PINK],
        3: [GREEN, PINK, GREEN, PINK],
        4: [PINK, PINK, PINK, GREEN],
        5: [PINK, GREEN, PINK, GREEN],
        6: [GREEN, GREEN, PINK, GREEN],
        7: [GREEN, PINK, PINK, GREEN],
        8: [GREEN, GREEN, GREEN, GREEN],
        9: [PINK, PINK, PINK, PINK],
        10: [PINK, PINK, GREEN, GREEN],
        11: [GREEN, GREEN, PINK, PINK],
        12: [PINK, GREEN, GREEN, GREEN],
        13: [PINK, GREEN, PINK, PINK],
        14: [GREEN, PINK, GREEN, GREEN],
        15: [GREEN, PINK, PINK, PINK],
    }

    ERROR_CODE_MESSAGES = {
        ErrorCode.LOW_CAP: "Low Cap",
        ErrorCode.LOW_BATTERY: "Low Battery",
        ErrorCode.HIGH_BOARD_TEMP: "High Board Temp",
        ErrorCode.DRIBBLER_MOTOR_HOT: "Dribbler Motor Hot",
    }

    toggle_robot_connection_signal = QtCore.pyqtSignal(int)

    def __init__(self, load_fullsystem):

        """Initialize the robot view."""

        super().__init__()

        self.robot_status_buffer = ThreadSafeBuffer(100, RobotStatus)

        self.layout = QVBoxLayout()

        self.robot_info_widgets = [RobotInfo(x, load_fullsystem) for x in range(MAX_ROBOT_IDS_PER_SIDE)]

        for id in range(MAX_ROBOT_IDS_PER_SIDE):
            self.layout.addLayout(self.robot_info_widgets[id])

        self.setLayout(self.layout)

    def refresh(self):
        """Refresh the view
        """
        # TODO (#2791): fix robot view refresh function

        robot_status = self.robot_status_buffer.get(block=False)

        self.robot_info_widgets[robot_status.robot_id].update(
            robot_status.power_status, robot_status.error_code
        )
