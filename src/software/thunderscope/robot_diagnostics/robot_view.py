import pyqtgraph as pg
import time
from google.protobuf import text_format
from typing import List
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from software.py_constants import *
from proto.import_all_protos import *
from software.thunderscope.constants import IndividualRobotMode
from software.thunderscope.robot_diagnostics.robot_info import RobotInfo
from software.thunderscope.robot_diagnostics.robot_status import RobotStatusView
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class RobotCrashDialog(QDialog):
    """Dialog to show information about a robot before it crashed,

    Displays the a message and RobotStatus.

    """

    def __init__(self, message, robot_status, parent=None):
        super().__init__(parent)

        self.setWindowTitle("Robot Crash")

        self.buttonBox = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

        self.layout = QVBoxLayout()
        self.message = QLabel(message)
        self.layout.addWidget(self.message)
        self.robot_status = RobotStatusView()
        if robot_status is not None:
            self.robot_status.update(robot_status)
            self.robot_status.toggle_visibility()
        self.layout.addWidget(self.robot_status)
        self.layout.addWidget(self.buttonBox)
        self.setLayout(self.layout)


class RobotViewComponent(QWidget):
    """Class to show a snapshot of the robot's current state,
    along with an expandable view of the full robot state

    Displays the vision pattern, capacitor/battery voltages,
    and other information about the robot state. Displays the whole RobotStatus
    message when expanded

    """

    def __init__(
        self,
        robot_id,
        available_control_modes: List[IndividualRobotMode],
        control_mode_signal,
    ):
        """
        Sets up a Robot Info Widget and a Robot Status Widget for each robot

        Sets the Robot Status widget to None so that it can be added later on button click

        :param robot_id: id of the current robot
        :param available_control_modes: the currently available input modes for the robots
                                        according to what mode thunderscope is run in
        :param control_mode_signal: the signal to emit when control mode changes
                                    in order to communicate to robot communications
        """
        super().__init__()

        self.robot_id = robot_id
        self.layout = QVBoxLayout()

        self.robot_info = RobotInfo(
            robot_id, available_control_modes, control_mode_signal
        )
        self.layout.addWidget(self.robot_info)

        self.robot_status = None

        self.robot_info.robot_status_expand.clicked.connect(self.robot_status_expand)

        self.layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(self.layout)

    def robot_status_expand(self):
        """
        Handles the info button click event from the Robot Info widget
        If robot status widget is not defined, initialises one and adds it to this layout
        If robot status widget is defined, toggles its visibility
        """
        if not self.robot_status:
            self.robot_status = RobotStatusView()
            self.layout.addWidget(self.robot_status)

        self.robot_status.toggle_visibility()

    def update(self, robot_status):
        """
        Updates the Robot View Components with the new robot status message
        Updates the robot info widget and, if initialized, the robot status widget as well

        :param robot_status: the new message data to update the widget with
        """
        self.robot_info.update(
            robot_status.motor_status,
            robot_status.power_status,
            robot_status.error_code,
        )
        if self.robot_status:
            self.robot_status.update(robot_status)


class RobotView(QScrollArea):
    """
    Widget that displays a collection of robot view components for all
    robots currently being used

    Contains signal to communicate with robot diagnostics when control mode changes
    """

    control_mode_signal = QtCore.pyqtSignal(int, int)

    def __init__(self, available_control_modes: List[IndividualRobotMode]):

        """
        Initialize the robot view component for each robot.

        :param available_control_modes: the currently available input modes for the robots
                                        according to what mode thunderscope is run in
        """

        super().__init__()

        self.robot_status_buffer = ThreadSafeBuffer(10, RobotStatus)
        self.robot_crash_buffer = ThreadSafeBuffer(10, RobotCrash)

        self.layout = QVBoxLayout()

        self.robot_view_widgets = []
        self.robot_last_crash_time_s = []

        for id in range(MAX_ROBOT_IDS_PER_SIDE):
            robot_view_widget = RobotViewComponent(
                id, available_control_modes, self.control_mode_signal
            )
            self.robot_view_widgets.append(robot_view_widget)
            self.layout.addWidget(robot_view_widget)
            self.robot_last_crash_time_s.append(0)

        # ignore repeated crash proto
        self.robot_crash_timeout_s = 5

        # for a QScrollArea, widgets cannot be added to it directly
        # doing so causes no scrolling to happen, and all the components get smaller
        # instead, widgets are added to the layout which is set for a container
        # the container is set as the current QScrollArea's widget
        self.container = QFrame(self)
        self.container.setLayout(self.layout)
        self.setWidget(self.container)
        self.setWidgetResizable(True)

    def refresh(self):
        """
        Refresh the view
        Gets a RobotStatus proto and calls the corresponding update method
        Until the buffer is empty
        """
        robot_status = self.robot_status_buffer.get(block=False, return_cached=False)

        while robot_status is not None:
            self.robot_view_widgets[robot_status.robot_id].update(robot_status)
            robot_status = self.robot_status_buffer.get(
                block=False, return_cached=False
            )

        robot_crash = self.robot_crash_buffer.get(block=False, return_cached=False)

        if robot_crash is not None:
            if (
                time.time() - self.robot_last_crash_time_s[robot_crash.robot_id]
                > self.robot_crash_timeout_s
            ):
                robot_crash_text = (
                    f"robot_id: {robot_crash.robot_id}\n"
                    + f"exit_signal: {robot_crash.exit_signal}\n"
                    + f"stack_dump: {robot_crash.stack_dump}"
                )
                dialog = RobotCrashDialog(robot_crash_text, robot_crash)
                dialog.exec()
            self.robot_last_crash_time_s[robot_crash.robot_id] = time.time()
