import pyqtgraph as pg
from typing import List
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from software.py_constants import *
from proto.import_all_protos import *
from software.thunderscope.constants import IndividualRobotMode
from software.thunderscope.robot_diagnostics.robot_info import RobotInfo
from software.thunderscope.robot_diagnostics.robot_status import RobotStatusView
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class RobotViewComponent(QWidget):
    """Class to show a snapshot of the robot's current state,
    along with an expandable view of the full robot state

    Displays the vision pattern, capacitor/battery voltages,
    and other information about the robot state. Displays the whole RobotStatus
    message when expanded

    """

    def __init__(
        self,
        id,
        available_control_modes: List[IndividualRobotMode],
        control_mode_signal,
    ):
        """
        Sets up a Robot Info Widget and a Robot Status Widget for each robot

        Sets the Robot Status widget to None so that it can be added later on button click

        :param id: id of the current robot
        :param available_control_modes: the currently available input modes for the robots
                                        according to what mode thunderscope is run in
        :param control_mode_signal: the signal to emit when control mode changes
                                    in order to communicate to robot communications
        """
        super().__init__()

        self.layout = QVBoxLayout()

        self.robot_info = RobotInfo(id, available_control_modes, control_mode_signal)
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
        if self.robot_status:
            self.robot_status.toggle_visibility()
        else:
            self.robot_status = RobotStatusView()
            self.layout.addWidget(self.robot_status)
            self.robot_status.toggle_visibility()


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

        self.robot_status_buffer = ThreadSafeBuffer(100, RobotStatus)

        self.layout = QVBoxLayout()

        self.robot_view_widgets = []

        for id in range(MAX_ROBOT_IDS_PER_SIDE):
            robot_view_widget = RobotViewComponent(
                id, available_control_modes, self.control_mode_signal
            )
            self.robot_view_widgets.append(robot_view_widget)
            self.layout.addWidget(robot_view_widget)

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
        """
        robot_status = self.robot_status_buffer.get(block=False, return_cached=False)

        if robot_status is not None:
            self.robot_view_widgets[robot_status.robot_id].robot_info.update(
                robot_status.power_status, robot_status.error_code
            )
            self.robot_view_widgets[robot_status.robot_id].robot_status.update(
                robot_status
            )
