from pyqtgraph.Qt import QtCore
from pyqtgraph.Qt.QtWidgets import *
from software.py_constants import *
from proto.import_all_protos import *
from software.thunderscope.constants import IndividualRobotMode
from software.thunderscope.robot_diagnostics.robot_info import RobotInfo
from software.thunderscope.robot_diagnostics.robot_status import RobotStatusView
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from typing import Type


class RobotViewComponent(QWidget):
    """Class to show a snapshot of the robot's current state,
    along with an expandable view of the full robot state

    Displays the vision pattern, capacitor/battery voltages, and other information
    about the robot state. Displays the whole RobotStatus message when expanded
    """

    def __init__(
        self,
        robot_id: int,
        available_control_modes: list[IndividualRobotMode],
        individual_robot_control_mode_signal: Type[QtCore.pyqtSignal],
    ):
        """Sets up a Robot Info Widget and a Robot Status Widget for each robot

        Sets the Robot Status widget to None so that it can be added later on button click

        :param robot_id: id of the current robot
        :param available_control_modes: the currently available input modes for the robots
                                        according to what mode thunderscope is run in
        :param individual_robot_control_mode_signal: the signal to emit when control mode changes
                                    in order to communicate to robot communications
        """
        super().__init__()

        self.robot_id = robot_id
        self.layout = QVBoxLayout()

        self.robot_info = RobotInfo(
            robot_id, available_control_modes, individual_robot_control_mode_signal
        )
        self.layout.addWidget(self.robot_info)

        self.robot_status = None

        self.robot_info.robot_status_expand.clicked.connect(self.robot_status_expand)

        self.layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(self.layout)

    def robot_status_expand(self) -> None:
        """Handles the info button click event from the Robot Info widget
        If robot status widget is not defined, initialises one and adds it to this layout
        If robot status widget is defined, toggles its visibility
        """
        if not self.robot_status:
            self.robot_status = RobotStatusView()
            self.layout.addWidget(self.robot_status)

        self.robot_status.toggle_visibility()

    def update(
        self, robot_status: RobotStatus, robot_statistic: RobotStatistic
    ) -> None:
        """Updates the Robot View Components with the new robot status message
        Updates the robot info widget and, if initialized, the robot status widget as well

        :param robot_status: the new message data to update the widget with
        :param robot_statistic : robot statistic proto to update with new metrics
        """
        if robot_status is not None:
            self.robot_info.update_robot_status(robot_status)
            if self.robot_status:
                self.robot_status.update(robot_status)

        if robot_statistic is not None:
            self.robot_info.update_rtt(robot_statistic)


class RobotView(QScrollArea):
    """Widget that displays a collection of robot view components for all
    robots currently being used

    Contains signal to communicate with robot diagnostics when control mode changes
    """

    individual_robot_control_mode_signal = QtCore.pyqtSignal(int, IndividualRobotMode)

    def __init__(self, available_control_modes: list[IndividualRobotMode]) -> None:
        """Initialize the robot view component for each robot.

        :param available_control_modes: the currently available input modes for the robots
                                        according to what mode thunderscope is run in
        """
        super().__init__()

        self.robot_status_buffer = ThreadSafeBuffer(10, RobotStatus)
        self.robot_statistic_buffer = ThreadSafeBuffer(10, RobotStatistic)

        self.layout = QVBoxLayout()

        self.components = []

        for id in range(MAX_ROBOT_IDS_PER_SIDE):
            component = RobotViewComponent(
                id, available_control_modes, self.individual_robot_control_mode_signal
            )
            self.components.append(component)
            self.layout.addWidget(component)

        # for a QScrollArea, widgets cannot be added to it directly
        # doing so causes no scrolling to happen, and all the components get smaller
        # instead, widgets are added to the layout which is set for a container
        # the container is set as the current QScrollArea's widget
        self.container = QFrame(self)
        self.container.setLayout(self.layout)
        self.setWidget(self.container)
        self.setWidgetResizable(True)

    def refresh(self) -> None:
        """Refresh the view
        Gets a RobotStatus proto and calls the corresponding update method
        until the buffer is empty
        """
        robot_status = self.robot_status_buffer.get(block=False, return_cached=False)
        robot_statistic = self.robot_statistic_buffer.get(
            block=False, return_cached=False
        )

        if (
            robot_status is not None
            and robot_statistic is not None
            and robot_status.robot_id == robot_statistic.robot_id
        ):  # if both pieces of data are available
            self.components[robot_status.robot_id].update(
                robot_status=robot_status, robot_statistic=robot_statistic
            )
        else:
            if robot_status is not None:
                self.components[robot_status.robot_id].update(
                    robot_status,
                    robot_statistic=None,
                )
            if robot_statistic is not None:
                self.components[robot_statistic.robot_id].update(
                    robot_status=None, robot_statistic=robot_statistic
                )
