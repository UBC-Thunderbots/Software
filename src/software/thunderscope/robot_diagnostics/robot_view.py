import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from software.py_constants import *
from proto.import_all_protos import *
from software.thunderscope.robot_diagnostics.robot_info import RobotInfo
from software.thunderscope.robot_diagnostics.robot_status import RobotStatusView
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class RobotView(QScrollArea):
    """Class to show a snapshot of the robot's current state.

    Displays the vision pattern, capacitor/battery voltages,
    and other information about the robot state.

    """

    control_mode_signal = QtCore.pyqtSignal(int, int)

    def __init__(self, load_fullsystem):

        """
        Initialize the robot view.
        Sets up a Robot Info Widget for each robot

        :param load_fullsystem: whether fullsystem is loaded currently
                                Shows / Hides AI option from control mode menu accordingly
        """

        super().__init__()

        self.robot_status_buffer = ThreadSafeBuffer(100, RobotStatus)

        self.layout = QVBoxLayout()

        self.robot_info_widgets = [
            RobotInfo(id, load_fullsystem, self.control_mode_signal)
            for id in range(MAX_ROBOT_IDS_PER_SIDE)
        ]
        self.robot_status_widgets = [
            RobotStatusView() for id in range(MAX_ROBOT_IDS_PER_SIDE)
        ]

        for id in range(MAX_ROBOT_IDS_PER_SIDE):
            self.robot_info_widgets[id].robot_status_expand.clicked.connect(
                self.robot_status_widgets[id].toggle_visibility
            )

            self.layout.addWidget(self.robot_info_widgets[id])
            self.layout.addWidget(self.robot_status_widgets[id])

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
            self.robot_info_widgets[robot_status.robot_id].update(
                robot_status.power_status, robot_status.error_code
            )
            self.robot_status_widgets[robot_status.robot_id].update(robot_status)
