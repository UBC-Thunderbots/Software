import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from software.py_constants import *
from proto.import_all_protos import *
from software.thunderscope.robot_diagnostics.robot_info import RobotInfo
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class RobotView(QScrollArea):
    """Class to show a snapshot of the robot's current state.

    Displays the vision pattern, capacitor/battery voltages,
    and other information about the robot state.

    """

    toggle_all_connection_signal = QtCore.pyqtSignal(int, int)

    def __init__(self, load_fullsystem):

        """
        Initialize the robot view.
        Sets up a Robot Info Widget for each robot
        """

        super().__init__()

        self.robot_status_buffer = ThreadSafeBuffer(100, RobotStatus)

        self.layout = QVBoxLayout()

        self.robot_info_widgets = [
            RobotInfo(x, load_fullsystem) for x in range(MAX_ROBOT_IDS_PER_SIDE)
        ]

        for id in range(MAX_ROBOT_IDS_PER_SIDE):
            self.robot_info_widgets[id].toggle_one_connection_signal.connect(
                lambda mode, robot_id: self.toggle_all_connection_signal.emit(
                    mode, robot_id
                )
            )

            self.layout.addWidget(self.robot_info_widgets[id])

        self.container = QWidget()
        self.container.setLayout(self.layout)
        self.setWidget(self.container)
        self.setWidgetResizable(True)

    def refresh(self):
        """
        Refresh the view
        Gets a RobotStatus proto and calls the corresponding update method
        """
        #robot_status = self.robot_status_buffer.get(block=False)

        robot_status = RobotStatus(
            robot_id=5,
            power_status=PowerStatus(
                breakbeam_tripped=True,
                battery_voltage=MAX_BATTERY_VOLTAGE
            ),
            error_code=[]
        )

        self.robot_info_widgets[robot_status.robot_id].update(
            robot_status.power_status, robot_status.error_code
        )
