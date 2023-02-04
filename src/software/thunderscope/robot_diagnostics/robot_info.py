import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from software.py_constants import *

class RobotInfo(QWidget):

    def __init__(self, robot_id, load_fullsystem):
        """
        Initialize a single robot's info widget
        """

        super().__init__()
        
        self.layout = QHBoxLayout()
        self.status_layout = QVBoxLayout()
        self.control_mode_layout = QHBoxLayout()
        self.battery_progress_bar = QProgressBar()
        self.control_mode_menu = self.create_control_mode_menu(robot_id, load_fullsystem)

    def create_control_mode_menu(self, robot_id, load_fullsystem):
        """
        Creates the drop down menu to select the input for each robot
        :param robot_id: the id of the robot this menu belongs to
        :param load_fullsystem: whether fullsystem is also an option
        :return: QComboBox object
        """
        control_mode_menu = QComboBox()

        if load_fullsystem:
            control_mode_menu.addItem("AI")

        control_mode_menu.addItems(["None", "Manual"])
        control_mode_menu.setCurrentIndex(0)

        control_mode_menu.currentIndexChanged.connect(
            lambda mode, robot_id=robot_id: self.toggle_robot_connection_signal.emit(
                mode, robot_id
            )
        )

        return control_mode_menu