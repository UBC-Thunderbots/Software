import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from software.py_constants import *
from proto.import_all_protos import *
import software.thunderscope.common.common_widgets as common_widgets
from software.thunderscope.constants import Colors, ERROR_CODE_MESSAGES


class RobotInfo(QWidget):

    toggle_one_connection_signal = QtCore.pyqtSignal(int, int)

    def __init__(self, robot_id, load_fullsystem):
        """
        Initialize a single robot's info widget
        """

        super().__init__()

        self.robot_id = robot_id

        # when this robot has a battery warning, this is set to True
        # which prevents spamming the same battery warning
        # set back to False if battery is back above warning level
        self.battery_warning_disabled = False

        self.layout = QHBoxLayout()

        self.status_layout = QVBoxLayout()

        # Battery Bar
        self.battery_progress_bar = common_widgets.ColorProgressBar(
            MIN_BATTERY_VOLTAGE, MAX_BATTERY_VOLTAGE
        )
        self.status_layout.addWidget(self.battery_progress_bar)

        # Control mode dropdown
        self.control_mode_layout = QHBoxLayout()
        self.control_mode_menu = self.create_control_mode_menu(load_fullsystem)

        # Robot Status expand button
        self.robot_status_expand = self.create_robot_status_expand_button()

        # Breakbeam status
        self.breakbeam_label = QLabel()
        self.breakbeam_label.setText("BREAKBEAM")
        self.breakbeam_label.setStyleSheet("background-color: grey")

        self.control_mode_layout.addWidget(self.breakbeam_label)
        self.control_mode_layout.addWidget(self.control_mode_menu)
        self.control_mode_layout.addWidget(self.robot_status_expand)

        self.status_layout.addLayout(self.control_mode_layout)

        # Vision Pattern
        self.layout.addWidget(self.create_vision_pattern_label("b", 25))

        self.layout.addLayout(self.status_layout)

        self.setLayout(self.layout)

    def create_robot_status_expand_button(self):
        """
        Creates the button to expand / collapse the robot status view
        :return: QPushButton object
        """
        button = QPushButton()
        button.setCheckable(True)
        button.setText("INFO")
        return button

    def create_control_mode_menu(self, load_fullsystem):
        """
        Creates the drop down menu to select the input for each robot
        :param robot_id: the id of the robot this menu belongs to
        :param load_fullsystem: whether fullsystem is also an option
        :return: QComboBox object
        """
        control_mode_menu = QComboBox()

        control_mode_menu.addItems(["None", "Manual"])
        control_mode_menu.setCurrentIndex(0)

        if load_fullsystem:
            control_mode_menu.addItem("AI")
            control_mode_menu.setCurrentIndex(2)

        control_mode_menu.currentIndexChanged.connect(
            lambda mode, robot_id=self.robot_id: self.toggle_one_connection_signal.emit(
                mode, robot_id
            )
        )

        return control_mode_menu

    def create_vision_pattern_label(self, team_colour, radius):
        """Given a robot id, team color and radius, draw the vision
        pattern on a label and return it.

        :param team_colour: The team colour
        :param radius: The radius of the robot

        """
        pixmap = QtGui.QPixmap(radius * 2, radius * 2)
        pixmap.fill(QtCore.Qt.GlobalColor.transparent)

        painter = QtGui.QPainter(pixmap)
        painter.setPen(pg.mkPen("black"))
        painter.setBrush(pg.mkBrush("black"))

        convert_degree = -16

        painter.drawChord(
            QtCore.QRectF(0, 0, int(radius * 2), int(radius * 2),),
            -45 * convert_degree,
            270 * convert_degree,
        )

        # Draw the vision pattern
        # Draw the centre team color
        painter.setBrush(pg.mkBrush(team_colour))
        painter.drawEllipse(QtCore.QPointF(radius, radius), radius / 3, radius / 3)
        painter.setPen(pg.mkPen("white"))
        painter.drawText(
            QtCore.QPointF(radius - radius / 8, radius + radius / 4), str(self.robot_id)
        )
        painter.setPen(pg.mkPen("black"))

        # Grab the colors for the vision pattern and setup the locations
        # for the four circles in the four corners
        top_right, top_left, bottom_left, bottom_right = Colors.VISION_PATTERN_LOOKUP[
            self.robot_id
        ]
        top_circle_locations = [
            QtCore.QPointF(radius + radius / 2 + 5, radius - radius / 2),
            QtCore.QPointF(radius - radius / 2 - 5, radius - radius / 2),
            QtCore.QPointF(radius - radius / 2, radius + radius / 2 + 5),
            QtCore.QPointF(radius + radius / 2, radius + radius / 2 + 5),
        ]

        for color, location in zip(
            Colors.VISION_PATTERN_LOOKUP[self.robot_id], top_circle_locations
        ):
            painter.setBrush(pg.mkBrush(color))
            painter.drawEllipse(location, radius / 5, radius / 5)

        painter.end()

        label = QLabel()
        label.setPixmap(pixmap)

        return label

    def update(self, power_status, error_codes):
        """
        Receives important sections of RobotStatus proto for this robot and updates widget with alerts
        Checks for
            - Whether breakbeam is tripped
            - If Battery Voltage is too low
            - If this robot has errors
        :param power_status: The power status message for this robot
        :param error_codes: The error codes of this robot
        :return:
        """

        if power_status.breakbeam_tripped:
            self.breakbeam_label.setText("In Beam")
            self.breakbeam_label.setStyleSheet("background-color: red")
        else:
            self.breakbeam_label.setText("Not in Beam")
            self.breakbeam_label.setStyleSheet("background-color: green")

        self.battery_progress_bar.setValue(power_status.battery_voltage)

        if (
            power_status.battery_voltage <= BATTERY_WARNING_VOLTAGE
            and not self.battery_warning_disabled
        ):
            QMessageBox.information(
                self,
                "Battery Voltage Alert",
                f"robot {self.robot_id} voltage is {power_status.battery_voltage}",
            )
            self.battery_warning_disabled = True
        elif power_status.battery_voltage > BATTERY_WARNING_VOLTAGE:
            self.battery_warning_disabled = False

        for code in error_codes:
            if code != ErrorCode.NO_ERROR:
                QMessageBox.warning(
                    self,
                    f"Warning: {ERROR_CODE_MESSAGES[code]}",
                    f"{ERROR_CODE_MESSAGES[code]} warning for robot {self.robot_id}",
                )
