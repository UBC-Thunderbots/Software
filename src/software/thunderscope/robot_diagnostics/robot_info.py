import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from software.py_constants import *
from typing import List
from proto.import_all_protos import *
import software.thunderscope.common.common_widgets as common_widgets
from software.thunderscope.constants import *
from threading import Lock


class RobotInfo(QWidget):

    # Offsets the minimum of the battery bar from the minimum ideal voltage
    # Allows battery % to go below the minimum ideal level
    BATTERY_MIN_OFFSET = 3

    toggle_one_connection_signal = QtCore.pyqtSignal(int, int)

    def __init__(
        self,
        robot_id,
        available_control_modes: List[IndividualRobotMode],
        control_mode_signal,
    ):
        """
        Initialize a single robot's info widget

        :param robot_id: id of robot whose info is being displayed
        :param available_control_modes: the currently available input modes for the robots
                                        according to what mode thunderscope is run in
        :param control_mode_signal: signal that should be emitted when a robot changes control mode
        """

        super().__init__()

        self.robot_id = robot_id
        self.control_mode_signal = control_mode_signal

        # when this robot has a battery warning, this is set to True
        # which prevents spamming the same battery warning
        # set back to False if battery is back above warning level
        self.battery_warning_disabled = False

        # True means the robot is receiving constant RobotStatus messages
        # False means that the robot has received no RobotStatus for at least 1 tick
        # used to grey out display when robot disconnects
        self.is_connected = False

        # True means the timer to grey oyt the UI has been triggered
        # to prevent too many timers and UI flashing
        self.to_be_disconnected = False
        self.connected_lock = Lock()

        self.layout = QHBoxLayout()

        self.status_layout = QVBoxLayout()

        # Battery Bar
        self.battery_layout = QHBoxLayout()
        self.battery_progress_bar = common_widgets.ColorProgressBar(
            MIN_BATTERY_VOLTAGE - self.BATTERY_MIN_OFFSET, MAX_BATTERY_VOLTAGE
        )
        # Battery Voltage Label
        self.battery_label = QLabel()
        self.battery_label.setText("NAN")
        # Label changes when voltage bar level changes
        self.battery_progress_bar.floatValueChanged.connect(
            lambda float_val: self.battery_label.setText("%.2fV" % float_val)
        )
        self.battery_layout.addWidget(self.battery_progress_bar)
        self.battery_layout.addWidget(self.battery_label)

        self.status_layout.addLayout(self.battery_layout)

        # Control mode dropdown
        self.control_mode_layout = QHBoxLayout()
        self.control_mode_menu = self.create_control_mode_menu(available_control_modes)

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
        self.color_vision_pattern = self.create_vision_pattern_label(
            Colors.ROBOT_MIDDLE_BLUE, ROBOT_RADIUS, True
        )
        self.bw_vision_pattern = self.create_vision_pattern_label(
            Colors.BW_ROBOT_MIDDLE_BLUE, ROBOT_RADIUS, False
        )

        self.vision_pattern_label = QLabel()
        self.disconnect_robot()
        self.layout.addWidget(self.vision_pattern_label)

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

    def create_control_mode_menu(
        self, available_control_modes: List[IndividualRobotMode]
    ):
        """
        Creates the drop down menu to select the input for each robot
        :param robot_id: the id of the robot this menu belongs to
        :param available_control_modes: the currently available input modes for the robots
                                        according to what mode thunderscope is run in
        :return: QComboBox object
        """
        control_mode_menu = QComboBox()

        control_mode_menu.addItems(
            [control_mode.name for control_mode in available_control_modes]
        )

        if IndividualRobotMode.AI in available_control_modes:
            control_mode_menu.setCurrentIndex(
                control_mode_menu.findText(IndividualRobotMode.AI.name)
            )
        else:
            control_mode_menu.setCurrentIndex(
                control_mode_menu.findText(IndividualRobotMode.NONE.name)
            )

        control_mode_menu.currentIndexChanged.connect(
            lambda mode, robot_id=self.robot_id: self.control_mode_signal.emit(
                mode, robot_id
            )
        )

        return control_mode_menu

    def create_vision_pattern_label(self, team_colour, radius, connected):
        """Given a robot id, team color and radius, draw the vision
        pattern on a label and return it.

        :param team_colour: The team colour
        :param radius: The radius of the robot
        :param connected: True if vision pattern should have color, False if black and white

        """
        pixmap = QtGui.QPixmap(radius * 2, radius * 2)
        pixmap.fill(QtCore.Qt.GlobalColor.transparent)

        painter = QtGui.QPainter(pixmap)
        painter.setPen(pg.mkPen("black"))
        painter.setBrush(pg.mkBrush("black"))

        common_widgets.draw_robot(
            painter, QtCore.QRectF(0, 0, int(radius * 2), int(radius * 2),), -45, 270,
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
        top_circle_locations = [
            QtCore.QPointF(radius + radius / 2 + 5, radius - radius / 2),
            QtCore.QPointF(radius - radius / 2 - 5, radius - radius / 2),
            QtCore.QPointF(radius - radius / 2, radius + radius / 2 + 5),
            QtCore.QPointF(radius + radius / 2, radius + radius / 2 + 5),
        ]

        for color, location in zip(
            (
                Colors.VISION_PATTERN_LOOKUP
                if connected
                else Colors.BW_VISION_PATTERN_LOOKUP
            )[self.robot_id],
            top_circle_locations,
        ):
            painter.setBrush(pg.mkBrush(color))
            painter.drawEllipse(location, radius / 5, radius / 5)

        painter.end()

        return pixmap

    def update(self, power_status, error_codes):
        """
        Receives parts of a RobotStatus message

        Sets the robot UI as connected and updates the UI
        Then sets the robot to disconnected with a timer callback to update the UI
        :param power_status: The power status message for this robot
        :param error_codes: The error codes of this robot
        """
        self.reconnect_robot()

        self.update_ui(power_status, error_codes)

        self.mark_to_be_disconnected()

    def mark_to_be_disconnected(self):
        """
        Sets the boolean flag to disconnected
        and calls a timer with a callback to grey out the UI in DISCONNECT_DURATION_MS
        """
        self.is_connected = False
        if not self.to_be_disconnected:
            QtCore.QTimer.singleShot(DISCONNECT_DURATION_MS, self.disconnect_robot)
            self.to_be_disconnected = True

    def disconnect_robot(self):
        """
        If robot is disconnected, attempts to get the lock
        Once locked, if robot is still disconnected, greys out the robot UI
        """
        if not self.is_connected:
            self.connected_lock.acquire()
            if not self.is_connected:
                self.vision_pattern_label.setPixmap(self.bw_vision_pattern)
                self.to_be_disconnected = False
            self.connected_lock.release()

    def reconnect_robot(self):
        """
        Attempts to get the lock
        Once locked, resets boolean flag and sets robot UI to the color version
        """
        self.is_connected = True
        self.connected_lock.acquire()
        self.vision_pattern_label.setPixmap(self.color_vision_pattern)
        self.connected_lock.release()

    def update_ui(self, power_status, error_codes):
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
