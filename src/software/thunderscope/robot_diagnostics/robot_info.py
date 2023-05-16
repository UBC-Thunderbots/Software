import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from software.py_constants import *
from typing import List
from proto.import_all_protos import *
import software.thunderscope.common.common_widgets as common_widgets
from software.thunderscope.constants import *
from software.thunderscope.robot_diagnostics.motor_fault_view import MotorFaultView


class BreakbeamLabel(QLabel):
    """
    Displays the current breakbeam status
    Extension of a QLabel which displays a tooltip and updates the UI with the current status
    """

    BREAKBEAM_BORDER = "border: 2px solid black"

    def __init__(self):
        """
        Constructs a breakbeam indicator and sets the UI to the default uninitialized state
        """
        super().__init__()
        self.update_breakbeam_status(None)

    def update_breakbeam_status(self, new_breakbeam_status):
        """
        Updates the current breakbeam status and refreshes the UI accordingly
        :param new_breakbeam_status: the new breakbeam status
        """
        self.breakbeam_status = new_breakbeam_status

        if self.breakbeam_status is None:
            self.setStyleSheet(
                f"background-color: transparent; {self.BREAKBEAM_BORDER}"
            )
        elif self.breakbeam_status:
            self.setStyleSheet(
                f"background-color: red; {self.BREAKBEAM_BORDER};" "border-color: red"
            )
        else:
            self.setStyleSheet(
                f"background-color: green; {self.BREAKBEAM_BORDER};"
                "border-color: green"
            )

    def event(self, event):
        """
        Overridden event function which intercepts all events
        On hover, displays a tooltip with the current breakbeam status
        :param event: event to check
        """
        common_widgets.display_tooltip(
            event,
            "No Signal Yet"
            if self.breakbeam_status is None
            else "In Beam"
            if self.breakbeam_status
            else "Not In Beam",
        )

        return super().event(event)


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

        # motor fault visualisation for the 4 wheel motors
        self.motor_fault_view = MotorFaultView()

        self.control_mode_layout.addWidget(self.motor_fault_view)
        self.control_mode_layout.addWidget(self.control_mode_menu)
        self.control_mode_layout.addWidget(self.robot_status_expand)

        self.status_layout.addLayout(self.control_mode_layout)

        # Layout containing the Vision Pattern and breakbeam indicator
        self.robot_model_layout = QVBoxLayout()
        self.robot_model_layout.setContentsMargins(0, 15, 5, 10)

        # Vision pattern
        self.robot_model = self.create_vision_pattern_label(
            Colors.ROBOT_MIDDLE_BLUE, ROBOT_RADIUS
        )

        # breakbeam indicator above robot
        self.breakbeam_label = BreakbeamLabel()
        self.breakbeam_label.setFixedWidth(self.robot_model.sizeHint().width())
        self.breakbeam_label.setFixedHeight(self.robot_model.sizeHint().width() * 0.25)

        self.robot_model_layout.addWidget(self.breakbeam_label)
        self.robot_model_layout.addWidget(self.robot_model)
        self.layout.addLayout(self.robot_model_layout)

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

    def update(self, motor_status, power_status, error_codes):
        """
        Receives important sections of RobotStatus proto for this robot and updates widget with alerts
        Checks for
            - Whether breakbeam is tripped
            - If there are any motor faults
            - If Battery Voltage is too low
            - If this robot has errors
        :param motor_status: The motor status message for this robot
        :param power_status: The power status message for this robot
        :param error_codes: The error codes of this robot
        :return:
        """

        self.breakbeam_label.update_breakbeam_status(power_status.breakbeam_tripped)

        self.motor_fault_view.refresh(
            motor_status,
            # we access the front left field just to get the enum descriptor
            # so that we can translate from enum indexes to fault names
            motor_status.front_left.DESCRIPTOR.fields_by_name["motor_faults"],
        )

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
