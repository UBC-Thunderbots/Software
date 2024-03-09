from __future__ import annotations
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from software.py_constants import *
from typing import List
from proto.import_all_protos import *
import software.thunderscope.common.common_widgets as common_widgets
from software.thunderscope.constants import *
from software.thunderscope.robot_diagnostics.motor_fault_view import MotorFaultView
import time as time
from typing import Type, List


class BreakbeamLabel(QLabel):
    """
    Displays the current breakbeam status
    Extension of a QLabel which displays a tooltip and updates the UI with the current status
    """

    BREAKBEAM_BORDER = "border: 1px solid black"

    def __init__(self) -> None:
        """
        Constructs a breakbeam indicator and sets the UI to the default uninitialized state
        """
        super().__init__()

    def update_breakbeam_status(self, new_breakbeam_status: bool) -> None:
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

    def event(self, event: QtCore.QEvent) -> bool:
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
        robot_id: int,
        available_control_modes: List[IndividualRobotMode],
        control_mode_signal: Type[QtCore.pyqtSignal],
    ) -> None:
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

        self.time_of_last_robot_status = time.time()

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

        # Stop primitive received indicator
        self.stop_primitive_label = QLabel()
        self.stop_primitive_label.setText("NA")
        self.battery_layout.addWidget(self.stop_primitive_label)

        # Primitive loss rate label
        self.primitive_loss_rate_label = common_widgets.ColorQLabel(
            max_val=MAX_ACCEPTABLE_PACKET_LOSS_PERCENT
        )
        self.battery_layout.addWidget(self.primitive_loss_rate_label)

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
        self.robot_model_layout.setContentsMargins(0, 5, 5, 0)

        # Vision Pattern
        self.color_vision_pattern = self.create_vision_pattern(
            Colors.ROBOT_MIDDLE_BLUE, ROBOT_RADIUS, True
        )
        self.bw_vision_pattern = self.create_vision_pattern(
            Colors.BW_ROBOT_MIDDLE_BLUE, ROBOT_RADIUS, False
        )

        self.robot_model = QLabel()

        # breakbeam indicator above robot
        self.breakbeam_label = BreakbeamLabel()
        self.breakbeam_label.setFixedWidth(self.color_vision_pattern.width())
        self.breakbeam_label.setFixedHeight(self.color_vision_pattern.width() * 0.25)

        self.robot_model_layout.addWidget(self.breakbeam_label)
        self.robot_model_layout.addWidget(self.robot_model)
        self.layout.addLayout(self.robot_model_layout)

        self.__reset_ui()

        self.layout.addLayout(self.status_layout)
        self.setLayout(self.layout)

    def create_robot_status_expand_button(self) -> QPushButton:
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
    ) -> QComboBox:
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

    def create_vision_pattern(
        self, team_colour: QtGui.QColor, radius: int, connected: bool
    ) -> QtGui.QPixmap:
        """Given a robot id, team color and radius, draw the vision
        pattern on a pixmap and return it.

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

    def update(self, robot_status: RobotStatus):
        """
        Receives parts of a RobotStatus message

        Saves the current time as the last robot status time
        Sets the robot UI as connected and updates the UI
        Then sets a timer callback to disconnect the robot if needed

        :param robot_status: The robot status message for this robot
        """
        self.time_of_last_robot_status = time.time()

        self.robot_model.setPixmap(self.color_vision_pattern)

        self.__update_ui(robot_status)

        QtCore.QTimer.singleShot(DISCONNECT_DURATION_MS, self.disconnect_robot)

    def disconnect_robot(self) -> None:
        """
        Calculates the time between the last robot status and now
        If more than our threshold, resets UI
        """
        time_since_last_robot_status = time.time() - self.time_of_last_robot_status
        if (
            time_since_last_robot_status
            > DISCONNECT_DURATION_MS * SECONDS_PER_MILLISECOND
        ):
            self.__reset_ui()

    def __reset_ui(self) -> None:
        """
        Resets the UI to the default, uninitialized values
        """
        self.robot_model.setPixmap(self.bw_vision_pattern)

        self.breakbeam_label.update_breakbeam_status(None)

    def __update_stop_primitive(self, is_running: bool) -> None:
        """
        Updates the stop primitive label based on the current running state
        :param is_running: if the robot is running currently
        """
        self.stop_primitive_label.setText("RUN" if is_running else "STOP")
        self.stop_primitive_label.setStyleSheet(
            f"background-color: {'green' if is_running else 'red'}; border: 1px solid black;"
        )

    def __update_ui(self, robot_status: RobotStatus) -> None:
        """
        Receives important sections of RobotStatus proto for this robot and updates widget with alerts
        Checks for
            - Whether breakbeam is tripped
            - If there are any motor faults
            - Battery voltage, and warns if it's too low
            - If this robot has errors
            - If the robot is stopped or running
        :param robot_status: The robot status message for this robot
        """
        motor_status = robot_status.motor_status
        power_status = robot_status.power_status
        network_status = robot_status.network_status
        primitive_executor_status = robot_status.primitive_executor_status

        self.__update_stop_primitive(primitive_executor_status.running_primitive)

        self.primitive_loss_rate_label.set_float_val(
            network_status.primitive_packet_loss_percentage
        )

        self.breakbeam_label.update_breakbeam_status(power_status.breakbeam_tripped)

        self.motor_fault_view.refresh(
            motor_status,
            # we access the front left field just to get the enum descriptor
            # so that we can translate from enum indexes to fault names
            motor_status.front_left.DESCRIPTOR.fields_by_name["motor_faults"],
        )

        self.battery_progress_bar.setValue(power_status.battery_voltage)
