import numpy

from proto.import_all_protos import *
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt import QtCore


import software.python_bindings as tbots_cpp

from software.py_constants import *
from software.thunderscope.constants import DiagnosticsConstants
from software.thunderscope.robot_diagnostics.controller_base import ControllerBase
from software.thunderscope.robot_diagnostics.handheld_controller import (
    HandheldController,
)
from software.thunderscope.robot_diagnostics.keyboard_controller import (
    KeyboardController,
)


class HandheldControllerWidget(QWidget):
    """This widget lets the user detect and use a connected handheld controller
    to manually control our robots. The user can toggle whether controller input
    is enabled or disabled.
    """

    kick_button_pressed = QtCore.pyqtSignal()
    """Signal emitted when the kick button is pressed on the controller"""

    chip_button_pressed = QtCore.pyqtSignal()
    """Signal emitted when the chip button is pressed on the controller"""

    def __init__(self) -> None:
        """Initialize the HandheldControllerWidget."""
        super().__init__()

        self.constants = tbots_cpp.createRobotConstants()

        self.handheld_controller: ControllerBase | None = None

        self.motor_control = MotorControl()
        self.dribbler_speed = 0
        self.kick_power = DiagnosticsConstants.MIN_KICK_POWER
        self.chip_distance = DiagnosticsConstants.MIN_CHIP_POWER

        widget_layout = QVBoxLayout()
        widget_layout.addWidget(self.__create_widgets())
        self.setLayout(widget_layout)

        self.detect_controller()

    def detect_controller(self) -> None:
        """Scan the currently connected devices for a supported handheld
        controller and, if one is found, set it as the device to accept
        controller inputs from.
        """
        if self.handheld_controller is not None:
            self.handheld_controller.close()
        self.handheld_controller = HandheldController.detect()
        self.__update_controller_status()

    def use_keyboard_controller(self) -> None:
        """Switch to keyboard input, closing any active controller first."""
        if self.handheld_controller is not None:
            self.handheld_controller.close()
        self.handheld_controller = KeyboardController()
        self.__update_controller_status()

    def controller_input_enabled(self) -> bool:
        """Check whether controller input is enabled.

        :return: true if controller input is enabled, false otherwise
        """
        return self.enable_input_checkbox.isChecked()

    def refresh(self) -> None:
        if self.handheld_controller is None:
            return

        self.handheld_controller.update()

        if not self.handheld_controller.connected():
            self.handheld_controller.close()
            self.handheld_controller = None
            self.__update_controller_status()
            return

        if self.enable_input_checkbox.isChecked():
            self.__read_controller_inputs()

    def __create_widgets(self) -> QGroupBox:
        """Create the widgets that make up the HandheldControllerWidget UI.

        :return: a QGroupBox containing:
            - a QLabel for displaying the current controller connection status
            - a QPushButton that tries detecting a controller when pressed
            - a QCheckBox that controls whether controller input is enabled
        """
        self.controller_status_label = QLabel()
        self.controller_status_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)

        self.detect_controller_button = QPushButton("Detect Controller")
        self.detect_controller_button.clicked.connect(self.detect_controller)

        self.use_keyboard_button = QPushButton("Use Keyboard")
        self.use_keyboard_button.clicked.connect(self.use_keyboard_controller)

        self.enable_input_checkbox = QCheckBox("Enable Input")
        self.enable_input_checkbox.setChecked(False)
        self.enable_input_checkbox.setEnabled(True)

        grid_layout = QGridLayout()
        grid_layout.addWidget(self.controller_status_label, 0, 0, 3, 1)
        grid_layout.addWidget(self.detect_controller_button, 0, 1)
        grid_layout.addWidget(self.use_keyboard_button, 1, 1)
        grid_layout.addWidget(
            self.enable_input_checkbox, 2, 1, QtCore.Qt.AlignmentFlag.AlignHCenter
        )
        grid_layout.setColumnStretch(0, 4)
        grid_layout.setColumnStretch(1, 1)

        box = QGroupBox()
        box.setTitle("Controller Status")
        box.setLayout(grid_layout)

        return box

    def __read_controller_inputs(self) -> None:
        """Read controller state and update motor control proto."""
        vx, vy, vrot = self.handheld_controller.get_move_velocity()
        speed = self.handheld_controller.get_speed_factor()

        self.motor_control.direct_velocity_control.velocity.x_component_meters = (
            vx * self.constants.robot_max_speed_m_per_s * speed
        )
        self.motor_control.direct_velocity_control.velocity.y_component_meters = (
            vy * self.constants.robot_max_speed_m_per_s * speed
        )
        self.motor_control.direct_velocity_control.angular_velocity.radians_per_second = (
            vrot * self.constants.robot_max_ang_speed_rad_per_s * speed
        )

        kick_step = self.handheld_controller.get_kick_power_step()
        if kick_step:
            self.kick_power = numpy.clip(
                a=self.kick_power + kick_step * DiagnosticsConstants.KICK_POWER_STEPPER,
                a_min=DiagnosticsConstants.MIN_KICK_POWER,
                a_max=DiagnosticsConstants.MAX_KICK_POWER,
            )
            self.chip_distance = numpy.clip(
                a=self.chip_distance
                + kick_step * DiagnosticsConstants.CHIP_DISTANCE_STEPPER,
                a_min=DiagnosticsConstants.MIN_CHIP_POWER,
                a_max=DiagnosticsConstants.MAX_CHIP_POWER,
            )

        dribbler_step = self.handheld_controller.get_dribbler_step()
        if dribbler_step:
            self.dribbler_speed = int(
                numpy.clip(
                    a=self.dribbler_speed
                    + dribbler_step * DiagnosticsConstants.DRIBBLER_RPM_STEPPER,
                    a_min=self.constants.indefinite_dribbler_speed_rpm,
                    a_max=-self.constants.indefinite_dribbler_speed_rpm,
                )
            )

        self.motor_control.dribbler_speed_rpm = (
            self.dribbler_speed if self.handheld_controller.is_dribbler_held() else 0
        )

        if self.handheld_controller.is_kick_fired():
            self.kick_button_pressed.emit()

        if self.handheld_controller.is_chip_fired():
            self.chip_button_pressed.emit()

    def __update_controller_status(self) -> None:
        """Update the widget to display the current controller connection status."""
        if self.handheld_controller and self.handheld_controller.connected():
            self.controller_status_label.setText(
                f"Connected: {self.handheld_controller.name()}"
            )
            self.controller_status_label.setStyleSheet("background-color: green")
            self.enable_input_checkbox.setEnabled(True)
        else:
            self.controller_status_label.setText("Disconnected")
            self.controller_status_label.setStyleSheet("background-color: red")
            self.enable_input_checkbox.setChecked(False)
            self.enable_input_checkbox.setEnabled(False)
