import numpy

import evdev
from evdev import ecodes

from proto.import_all_protos import *
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt import QtCore


import software.python_bindings as tbots_cpp

from software.py_constants import *
from software.thunderscope.constants import DiagnosticsConstants
from software.thunderscope.robot_diagnostics.handheld_controller import (
    HandheldController,
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

        self.constants = tbots_cpp.create2021RobotConstants()

        self.handheld_controller: HandheldController | None = None

        self.last_d_pad_axis_x_value = 0
        self.last_d_pad_axis_y_value = 0
        self.last_btn_a_value = False
        self.last_btn_b_value = False

        self.motor_control = MotorControl()
        self.dribbler_speed = 0
        self.kick_power = DiagnosticsConstants.MIN_KICK_POWER
        self.chip_distance = DiagnosticsConstants.MIN_CHIP_POWER

        widget_layout = QVBoxLayout()
        widget_layout.addWidget(self.__create_widgets())
        self.setLayout(widget_layout)

        self.detect_controller()

    def detect_controller(self) -> None:
        """Scan through the list of currently connected devices for a supported
        handheld controller and, if one is found, set it as the device to accept
        controller inputs from.
        """
        self.handheld_controller = None

        for path in evdev.list_devices():
            device = evdev.InputDevice(path)
            if device.name in DiagnosticsConstants.SUPPORTED_CONTROLLERS:
                self.handheld_controller = HandheldController(path)
                break

        self.__update_controller_status()

    def controller_input_enabled(self) -> bool:
        """Check whether controller input is enabled.

        :return: true if controller input is enabled, false otherwise
        """
        return self.enable_input_checkbox.isChecked()

    def refresh(self) -> None:
        if self.handheld_controller is None:
            return

        if not self.handheld_controller.connected():
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

        self.enable_input_checkbox = QCheckBox("Enable Input")
        self.enable_input_checkbox.setChecked(False)
        self.enable_input_checkbox.setEnabled(True)

        grid_layout = QGridLayout()
        grid_layout.addWidget(self.controller_status_label, 0, 0, 2, 1)
        grid_layout.addWidget(self.detect_controller_button, 0, 1)
        grid_layout.addWidget(
            self.enable_input_checkbox, 1, 1, QtCore.Qt.AlignmentFlag.AlignHCenter
        )
        grid_layout.setColumnStretch(0, 4)
        grid_layout.setColumnStretch(1, 1)

        box = QGroupBox()
        box.setTitle("Controller Status")
        box.setLayout(grid_layout)

        return box

    def __read_controller_inputs(self) -> None:
        """Read and interpret the current controller input values."""

        def with_deadzone(abs_value: float) -> float:
            return (
                abs_value
                if abs(abs_value) >= DiagnosticsConstants.DEADZONE_PERCENTAGE
                else 0
            )

        move_axis_x = with_deadzone(self.handheld_controller.abs_value(ecodes.ABS_Y))
        move_axis_y = with_deadzone(self.handheld_controller.abs_value(ecodes.ABS_X))
        move_axis_rot = with_deadzone(self.handheld_controller.abs_value(ecodes.ABS_RX))

        left_trigger_value = self.handheld_controller.abs_value(ecodes.ABS_Z)
        speed_factor = (
            DiagnosticsConstants.SPEED_SLOWDOWN_FACTOR
            if left_trigger_value > DiagnosticsConstants.BUTTON_PRESSED_THRESHOLD
            else 1
        )

        self.motor_control.direct_velocity_control.velocity.x_component_meters = (
            -move_axis_x * self.constants.robot_max_speed_m_per_s * speed_factor
        )
        self.motor_control.direct_velocity_control.velocity.y_component_meters = (
            -move_axis_y * self.constants.robot_max_speed_m_per_s * speed_factor
        )
        self.motor_control.direct_velocity_control.angular_velocity.radians_per_second = (
            -move_axis_rot * self.constants.robot_max_ang_speed_rad_per_s * speed_factor
        )

        d_pad_axis_x = self.handheld_controller.abs_value(ecodes.ABS_HAT0X)
        d_pad_axis_y = self.handheld_controller.abs_value(ecodes.ABS_HAT0Y)

        if d_pad_axis_x != self.last_d_pad_axis_x_value:
            self.last_d_pad_axis_x_value = d_pad_axis_x
            self.kick_power = numpy.clip(
                a=self.kick_power
                + int(d_pad_axis_x) * DiagnosticsConstants.KICK_POWER_STEPPER,
                a_min=DiagnosticsConstants.MIN_KICK_POWER,
                a_max=DiagnosticsConstants.MAX_KICK_POWER,
            )
            self.chip_distance = numpy.clip(
                a=self.chip_distance
                + d_pad_axis_x * DiagnosticsConstants.CHIP_DISTANCE_STEPPER,
                a_min=DiagnosticsConstants.MIN_CHIP_POWER,
                a_max=DiagnosticsConstants.MAX_CHIP_POWER,
            )

        if d_pad_axis_y != self.last_d_pad_axis_y_value:
            self.last_d_pad_axis_y_value = d_pad_axis_y
            self.dribbler_speed = int(
                numpy.clip(
                    a=self.dribbler_speed
                    - d_pad_axis_y * DiagnosticsConstants.DRIBBLER_RPM_STEPPER,
                    a_min=self.constants.indefinite_dribbler_speed_rpm,
                    a_max=-self.constants.indefinite_dribbler_speed_rpm,
                )
            )

        right_trigger_value = self.handheld_controller.abs_value(ecodes.ABS_RZ)
        self.motor_control.dribbler_speed_rpm = (
            self.dribbler_speed
            if right_trigger_value > DiagnosticsConstants.BUTTON_PRESSED_THRESHOLD
            else 0
        )

        btn_a = self.handheld_controller.key_down(ecodes.BTN_A)
        btn_b = self.handheld_controller.key_down(ecodes.BTN_B)

        if btn_a != self.last_btn_a_value:
            self.last_btn_a_value = btn_a
            if btn_a:
                self.kick_button_pressed.emit()

        if btn_b != self.last_btn_b_value:
            self.last_btn_b_value = btn_b
            if btn_b:
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
