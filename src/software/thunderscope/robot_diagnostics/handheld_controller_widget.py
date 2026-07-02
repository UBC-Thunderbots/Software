import os

# Suppress the pygame "Hello from the pygame community" banner on import
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"

import pygame

from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt import QtCore

from software.thunderscope.constants import DiagnosticsConstants
from software.thunderscope.robot_diagnostics.handheld_controller import (
    HandheldController,
)
from software.thunderscope.robot_diagnostics.manual_input_widget import (
    ManualInputWidget,
)


class HandheldControllerWidget(ManualInputWidget):
    """Input source that drives a robot from a connected handheld game
    controller (e.g. an Xbox gamepad).

    Displays the controller's connection status and lets the user (re)scan for
    a controller. Reads the controller's analog sticks, triggers, D-pad and
    face buttons and feeds them through the shared
    :meth:`ManualInputWidget._apply_inputs` mapping.
    """

    def __init__(self) -> None:
        """Initialize the HandheldControllerWidget."""
        super().__init__()

        self.handheld_controller: HandheldController | None = None

        widget_layout = QVBoxLayout()
        widget_layout.setContentsMargins(0, 0, 0, 0)
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

    def is_connected(self) -> bool:
        """Check whether a handheld controller is currently connected.

        :return: true if a controller is connected, false otherwise
        """
        return (
            self.handheld_controller is not None
            and self.handheld_controller.connected()
        )

    def refresh_status(self) -> None:
        """Refresh the controller's connection state and status display.

        Should be called every refresh tick while the controller is the
        selected source so the UI reflects connects and disconnects.
        """
        if self.handheld_controller is None:
            return

        self.handheld_controller.update()

        if not self.handheld_controller.connected():
            self.handheld_controller.close()
            self.handheld_controller = None
            self.__update_controller_status()

    def poll(self) -> None:
        """Read the controller's inputs and update the manual control outputs."""
        if not self.is_connected():
            return

        controller = self.handheld_controller

        slow_mode = (
            controller.abs_value(pygame.CONTROLLER_AXIS_TRIGGERLEFT)
            > DiagnosticsConstants.BUTTON_PRESSED_THRESHOLD
        )
        dribbler_on = (
            controller.abs_value(pygame.CONTROLLER_AXIS_TRIGGERRIGHT)
            > DiagnosticsConstants.BUTTON_PRESSED_THRESHOLD
        )

        # SDL exposes the D-pad as four buttons; synthesize hat axis values
        # (right/down = +1, left/up = -1) to match the previous behaviour.
        d_pad_axis_x = controller.key_down(
            pygame.CONTROLLER_BUTTON_DPAD_RIGHT
        ) - controller.key_down(pygame.CONTROLLER_BUTTON_DPAD_LEFT)
        d_pad_axis_y = controller.key_down(
            pygame.CONTROLLER_BUTTON_DPAD_DOWN
        ) - controller.key_down(pygame.CONTROLLER_BUTTON_DPAD_UP)

        self._apply_inputs(
            move_x=controller.abs_value(pygame.CONTROLLER_AXIS_LEFTY),
            move_y=controller.abs_value(pygame.CONTROLLER_AXIS_LEFTX),
            move_rot=controller.abs_value(pygame.CONTROLLER_AXIS_RIGHTX),
            slow_mode=slow_mode,
            dribbler_on=dribbler_on,
            d_pad_axis_x=d_pad_axis_x,
            d_pad_axis_y=d_pad_axis_y,
            kick=controller.key_down(pygame.CONTROLLER_BUTTON_A),
            chip=controller.key_down(pygame.CONTROLLER_BUTTON_B),
        )

    def __create_widgets(self) -> QWidget:
        """Create the widgets that make up the HandheldControllerWidget UI.

        :return: a QWidget containing:
            - a QLabel for displaying the current controller connection status
            - a QPushButton that tries detecting a controller when pressed
        """
        self.controller_status_label = QLabel()
        self.controller_status_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)

        self.detect_controller_button = QPushButton("Detect Controller")
        self.detect_controller_button.clicked.connect(self.detect_controller)

        layout = QHBoxLayout()
        layout.addWidget(self.controller_status_label, stretch=4)
        layout.addWidget(self.detect_controller_button, stretch=1)

        panel = QWidget()
        panel.setLayout(layout)
        return panel

    def __update_controller_status(self) -> None:
        """Update the widget to display the current controller connection status."""
        if self.is_connected():
            self.controller_status_label.setText(
                f"Connected: {self.handheld_controller.name()}"
            )
            self.controller_status_label.setStyleSheet("background-color: green")
        else:
            self.controller_status_label.setText("Disconnected")
            self.controller_status_label.setStyleSheet("background-color: red")
