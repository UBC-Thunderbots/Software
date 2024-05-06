from typing import Type

from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from software.py_constants import *
import software.thunderscope.common.common_widgets as common_widgets
from enum import IntEnum

from software.thunderscope.robot_diagnostics.handheld_device_status_view import (
    HandheldDeviceConnectionStatus,
)


class ControlMode(IntEnum):
    """
    Enum for the 2 modes of control (Manual and XBox)
    """

    DIAGNOSTICS = 0
    HANDHELD = 1


class DiagnosticsInputToggleWidget(QWidget):
    """
    Class to allow the user to switch between Manual, XBox, and Fullsystem control through Thunderscope UI

    Disables Manual controls in the other two modes

    """

    def __init__(self, diagnostics_input_mode_signal: Type[QtCore.pyqtSignal]) -> None:
        """
        Initialises a new Fullsystem Connect Widget to allow switching between Diagnostics and XBox control
        :param diagnostics_input_mode_signal The signal to emit when the input mode changes
        """
        super(DiagnosticsInputToggleWidget, self).__init__()

        self.__control_mode = ControlMode.DIAGNOSTICS

        self.diagnostics_input_mode_signal = diagnostics_input_mode_signal

        vbox_layout = QVBoxLayout()

        self.connect_options_group = QButtonGroup()

        self.connect_options_box, self.connect_options = common_widgets.create_radio(
            ["Diagnostics Control", "Handheld Control"], self.connect_options_group
        )

        self.diagnostics_control_button = self.connect_options[ControlMode.DIAGNOSTICS]
        self.handheld_control_button = self.connect_options[ControlMode.HANDHELD]

        self.diagnostics_control_button.clicked.connect(
            lambda: self.diagnostics_input_mode_signal.emit(ControlMode.DIAGNOSTICS)
        )
        self.handheld_control_button.clicked.connect(
            lambda: self.diagnostics_input_mode_signal.emit(ControlMode.HANDHELD)
        )

        # default to diagnostics input, and disable handheld
        self.handheld_control_button.setEnabled(False)
        self.diagnostics_control_button.setChecked(True)

        vbox_layout.addWidget(self.connect_options_box)

        self.setLayout(vbox_layout)

    def refresh(self, status: HandheldDeviceConnectionStatus) -> None:
        """
        Refresh this widget.
        If the controller is connected:
            - enables the handheld button
        If the controller is disconnected:
            - disables the handheld control button
            - sets the diagnostics button to checked
            - emits diagnostics input change signal

        :param status:
        """
        if status == HandheldDeviceConnectionStatus.CONNECTED:
            self.handheld_control_button.setEnabled(True)

        elif status == HandheldDeviceConnectionStatus.DISCONNECTED:
            self.diagnostics_control_button.setChecked(True)
            self.handheld_control_button.setEnabled(False)
            self.diagnostics_input_mode_signal.emit(ControlMode.DIAGNOSTICS)
