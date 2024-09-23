from pyqtgraph.Qt import QtCore
from pyqtgraph.Qt.QtWidgets import *
from software.py_constants import *
import software.thunderscope.common.common_widgets as common_widgets
from enum import IntEnum

from software.thunderscope.robot_diagnostics.handheld_device_status_view import (
    HandheldDeviceConnectionStatus,
)


class ControlMode(IntEnum):
    """Enum representing the available control modes in Robot Diagnostics"""

    DIAGNOSTICS = 0
    HANDHELD = 1


class DiagnosticsInputToggleWidget(QWidget):
    """Widget for switching between manual (Diagnostics) and Xbox (Handheld) control"""

    control_mode_changed_signal = QtCore.pyqtSignal(ControlMode)
    """Signal emitted when the control mode changes"""

    def __init__(self) -> None:
        """Initialise the DiagnosticsInputToggleWidget"""
        super().__init__()

        diagnostics_input_widget_vbox_layout = QVBoxLayout()

        self.connect_options_group = QButtonGroup()

        self.connect_options_box, self.connect_options = common_widgets.create_radio(
            ["Diagnostics Control", "Handheld Control"], self.connect_options_group
        )

        self.connect_options_box.setTitle("Diagnostics Input")

        self.diagnostics_control_button = self.connect_options[ControlMode.DIAGNOSTICS]
        self.handheld_control_button = self.connect_options[ControlMode.HANDHELD]

        self.diagnostics_control_button.clicked.connect(
            lambda: self.control_mode_changed_signal.emit(ControlMode.DIAGNOSTICS)
        )
        self.handheld_control_button.clicked.connect(
            lambda: self.control_mode_changed_signal.emit(ControlMode.HANDHELD)
        )

        # default to diagnostics input, and disable handheld
        self.handheld_control_button.setEnabled(False)
        self.diagnostics_control_button.setChecked(True)

        diagnostics_input_widget_vbox_layout.addWidget(self.connect_options_box)

        self.setLayout(diagnostics_input_widget_vbox_layout)

    def update(self, status: HandheldDeviceConnectionStatus) -> None:
        """Update this widget with the current handheld device connection status. 

        If the handheld device is connected:
            - enables the handheld button
        If the handheld device is disconnected:
            - disables the handheld control button
            - sets the diagnostics button to checked
            - emits diagnostics input change signal

        :param status: the current handheld device connection status
        """
        if status == HandheldDeviceConnectionStatus.CONNECTED:
            self.handheld_control_button.setEnabled(True)

        elif status == HandheldDeviceConnectionStatus.DISCONNECTED:
            self.diagnostics_control_button.setChecked(True)
            self.handheld_control_button.setEnabled(False)
            self.control_mode_changed_signal.emit(ControlMode.DIAGNOSTICS)

    def get_control_mode(self) -> ControlMode:
        """Get the currently selected control mode.
        
        :returns: the currently selected control mode
        """
        return (
            ControlMode.DIAGNOSTICS
            if self.diagnostics_control_button.isChecked()
            else ControlMode.HANDHELD
        )
