from pyqtgraph.Qt import QtCore
from pyqtgraph.Qt.QtWidgets import *
from software.py_constants import *
import software.thunderscope.common.common_widgets as common_widgets
from enum import IntEnum

from software.thunderscope.robot_diagnostics.handheld_device_status_view import (
    HandheldDeviceConnectionStatus,
)


class ControlMode(IntEnum):
    """Enum for the 2 modes of control (Manual and Xbox)"""

    DIAGNOSTICS = 0
    HANDHELD = 1


class DiagnosticsInputToggleWidget(QWidget):
    """Class to allow the user to switch between Manual, Xbox, and Fullsystem control
    through Thunderscope UI

    Disables Manual controls in the other two modes
    """

    control_mode_changed_signal = QtCore.pyqtSignal(ControlMode)

    def __init__(self) -> None:
        """Initialises a new Fullsystem Connect Widget to allow switching
        between Diagnostics and Xbox control
        """
        super(DiagnosticsInputToggleWidget, self).__init__()

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
        """Update this widget.

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
        return (
            ControlMode.DIAGNOSTICS
            if self.diagnostics_control_button.isChecked()
            else ControlMode.HANDHELD
        )
