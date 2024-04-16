from typing import Callable, Type

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

    def __init__(
        self, diagnostics_input_mode_signal: Type[QtCore.pyqtSignal]
    ) -> None:
        """
        Initialises a new Fullsystem Connect Widget to allow switching between Diagnostics and XBox control
        :param on_control_mode_switch_callback The callback to use for handling changes in input mode
        """
        super(DiagnosticsInputToggleWidget, self).__init__()

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

        self.handheld_control_button.setEnabled(False)
        self.diagnostics_control_button.setChecked(True)

        vbox_layout.addWidget(self.connect_options_box)

        self.setLayout(vbox_layout)

    def refresh(self, status: HandheldDeviceConnectionStatus) -> None:
        self.handheld_control_button.setEnabled(
            status == HandheldDeviceConnectionStatus.CONNECTED
        )
