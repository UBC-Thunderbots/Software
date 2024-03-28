from typing import Callable

from pyqtgraph.Qt.QtCore import *
from pyqtgraph.Qt.QtWidgets import *
from software.py_constants import *
import software.thunderscope.common.common_widgets as common_widgets
from enum import IntEnum


class ControlMode(IntEnum):
    """
    Enum for the 2 modes of control (Manual and XBox)
    """

    DIAGNOSTICS = 0
    HANDHELD = 1


# this class name doesnt make sense
#
class DiagnosticsInputModeWidget(QWidget):
    """
    Class to allow the user to switch between Manual, XBox, and Fullsystem control through Thunderscope UI

    Disables Manual controls in the other two modes

    """

    # Signal to indicate if manual controls should be disabled based on boolean parameter

    def __init__(
        self, on_control_mode_switch_callback: Callable[[ControlMode], None]
    ) -> None:
        """
        Initialises a new Fullsystem Connect Widget to allow switching between Diagnostics and XBox control
        :param on_control_mode_switch_callback The signal to use for handling changes in input mode
        """
        super(DiagnosticsInputModeWidget, self).__init__()
        self.on_control_mode_switch_callback = on_control_mode_switch_callback

        vbox_layout = QVBoxLayout()

        self.connect_options_group = QButtonGroup()

        self.connect_options_box, self.connect_options = common_widgets.create_radio(
            ["Diagnostics Control", "Handheld Control"], self.connect_options_group
        )

        self.diagnostics_control_button = self.connect_options[ControlMode.DIAGNOSTICS]
        self.handheld_control_button = self.connect_options[ControlMode.HANDHELD]

        self.diagnostics_control_button.clicked.connect(
            lambda: on_control_mode_switch_callback(ControlMode.DIAGNOSTICS)
        )
        self.handheld_control_button.clicked.connect(
            lambda: on_control_mode_switch_callback(ControlMode.HANDHELD)
        )

        self.handheld_control_button.setEnabled(False)
        self.diagnostics_control_button.setChecked(True)

        vbox_layout.addWidget(self.connect_options_box)

        self.setLayout(vbox_layout)

    def enable_handheld(self):
        self.handheld_control_button.setEnabled(True)

    def disable_handheld(self):
        self.handheld_control_button.setEnabled(False)

    def refresh(self, mode=ControlMode.DIAGNOSTICS) -> None:
        self.handheld_control_button.setEnabled(mode == ControlMode.HANDHELD)
