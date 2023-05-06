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
    XBOX = 1


class FullSystemConnectWidget(QWidget):
    """
    Class to allow the user to switch between Manual, XBox, and Fullsystem control through Thunderscope UI

    Disables Manual controls in the other two modes

    """

    # Signal to indicate if manual controls should be disabled based on boolean parameter
    toggle_controls_signal = pyqtSignal(bool)

    def __init__(self):
        """
        Initialises a new Fullsystem Connect Widget to allow switching between Diagnostics and XBox control
        """

        super(FullSystemConnectWidget, self).__init__()

        vbox_layout = QVBoxLayout()
        self.connect_options_group = QButtonGroup()

        radio_button_names = ["Diagnostics Control", "XBox Control"]

        self.connect_options_box, self.connect_options = common_widgets.create_radio(
            radio_button_names, self.connect_options_group
        )

        self.diagnostics_control_button = self.connect_options[ControlMode.DIAGNOSTICS]
        self.xbox_control_button = self.connect_options[ControlMode.XBOX]

        self.diagnostics_control_button.clicked.connect(
            lambda: self.switch_control_mode(ControlMode.DIAGNOSTICS)
        )
        self.xbox_control_button.clicked.connect(
            lambda: self.switch_control_mode(ControlMode.XBOX)
        )

        self.diagnostics_control_button.setChecked(True)
        self.control_mode = ControlMode.DIAGNOSTICS

        vbox_layout.addWidget(self.connect_options_box)

        self.setLayout(vbox_layout)

    def switch_control_mode(self, mode):
        """
        Switches the control mode to the given mode

        Emits a signal to indicate whether diagnostics controls should be disabled or not

        :param mode: mode to switch to (one of ControlMode values)

        """
        self.control_mode = mode

        self.toggle_controls_signal.emit(self.control_mode == ControlMode.DIAGNOSTICS)

    def refresh(self):
        pass
