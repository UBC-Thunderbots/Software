from pyqtgraph.Qt.QtCore import *
from pyqtgraph.Qt.QtWidgets import *
from software.py_constants import *
import software.thunderscope.common.common_widgets as common_widgets
from enum import Enum


class ControlMode(Enum):
    MANUAL = 1
    XBOX = 2
    AI = 3


class FullSystemConnectWidget(QWidget):

    toggle_controls_signal = pyqtSignal(bool)

    def __init__(self, proto_unix_io, load_fullsystem):
        """
        Initialises a new Fullsystem Connect Widget to allow switching between Manual, XBox, and Fullsystem control
        :param proto_unix_io: The proto_unix_io object
        :param load_fullsystem: Whether the fullsystem is being loaded currently
        """

        super(FullSystemConnectWidget, self).__init__()

        vbox_layout = QVBoxLayout()
        self.connect_options_group = QButtonGroup()

        radio_button_names = ["Manual Control", "XBox Control"]

        if load_fullsystem:
            radio_button_names.append("AI Control")

        self.connect_options_box, self.connect_options = common_widgets.create_radio(
            radio_button_names, self.connect_options_group
        )

        self.manual_control_button = self.connect_options[0]
        self.xbox_control_button = self.connect_options[1]

        if load_fullsystem:
            self.ai_control_button = self.connect_options[2]
            self.ai_control_button.toggled.connect(
                lambda: self.switch_control_mode(ControlMode.AI)
            )

        self.manual_control_button.toggled.connect(
            lambda: self.switch_control_mode(ControlMode.MANUAL)
        )
        self.xbox_control_button.toggled.connect(
            lambda: self.switch_control_mode(ControlMode.XBOX)
        )

        self.manual_control_button.setChecked(True)
        self.control_mode = ControlMode.MANUAL

        vbox_layout.addWidget(self.connect_options_box)

        self.setLayout(vbox_layout)

    def switch_control_mode(self, mode):
        self.control_mode = mode

    def refresh(self):
        if self.control_mode == ControlMode.XBOX or self.control_mode == ControlMode.AI:
            self.toggle_controls_signal.emit(False)
        else:
            self.toggle_controls_signal.emit(True)
