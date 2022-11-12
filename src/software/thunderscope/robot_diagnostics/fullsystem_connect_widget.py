from pyqtgraph.Qt.QtCore import *
from pyqtgraph.Qt.QtWidgets import *
from software.py_constants import *
import software.thunderscope.common.common_widgets as common_widgets
from enum import Enum


class ControlMode(Enum):
    AI = 1
    MANUAL = 2
    XBOX = 3


class FullSystemConnectWidget(QWidget):
    def __init__(self, proto_unix_io, drive_widget):

        super(FullSystemConnectWidget, self).__init__()

        vbox_layout = QVBoxLayout()
        self.connect_options_group = QButtonGroup()

        self.connect_options_box, self.connect_options = common_widgets.create_radio(
            ["AI Control", "Manual Control", "XBox Control"], self.connect_options_group
        )

        self.ai_control_button = self.connect_options[0]
        self.manual_control_button = self.connect_options[1]
        self.xbox_control_button = self.connect_options[2]

        self.ai_control_button.toggled.connect(lambda: self.switch_control_mode(ControlMode.AI))
        self.manual_control_button.toggled.connect(lambda: self.switch_control_mode(ControlMode.MANUAL))
        self.xbox_control_button.toggled.connect(lambda: self.switch_control_mode(ControlMode.XBOX))

        self.ai_control_button.setChecked(True)
        self.control_mode = ControlMode.AI

        vbox_layout.addWidget(self.connect_options_box)

        self.drive_widget = drive_widget

        self.setLayout(vbox_layout)

    def switch_control_mode(self, mode):
        self.control_mode = mode

    def refresh(self):
        if self.control_mode == ControlMode.AI or self.control_mode == ControlMode.XBOX:
            self.drive_widget.toggle_all(False)
        else:
            self.drive_widget.toggle_all(True)





