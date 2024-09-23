from enum import Enum


from proto.import_all_protos import *
from pyqtgraph.Qt import QtCore
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt.QtCore import *

from software.py_constants import *


class HandheldDeviceConnectionStatus(Enum):
    CONNECTED = 1
    DISCONNECTED = 2


class HandheldDeviceStatusView(QWidget):
    reinitialize_handheld_device_signal = QtCore.pyqtSignal()

    def __init__(self) -> None:
        """Initialize the HandheldDeviceStatusView widget.
        This widget shows the user the current state of the connection with a handheld device,
        as well as a button that attempts to reinitialize a handheld device object when clicked
        """
        super(HandheldDeviceStatusView, self).__init__()

        self.handheld_device_status = QLabel()
        self.handheld_device_status.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.status_label_view_map = {
            HandheldDeviceConnectionStatus.CONNECTED: (
                "Handheld Device is Connected",
                "background-color: green",
            ),
            HandheldDeviceConnectionStatus.DISCONNECTED: (
                "Handheld Device is Disconnected...",
                "background-color: red",
            ),
        }

        self.handheld_device_reinitialize_button = QPushButton()
        self.handheld_device_reinitialize_button.setText("Detect Handheld Device")
        self.handheld_device_reinitialize_button.clicked.connect(
            self.reinitialize_handheld_device_signal
        )

        box = QGroupBox()
        hbox_layout = QHBoxLayout()
        box.setTitle("Handheld Device Status")
        widget_layout = QVBoxLayout()

        hbox_layout.addWidget(self.handheld_device_status)
        hbox_layout.addWidget(self.handheld_device_reinitialize_button)
        hbox_layout.setStretch(0, 4)
        hbox_layout.setStretch(1, 1)

        box.setLayout(hbox_layout)
        widget_layout.addWidget(box)
        self.setLayout(widget_layout)

        self.update(HandheldDeviceConnectionStatus.DISCONNECTED)

    def update(self, connection_status: HandheldDeviceConnectionStatus) -> None:
        """Sets the label to display the correct status depending on the connection status

        :param connection_status: the connection status to display
        """
        text, color = self.status_label_view_map[connection_status]
        self.handheld_device_status.setText(text)
        self.handheld_device_status.setStyleSheet(color)
