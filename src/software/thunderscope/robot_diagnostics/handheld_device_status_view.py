from enum import Enum


from proto.import_all_protos import *
from pyqtgraph.Qt import QtCore
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt.QtCore import *

from software.py_constants import *


class HandheldDeviceConnectionStatus(Enum):
    """Enum representing whether a handheld device is connected or disconnected"""

    CONNECTED = 1
    DISCONNECTED = 2


class HandheldDeviceStatusView(QWidget):
    """Widget that shows the user the current state of the connection with a handheld device,
    as well as a button that attempts to detect and reinitialize a handheld device when pressed
    """

    reinitialize_handheld_device_signal = QtCore.pyqtSignal()
    """Signal emitted when the "Detect Handheld Device" button is pressed"""

    def __init__(self) -> None:
        """Initialize the HandheldDeviceStatusView"""
        super(HandheldDeviceStatusView, self).__init__()

        self.handheld_device_status = QLabel()
        self.handheld_device_status.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.detect_handheld_device_button = QPushButton()
        self.detect_handheld_device_button.setText("Detect Handheld Device")
        self.detect_handheld_device_button.setSizePolicy(
            QSizePolicy.Policy.Expanding, 
            QSizePolicy.Policy.Expanding,
        )
        self.detect_handheld_device_button.clicked.connect(
            self.reinitialize_handheld_device_signal
        )

        box = QGroupBox()
        hbox_layout = QHBoxLayout()
        box.setTitle("Handheld Device Status")
        widget_layout = QVBoxLayout()

        hbox_layout.addWidget(self.handheld_device_status)
        hbox_layout.addWidget(self.detect_handheld_device_button)
        hbox_layout.setStretch(0, 4)
        hbox_layout.setStretch(1, 1)

        box.setLayout(hbox_layout)
        widget_layout.addWidget(box)
        self.setLayout(widget_layout)

        self.update(HandheldDeviceConnectionStatus.DISCONNECTED, None)

    def update(
        self, connection_status: HandheldDeviceConnectionStatus, device_name: str | None
    ) -> None:
        """Sets the label to display the correct status depending on the connection status

        :param connection_status: the connection status to display
        :param device_name: the name of the handheld device that was connected
        """
        if connection_status == HandheldDeviceConnectionStatus.CONNECTED:
            self.handheld_device_status.setText(f"Connected: {device_name}")
            self.handheld_device_status.setStyleSheet("background-color: green")
        elif connection_status == HandheldDeviceConnectionStatus.DISCONNECTED:
            self.handheld_device_status.setText("Disconnected")
            self.handheld_device_status.setStyleSheet("background-color: red")
