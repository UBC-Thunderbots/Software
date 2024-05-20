from enum import Enum
from typing import Type

import pyqtgraph as pg

from proto.import_all_protos import *
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt.QtCore import *

import software.thunderscope.common.common_widgets as common_widgets
from software.py_constants import *


class HandheldDeviceConnectionStatus(Enum):
    CONNECTED = 1
    DISCONNECTED = 2


class HandheldDeviceStatusView(QWidget):
    def __init__(
        self, reinitialize_handheld_device_signal: Type[QtCore.pyqtSignal]
    ) -> None:
        """
        Initialize the HandheldDeviceStatusView widget.
        This widget shows the user the current state of the connection with a handheld device,
        as well as a button that attempts to reinitialize a handheld device object when clicked
        :param reinitialize_handheld_device_signal: The signal to use for the reinitialize button
        """
        super(HandheldDeviceStatusView, self).__init__()

        self.reinitialize_handheld_device_signal = reinitialize_handheld_device_signal

        self.handheld_device_status = QLabel()
        self.handheld_device_status.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.status_label_view_map: dict[HandheldDeviceConnectionStatus, (str, str)] = {
            HandheldDeviceConnectionStatus.CONNECTED: (
                "Handheld Device is Connected",
                "background-color: green",
            ),
            HandheldDeviceConnectionStatus.DISCONNECTED: (
                "Handheld Device is Disconnected...",
                "background-color: red",
            ),
        }

        # initialize device refresh button
        self.handheld_device_reinitialize_button = QPushButton()
        self.handheld_device_reinitialize_button.setText(
            "Re-initialize Handheld Device"
        )
        self.handheld_device_reinitialize_button.clicked.connect(
            self.reinitialize_handheld_device_signal
        )

        box = QGroupBox()
        hbox_layout = QHBoxLayout()
        hbox_layout.setContentsMargins(0, 0, 0, 0)
        box.setTitle("Handheld Device Status and Re-initialization")
        widget_layout = QVBoxLayout()

        hbox_layout.addWidget(self.handheld_device_status)
        hbox_layout.addWidget(self.handheld_device_reinitialize_button)
        hbox_layout.setStretch(0, 4)
        hbox_layout.setStretch(1, 1)

        box.setLayout(hbox_layout)
        widget_layout.addWidget(box)

        self.set_view_state(HandheldDeviceConnectionStatus.DISCONNECTED)
        self.setLayout(widget_layout)

    def set_view_state(self, connection_state: HandheldDeviceConnectionStatus) -> None:
        """
        Sets the label to display the correct status depending on the connection state
        :param connection_state: The state to use
        """
        self.handheld_device_status.setText(
            self.status_label_view_map[connection_state][0]
        )
        self.handheld_device_status.setStyleSheet(
            self.status_label_view_map[connection_state][1]
        )

    def refresh(
        self, connection_state=HandheldDeviceConnectionStatus.DISCONNECTED
    ) -> None:
        """
        Refreshes this widget.
        The status label will reflect to the user the current state of the handheld device connection
        :param connection_state: The new state of the handheld device connection
        """
        self.set_view_state(connection_state)
