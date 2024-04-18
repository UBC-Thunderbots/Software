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
    """Class to show whether a handheld controller is connected to thunderscope and initialized,
    or no controller is connected at all.
    """

    def __init__(self, reinitialize_controller_signal: Type[QtCore.pyqtSignal]) -> None:
        super().__init__()

        self.reinitialize_controller_signal = reinitialize_controller_signal

        self.handheld_device_status = QLabel()

        self.status_label_view_map: dict[HandheldDeviceConnectionStatus, (str, str)] = {
            HandheldDeviceConnectionStatus.CONNECTED: (
                "Handheld Controller is Connected & Initialized",
                "background-color: green",
            ),
            HandheldDeviceConnectionStatus.DISCONNECTED: (
                "No Handheld Controller is Connected...",
                "background-color: red",
            ),
        }


        # initialize controller refresh button
        self.handheld_device_reinitialize_button = QPushButton()
        self.handheld_device_reinitialize_button.setText(
            "Re-initialize Handheld Controller"
        )
        self.handheld_device_reinitialize_button.clicked.connect(
            self.reinitialize_controller_signal
        )

        # layout for the whole widget
        self.vbox_layout = QVBoxLayout()

        # layout for controller button & status
        self.hbox_layout = QHBoxLayout()

        # set layout and spacing
        self.hbox_layout.addWidget(self.handheld_device_status)
        self.hbox_layout.addWidget(self.handheld_device_reinitialize_button)
        self.hbox_layout.setStretch(0, 4)
        self.hbox_layout.setStretch(1, 1)

        # set groupbox to contain layout with status and button
        # self.group_box.setLayout(self.hbox_layout)

        # add box to whole widget layout
        # self.vbox_layout.addWidget(self.group_box)

        # set the layout for the whole widget
        self.setLayout(self.hbox_layout)
        self.set_view_state(HandheldDeviceConnectionStatus.DISCONNECTED)

    def set_view_state(
        self, connection_state=HandheldDeviceConnectionStatus.DISCONNECTED
    ):
        self.handheld_device_status.setText(
            self.status_label_view_map[connection_state][0]
        )
        self.handheld_device_status.setStyleSheet(
            self.status_label_view_map[connection_state][1]
        )

    def refresh(self, connected=HandheldDeviceConnectionStatus.DISCONNECTED) -> None:
        """Refresh the label
        """
        self.set_view_state(connected)
