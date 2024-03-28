import logging
from enum import Enum

import pyqtgraph as pg
from proto.import_all_protos import *
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *

import software.thunderscope.common.common_widgets as common_widgets
from software.py_constants import *


class ControllerConnected(Enum):
    CONNECTED = 1
    DISCONNECTED = 2


class ControllerStatusView(QLabel):
    """Class to show whether a handheld controller is connected to thunderscope and initialized,
    or no controller is connected at all.
    """

    def __init__(self) -> None:
        super().__init__()

        self.state: dict[ControllerConnected, (str, str)] = {
            ControllerConnected.CONNECTED: (
                "Handheld Controller is Connected & Initialized",
                "background-color: green",
            ),
            ControllerConnected.DISCONNECTED: (
                "No Handheld Controller is Connected...", "background-color: red"
            ),
        }

        self.connected = False
        self.set_view_state(ControllerConnected.DISCONNECTED)

    def set_view_state(self, state_discriminator=ControllerConnected.DISCONNECTED):
        # bruh python doesn't even have value-types or unions
        # how do you even do anything in this language and still maintain a sanity ffs i legit can't
        self.setText(self.state[state_discriminator][0])
        self.setStyleSheet(self.state[state_discriminator][1])

    def refresh(self, connected=ControllerConnected.DISCONNECTED) -> None:
        """Refresh the label
        """
        self.set_view_state(connected)
