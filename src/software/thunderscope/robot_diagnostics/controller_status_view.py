from enum import Enum

import pyqtgraph as pg
from proto.import_all_protos import *
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *

import software.thunderscope.common.common_widgets as common_widgets
from software.py_constants import *


class ControllerConnectionState(Enum):
    CONNECTED = 1
    DISCONNECTED = 2


class ControllerStatusView(QLabel):
    """Class to show whether a handheld controller is connected to thunderscope and initialized,
    or no controller is connected at all.
    """

    def __init__(self) -> None:
        super().__init__()

        self.state: dict[ControllerConnectionState, (str, str)] = {
            ControllerConnectionState.CONNECTED: (
                "Handheld Controller is Connected & Initialized",
                "background-color: green",
            ),
            ControllerConnectionState.DISCONNECTED: (
                "No Handheld Controller is Connected...",
                "background-color: red",
            ),
        }

        self.connected = False
        self.set_view_state(ControllerConnectionState.DISCONNECTED)

    def set_view_state(
        self, state_discriminator=ControllerConnectionState.DISCONNECTED
    ):
        self.setText(self.state[state_discriminator][0])
        self.setStyleSheet(self.state[state_discriminator][1])

    def refresh(self, connected=ControllerConnectionState.DISCONNECTED) -> None:
        """Refresh the label
        """
        self.set_view_state(connected)
