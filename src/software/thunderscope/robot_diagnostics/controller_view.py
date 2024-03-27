import pyqtgraph as pg
from proto.import_all_protos import *
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *

import software.thunderscope.common.common_widgets as common_widgets
from software.py_constants import *


class ControllerStatusView(QLabel):
    """Class to show whether a handheld controller is connected to thunderscope and initialized,
    or no controller is connected at all.
    """

    def __init__(self) -> None:
        super().__init__()

        # TODO align text center for status
        self.state: dict[str, (str, str)] = {
            "On": ("Handheld Controller is Connected & Initialized", "background-color: green"),
            "Off": ("No Handheld Controller is Connected...", "background-color: red")
        }

        self.connected = False
        self.set_view_state('Off')

    def set_view_state(self, state_discriminator='Off'):
        # bruh python doesn't even have value-types or unions
        # how do you even do anything in this language and still maintain a sanity ffs i legit cant
        self.setText(self.state[state_discriminator][0])
        self.setStyleSheet(self.state[state_discriminator][1])

    def set_connected(self):
        self.connected = True
        self.set_view_state('On')

    def set_disconnected(self):
        self.connected = False
        self.set_view_state('Off')

    def refresh(self) -> None:
        """Refresh the label
        """

        if self.connected:
            self.set_view_state('Ok')
        else:
            self.set_view_state('Off')
