from pyqtgraph.Qt import QtGui
from pyqtgraph.Qt.QtCore import pyqtSignal
from pyqtgraph.Qt.QtWidgets import *
from proto.import_all_protos import *
from software.thunderscope.gl.widgets.gl_toolbar import GLToolbar


class ShiftButtonToolbar(GLToolbar):
    """A shift button toolbar that is going to be used to enable/disable shift click ball placement in Thunderscope"""

    enable_ball_placement_signal = pyqtSignal(bool)

    def __init__(self, parent: QWidget) -> None:
        """Initialize the ShiftButtonToolbar

        :param parent: the parent of this widget
        """
        super().__init__(parent)
        self.menu = QMenu()
        self.push_button = QPushButton()
        # TODO: make this a static method!
        self.push_button.setStyleSheet(GLToolbar.get_button_style())
        self.push_button.setMenu(self.menu)

        self.actions = [
            QtGui.QAction("[1] Enable Shift Click Ball Move"),
            QtGui.QAction("[2] Disable Shift Click Ball Move"),
        ]

        self.actions[0].triggered.connect(self.enable_ball_placement)
        self.actions[1].triggered.connect(self.disable_ball_placement)

        for action in self.actions:
            self.menu.addAction(action)

        self.layout().addWidget(self.push_button)

        # enable ball by default, just in case
        self.enable_ball_placement_signal.emit(True)
        self.push_button.setText("[1] Enable Shift Click Ball Move")

    def disable_ball_placement(self):
        """Disable shift click ball placement"""
        self.push_button.setText("[2] Disable Shift Click Ball Move")
        self.enable_ball_placement_signal.emit(False)

    def enable_ball_placement(self):
        """Enable shift click ball placement"""
        self.push_button.setText("[1] Enable Shift Click Ball Move")
        self.enable_ball_placement_signal.emit(True)
