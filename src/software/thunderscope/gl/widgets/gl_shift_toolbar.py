import textwrap
from pyqtgraph.Qt import QtGui
from pyqtgraph.Qt.QtCore import pyqtSignal
from pyqtgraph.Qt.QtWidgets import *
from proto.import_all_protos import *
from software.thunderscope.gl.widgets.gl_toolbar import GLToolbar

class ShiftButtonToolbar(GLToolbar):
    enable_ball_movement_signal = pyqtSignal(bool)

    def __init__(self, parent) -> None:
        super().__init__(parent)
        self.menu = QMenu()
        self.push_button = QPushButton()
        # TODO: make this a static method!
        button_style = textwrap.dedent(
            f"""
            QPushButton {{
                color: #969696;
                background-color: transparent;
                border-color: transparent;
                icon-size: 22px;
                border-width: 4px;
                border-radius: 4px;
                height: 16px;
            }}
            QPushButton:hover {{
                background-color: {"#363636"};
                border-color: {"#363636"};
            }}
            """
        )
        self.push_button.setStyleSheet(button_style)
        self.push_button.setMenu(self.menu)

        self.actions = [
            QtGui.QAction("[1] Enable Shift Click Ball Move"),
            QtGui.QAction("[2] Disable Shift Click Ball Move"),
        ]

        self.actions[0].triggered.connect(self.enable_ball_movement)
        self.actions[1].triggered.connect(self.disable_ball_movement)

        for action in self.actions:
            self.menu.addAction(action)

        self.layout().addWidget(self.push_button)
        
        # enable ball by default, just in case
        self.enable_ball_movement_signal.emit(True)
        self.push_button.setText("[1] Enable Shift Click Ball Move")

    def disable_ball_movement(self):
        self.push_button.setText("[2] Disable Shift Click Ball Move")
        self.enable_ball_movement_signal.emit(False)

    def enable_ball_movement(self):
        self.push_button.setText("[1] Enable Shift Click Ball Move")
        self.enable_ball_movement_signal.emit(True)

