import textwrap
from pyqtgraph.Qt import QtGui, QtCore
from pyqtgraph.Qt.QtWidgets import *


class GLToolbar(QWidget):

    BUTTON_ICON_COLOR = "white"

    def __init__(self):
        super(GLToolbar, self).__init__()

        # Setup toolbar
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        self.setStyleSheet("background-color: black;" "padding: 0px;")
        self.setAttribute(QtCore.Qt.WidgetAttribute.WA_StyledBackground)
        self.setLayout(QHBoxLayout())

    def refresh(self):
        pass

    def get_button_style(self, is_enabled: bool = True) -> str:
        """
        Returns the stylesheet for a QPushButton based on if it's enabled or not
        :param is_enabled: True if button is enabled, False if not
        :return: the corresponding stylesheet indicating the button state
        """
        # the style for each toolbar button
        return textwrap.dedent(
            f"""
            QPushButton {{
                background-color: transparent;
                border-color: transparent;
                icon-size: 22px;
                border-width: 4px;
                border-radius: 4px;
                height: 16px;
            }}
            QPushButton:hover {{
                background-color: {"#363636" if is_enabled else "transparent"};
                border-color: {"#363636" if is_enabled else "transparent"};
            }}
            """
        )
