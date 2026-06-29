from pyqtgraph.Qt import QtCore
from pyqtgraph.Qt.QtWidgets import *


class GLToolbar(QWidget):
    """Base class for a toolbar in our UI

    Has a refresh method to update UI
    Has button colors / styles and initializes base formatting
    """

    BUTTON_ICON_COLOR = "white"
    DISABLED_BUTTON_ICON_COLOR = "#969696"

    def __init__(self, parent: QWidget):
        """Sets the base formatting for a toolbar

        :param parent: the parent to overlay this toolbar over
        """
        super(GLToolbar, self).__init__(parent=parent)

        # Setup toolbar
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        self.setStyleSheet("background-color: rgba(0,0,0,0);" "padding: 0px;")
        self.setAttribute(QtCore.Qt.WidgetAttribute.WA_StyledBackground)
        self.setLayout(QHBoxLayout())

    def refresh(self) -> None:
        """Refreshes the UI (overridden by child classes)"""
        raise NotImplementedError("Subclasses must implement this method!")
