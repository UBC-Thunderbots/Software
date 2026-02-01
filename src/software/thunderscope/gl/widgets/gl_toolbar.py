import textwrap
from pyqtgraph.Qt import QtCore
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt import QtGui
from typing import Callable

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

    def get_button_style(self, is_enabled: bool = True) -> str:
        """Returns the stylesheet for a QPushButton based on if it's enabled or not

        :param is_enabled: True if button is enabled, False if not
        :return: the corresponding stylesheet indicating the button state
        """
        # the style for each toolbar button
        return textwrap.dedent(
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
                background-color: {"#363636" if is_enabled else "transparent"};
                border-color: {"#363636" if is_enabled else "transparent"};
            }}
            """
        )

    def add_separator(self) -> None:
        """Adds a separator line with enough spacing to the layout
        """
        self.layout().addSpacing(10)
        self.add_label("<b>|</b>")
        self.layout().addSpacing(10)

    def setup_icon_button(
            self,
            icon: QtGui.QPixmap,
            tooltip: str,
            callback: Callable[[], None],
            display_text: str = None,
    ) -> QPushButton:
        """Sets up a button with the given name and callback

        :param icon: the icon displayed on the button
        :param tooltip: the tooltip displayed when hovering over the button
        :param callback: the callback for the button click
        :param display_text: optional param if button needs both text and an icon
        :return: the button
        """
        button = QPushButton()
        button.setIcon(icon)
        button.setToolTip(tooltip)
        button.setStyleSheet(self.get_button_style())
        button.clicked.connect(callback)

        if display_text:
            button.setText(display_text)
        return button

    def add_button(self, button: QPushButton) -> None:
        """Adds an already setup button to the toolbar"""
        self.layout().addWidget(button)

    def add_label(self, text: str) -> None:
        """Adds an already setup label to the toolbar"""
        self.layout().addWidget(QLabel(text))