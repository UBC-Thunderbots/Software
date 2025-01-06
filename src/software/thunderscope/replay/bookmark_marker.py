from collections.abc import Callable

from PyQt6.QtCore import QSize
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import QPushButton


class BookmarkMarker(QPushButton):
    """
    Bookmark Visual
    """
    # Color of the visual
    MARKER_COLOR = QColor(130, 130, 130)

    # Radius of the visual
    MARKER_RADIUS = 10

    def __init__(self, value: float, click_func: Callable[[float], None], parent=None):
        super(BookmarkMarker, self).__init__(parent)
        self.value = value

        self.clicked.connect(lambda _: click_func(value))
        self.setFixedSize(QSize(BookmarkMarker.MARKER_RADIUS, BookmarkMarker.MARKER_RADIUS))
        self.setStyleSheet(f"border : 1px solid black;background-color : green; color: yellow; border-radius : {BookmarkMarker.MARKER_RADIUS}px")


