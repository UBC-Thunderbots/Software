from collections.abc import Callable

from PyQt6 import QtCore, QtGui
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import QLabel


class BookmarkMarker(QLabel):
    """
    A BookmarkMarker represents
    """
    MARKER_COLOR = QColor(130, 130, 130)
    MARKER_DIAMETER = 10


    def __init__(self, value: float, click_func: Callable[[float], None], parent=None):
        super(BookmarkMarker, self).__init__(parent)
        self.value = value
        self.clicked.connect(lambda: click_func(self.value))
        self.resize(BookmarkMarker.MARKER_DIAMETER, BookmarkMarker.MARKER_DIAMETER)

    def update(self):
        pass
