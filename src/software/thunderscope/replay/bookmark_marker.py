from collections.abc import Callable

from PyQt6.QtCore import QRect
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import QPushButton, QSlider
from software.py_constants import *


class BookmarkMarker(QPushButton):
    """Bookmark Visual"""

    # Color of the visual
    MARKER_COLOR = QColor(130, 130, 130)

    # Radius of the visual
    MARKER_RADIUS = 5

    def __init__(
        self,
        value: float,
        click_func: Callable[[float], None],
        slider: QSlider,
        parent=None,
    ):
        """Create a bookmark visual
        :param value: timestamp of the bookmark
        :param click_func: callback when the bookmark is clicked
        :param slider: slider object to bind the bookmark with
        :param parent: parent of the current qt widget
        """
        super(BookmarkMarker, self).__init__(parent)
        self.value = value
        self.slider = slider

        self.clicked.connect(lambda _: click_func(value))
        self.setGeometry(
            0, 0, BookmarkMarker.MARKER_RADIUS * 2, BookmarkMarker.MARKER_RADIUS * 2
        )
        self.setStyleSheet(
            f"border : 2px solid {BookmarkMarker.MARKER_COLOR.name()}; color:{BookmarkMarker.MARKER_COLOR.name()}; border-radius: {BookmarkMarker.MARKER_RADIUS}px"
        )

        self.update()

    def update(self):
        """Update bookmark visuals display"""
        super().update()
        # Re-calculate the position of the visuals
        slider_rect: QRect = self.slider.geometry()
        max_val = self.slider.maximum() / MILLISECONDS_PER_SECOND
        self.move(int(slider_rect.width() * self.value // max_val), 42)
