from PyQt6 import QtCore, QtGui
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import QLabel


class BookmarkMarker(QLabel):
    MARKER_COLOR = QColor(130, 130, 130)

    def __init__(self, value, parent=None):
        super(BookmarkMarker, self).__init__(parent)
        self.value = value

        pix = QtGui.QPixmap(30, 30)
        pix.fill(QtGui.QColor("transparent"))
        paint = QtGui.QPainter(pix)
        handle_pen = QtGui.QPen(QtGui.QColor(self.MARKER_COLOR.darker(200)))
        handle_pen.setWidth(3)
        paint.setPen(handle_pen)
        paint.setBrush(QtGui.QBrush(self.MARKER_COLOR))
        points = QtGui.QPolygon([
            QtCore.QPoint(7, 5),
            QtCore.QPoint(7, 19),
            QtCore.QPoint(15, 27),
            QtCore.QPoint(22, 19),
            QtCore.QPoint(22, 5),

        ])
        paint.drawPolygon(points)
        self.setPixmap(pix)

        self.adjustSize()