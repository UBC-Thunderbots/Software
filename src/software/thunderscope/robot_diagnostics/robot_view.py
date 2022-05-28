import pyqtgraph as pg
from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.Qt.QtWidgets import *
from software.python_constants import *
import software.thunderscope.common.common_widgets as common_widgets

from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class RobotView(QWidget):
    """

    """

    def __init__(self):
        self.label = QLabel()

    def draw_something(self):
        painter = QtGui.QPainter(self.label.pixmap())
        pen = QtGui.QPen()
        pen.setWidth(40)
        pen.setColor(QtGui.QColor("red"))
        painter.setPen(pen)
        painter.drawPoint(200, 150)
        painter.end()
