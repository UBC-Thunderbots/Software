import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
from pyqtgraph.Qt.QtWidgets import *
from software.python_constants import *
import software.thunderscope.common.common_widgets as common_widgets

from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class RobotView(QWidget):
    """Class to show a snapshot of the robot's current state.

    Displays the vision pattern, capacitor/battery voltages,
    and other information about the robot state.

    """

    def __init__(self):

        """Initialize the robot view."""

        super().__init__()
        self.label = QLabel()
        self.layout = QVBoxLayout()
        self.layout.addWidget(self.label)
        self.setLayout(self.layout)

    def draw_robot_view(self, robot_status):
        """Draw the robot view with the given robot status.

        :param robot_status: The robot status

        """

        pixmap = QtGui.QPixmap(self.label.size())
        pixmap.fill(QtCore.Qt.GlobalColor.transparent)

        painter = QtGui.QPainter(pixmap)
        self.draw_robot(painter, 1, 100, 100, 50)
        painter.end()

        self.label.setPixmap(pixmap)

    def draw_robot(self, painter, id, x, y, radius):
        """Draw a robot with the given painter. The vision pattern
        is drawn ontop of the robot.

        :param id: The robot
        :param x: The x position
        :param y: The y position
        :param radius: The radius of the robot

        """
        painter.setPen(pyqtgraph.mkPen("black"))
        painter.setBrush(pyqtgraph.mkBrush("black"))

        convert_degree = -16

        painter.drawChord(
            QtCore.QRectF(
                int(x - radius),
                int(y - radius),
                int(radius * 2),
                int(radius * 2),
            ),
            -45 * convert_degree,
            270 * convert_degree,
        )

        painter.setPen(pyqtgraph.mkPen("pink"))
        painter.setBrush(pyqtgraph.mkBrush("pink"))

        painter.drawEllipse(
            QtCore.QPointF(x-radius, y-radius), radius, radius)

