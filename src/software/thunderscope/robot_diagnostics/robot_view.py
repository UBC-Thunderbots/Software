import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
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

        self.pink = QtGui.QColor(255, 0, 255)
        self.green = QtGui.QColor(0, 255, 0)

        self.vision_pattern = QLabel()

        self.vision_pattern.setSizePolicy(
            QSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.MinimumExpanding)
        )
        self.vision_pattern.setGeometry(QtCore.QRect(0, 0, 10, 10))

        # There is no pattern to this so we just have to create
        # mapping from robot id to the four corners of the vision pattern
        #
        # robot-id: top-right, top-left, bottom-left, bottom-right
        #
        # https://robocup-ssl.github.io/ssl-rules/sslrules.html
        self.vision_pattern_lookup = {
            0: (self.pink, self.pink, self.green, self.pink),
            1: (self.pink, self.green, self.green, self.pink),
            2: (self.green, self.green, self.green, self.pink),
            3: (self.green, self.pink, self.green, self.pink),
            4: (self.pink, self.pink, self.pink, self.green),
            5: (self.pink, self.green, self.pink, self.green),
            6: (self.green, self.green, self.pink, self.green),
            7: (self.green, self.pink, self.pink, self.green),
            8: (self.green, self.green, self.green, self.green),
            9: (self.pink, self.pink, self.pink, self.pink),
            10: (self.pink, self.pink, self.green, self.green),
            11: (self.green, self.green, self.pink, self.pink),
            12: (self.pink, self.green, self.green, self.green),
            13: (self.pink, self.green, self.pink, self.pink),
            14: (self.green, self.pink, self.green, self.green),
            15: (self.green, self.pink, self.pink, self.pink),
        }

        self.layout = QVBoxLayout()
        self.layout.addWidget(self.vision_pattern)
        self.setLayout(self.layout)

    def draw_robot_view(self, robot_status):
        """Draw the robot view with the given robot status.

        :param robot_status: The robot status

        """

        pixmap = QtGui.QPixmap(self.size())
        pixmap.fill(QtCore.Qt.GlobalColor.transparent)

        painter = QtGui.QPainter(pixmap)
        self.draw_robot(painter, 1, "blue", 50, 50, 50)
        self.draw_voltage(
            painter,
            x=150,
            y=50,
            width=100,
            voltage=19.2,
            min_voltage=19.2,
            max_voltage=25.2,
        )
        painter.end()

        self.vision_pattern.setPixmap(pixmap)

    def draw_voltage(self, painter, x, y, width, voltage, min_voltage, max_voltage):
        """Draw the given voltage on the given painter.

        :param painter: The painter to draw on
        :param x: The x coordinate of the voltage bar
        :param y: The y coordinate of the voltage bar
        :param width: The height of the voltage bar
        :param voltage: The voltage to draw
        :param min_voltage: The minimum voltage
        :param max_voltage: The maximum voltage

        """
        painter.setPen(pg.mkPen("black", size=2))
        painter.setBrush(pg.mkBrush(None))

        painter.drawRect(QtCore.QRectF(x, y, width, 20),)

        painter.setBrush(pg.mkBrush(self.green))

        painter.drawRect(QtCore.QRectF(x, y, (voltage - min_voltage), 20),)

    def draw_robot(self, painter, id, team_colour, x, y, radius):
        """Draw a robot with the given painter. The vision pattern
        is drawn ontop of the robot.

        :param id: The robot
        :param team_colour: The team colour
        :param x: The x position
        :param y: The y position
        :param radius: The radius of the robot

        """
        painter.setPen(pg.mkPen("white"))
        painter.setBrush(pg.mkBrush("white"))

        painter.drawText(
            QtCore.QPointF(0, 10), "Robot str(id)",
        )

        painter.setPen(pg.mkPen("black"))
        painter.setBrush(pg.mkBrush("black"))

        convert_degree = -16

        painter.drawChord(
            QtCore.QRectF(
                int(x - radius), int(y - radius), int(radius * 2), int(radius * 2),
            ),
            -45 * convert_degree,
            270 * convert_degree,
        )

        painter.setBrush(pg.mkBrush(team_colour))
        painter.drawEllipse(QtCore.QPointF(x, y), radius / 4, radius / 4)

        top_right, top_left, bottom_left, bottom_right = self.vision_pattern_lookup[id]

        painter.setBrush(pg.mkBrush(top_right))
        painter.drawEllipse(
            QtCore.QPointF(x + radius / 2 + 5, y - radius / 2), radius / 5, radius / 5
        )

        painter.setBrush(pg.mkBrush(top_left))
        painter.drawEllipse(
            QtCore.QPointF(x - radius / 2 - 5, y - radius / 2), radius / 5, radius / 5
        )

        painter.setBrush(pg.mkBrush(bottom_left))
        painter.drawEllipse(
            QtCore.QPointF(x - radius / 2, y + radius / 2 + 5), radius / 5, radius / 5
        )

        painter.setBrush(pg.mkBrush(bottom_right))
        painter.drawEllipse(
            QtCore.QPointF(x + radius / 2, y + radius / 2 + 5), radius / 5, radius / 5
        )
