import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from software.py_constants import *
import software.thunderscope.common.common_widgets as common_widgets
from proto.import_all_protos import *


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

        self.robot_status_buffer = ThreadSafeBuffer(100, RobotStatus)
        self.MIN_VOLTAGE_LEVEL = 20.5  # TODO PULL THIS OUT

        # There is no pattern to this so we just have to create
        # mapping from robot id to the four corners of the vision pattern
        #
        # robot-id: top-right, top-left, bottom-left, bottom-right
        #
        # https://robocup-ssl.github.io/ssl-rules/sslrules.html
        self.vision_pattern_lookup = {
            0: [self.pink, self.pink, self.green, self.pink],
            1: [self.pink, self.green, self.green, self.pink],
            2: [self.green, self.green, self.green, self.pink],
            3: [self.green, self.pink, self.green, self.pink],
            4: [self.pink, self.pink, self.pink, self.green],
            5: [self.pink, self.green, self.pink, self.green],
            6: [self.green, self.green, self.pink, self.green],
            7: [self.green, self.pink, self.pink, self.green],
            8: [self.green, self.green, self.green, self.green],
            9: [self.pink, self.pink, self.pink, self.pink],
            10: [self.pink, self.pink, self.green, self.green],
            11: [self.green, self.green, self.pink, self.pink],
            12: [self.pink, self.green, self.green, self.green],
            13: [self.pink, self.green, self.pink, self.pink],
            14: [self.green, self.pink, self.green, self.green],
            15: [self.green, self.pink, self.pink, self.pink],
        }

        self.layout = QVBoxLayout()

        self.robot_layouts = [QHBoxLayout() for x in range(8)]
        self.robot_status_layouts = [QVBoxLayout() for x in range(8)]
        self.robot_battery_progress_bars = [QProgressBar() for x in range(8)]
        self.breakbeam_labels = [QLabel() for x in range(8)]

        for x in range(8):
            QVBoxLayout()
            self.robot_battery_progress_bars[x].setMaximum(100)
            self.robot_battery_progress_bars[x].setMinimum(0)
            self.robot_battery_progress_bars[x].setValue(10)

            self.breakbeam_labels[x].setText("BREAKBEAM")
            self.breakbeam_labels[x].setStyleSheet("background-color: blue")

            self.robot_status_layouts[x].addWidget(self.robot_battery_progress_bars[x])
            self.robot_status_layouts[x].addWidget(self.breakbeam_labels[x])

            self.robot_layouts[x].addWidget(
                self.create_vision_pattern_label(x, "b", 25)
            )
            self.robot_layouts[x].addLayout(self.robot_status_layouts[x])
            self.layout.addLayout(self.robot_layouts[x])

        self.setLayout(self.layout)

    def create_vision_pattern_label(self, id, team_colour, radius):
        """Given a robot id, team color and radius, draw the vision
        pattern on a label and return it.

        :param id: The robot
        :param team_colour: The team colour
        :param radius: The radius of the robot

        """
        pixmap = QtGui.QPixmap(radius * 2, radius * 2)
        pixmap.fill(QtCore.Qt.GlobalColor.transparent)

        painter = QtGui.QPainter(pixmap)
        painter.setPen(pg.mkPen("black"))
        painter.setBrush(pg.mkBrush("black"))

        convert_degree = -16

        painter.drawChord(
            QtCore.QRectF(0, 0, int(radius * 2), int(radius * 2),),
            -45 * convert_degree,
            270 * convert_degree,
        )

        # Draw the vision pattern
        # Draw the centre team color
        painter.setBrush(pg.mkBrush(team_colour))
        painter.drawEllipse(QtCore.QPointF(radius, radius), radius / 4, radius / 4)

        # Grab the collors for the vision pattern and setup the locations
        # for the four cicles in the four corners
        top_right, top_left, bottom_left, bottom_right = self.vision_pattern_lookup[id]
        top_circle_locations = [
            QtCore.QPointF(radius + radius / 2 + 5, radius - radius / 2),
            QtCore.QPointF(radius - radius / 2 - 5, radius - radius / 2),
            QtCore.QPointF(radius - radius / 2, radius + radius / 2 + 5),
            QtCore.QPointF(radius + radius / 2, radius + radius / 2 + 5),
        ]

        for color, location in zip(
            self.vision_pattern_lookup[id], top_circle_locations
        ):
            painter.setBrush(pg.mkBrush(color))
            painter.drawEllipse(location, radius / 5, radius / 5)

        painter.end()

        label = QLabel()
        label.setPixmap(pixmap)

        return label

    def refresh(self):
        """Refresh the view
        """
        return
        breakbeam_status = self.breakbeam_buffer.get(block=False)
        for i in range(8):
            if breakbeam_status.ball_in_beam:
                self.breakbeam_labels[i].setText("In Beam")
                self.breakbeam_labels[i].setStyleSheet("background-color: red")
            else:
                self.breakbeam_labels[i].setText("Not in Beam")
                self.breakbeam_labels[i].setStyleSheet("background-color: green")

        power_status = self.power_status_buffer.get(block=False)
        for i in range(8):
            self.robot_battery_progress_bars[i].setValue(power_status.battery_voltage)

            if power_status.battery_voltage > self.MIN_VOLTAGE_LEVEL:
                msg = QMessageBox()
                msg.setIcon(QMessageBox.Information)
                msg.setText(f"robot {i} voltage is {power_status.battery_voltage}")
                msg.setWindowTitle("battery voltage alert")
                msg.setDetailedText("not cool man")

                msg.show()
