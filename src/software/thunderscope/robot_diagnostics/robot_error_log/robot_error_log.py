from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.Qt import QtGui
from software.thunderscope.robot_diagnostics.robot_error_log.error_log_constants import (
    RobotErrorLogMessage,
)


class RobotErrorMessage(QWidget):
    """
    A single error message from a robot.

    Displays an icon corresponding to the type of error, the robot id, the message
    and the timestamp of the error
    """

    def __init__(self, error: RobotErrorLogMessage):
        super(RobotErrorMessage, self).__init__()

        self.layout = QHBoxLayout()
        self.icon = QLabel()

        # setting up the icon
        self.icon_size = self.width() / 7
        self.icon.setPixmap(
            error.icon.scaled(
                self.icon_size, self.icon_size, Qt.AspectRatioMode.KeepAspectRatio
            )
        )

        # info layout contains the robot id and message
        self.info_layout = QVBoxLayout()
        self.info_layout.setContentsMargins(
            0, self.height() / 16, 0, self.height() / 12
        )

        # robot id displayed more prominently
        self.robot_id = QLabel(f"Robot {error.robot_id}")
        self.robot_id.setStyleSheet(
            "font-weight: bold;" f"font-size: {int(self.height() / 20)}px;"
        )

        # message of the error
        self.message = QLabel(error.message)
        self.message.setStyleSheet(f"font-size: {int(self.height() / 30)}px;")

        self.info_layout.addWidget(self.robot_id)
        self.info_layout.addWidget(self.message)

        # the timestamp the error took place
        self.timestamp = QLabel(error.timestamp)

        self.layout.addWidget(self.icon)
        self.layout.addStretch(1)
        self.layout.addLayout(self.info_layout)
        self.layout.addStretch(16)
        self.layout.addWidget(self.timestamp)
        self.layout.setContentsMargins(0, 0, 0, 0)

        self.setLayout(self.layout)


class RobotErrorLog(QScrollArea):
    """
    A log of all the errors from all robots during gameplay

    Allows for dynamically adding new entries
    """

    def __init__(self):
        super(RobotErrorLog, self).__init__()

        self.layout = QVBoxLayout()
        self.layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        # for a QScrollArea, widgets cannot be added to it directly
        # doing so causes no scrolling to happen, and all the components get smaller
        # instead, widgets are added to the layout which is set for a container
        # the container is set as the current QScrollArea's widget
        self.container = QFrame(self)
        self.container.setLayout(self.layout)
        self.setWidget(self.container)
        self.setWidgetResizable(True)

    def refresh(self):
        """
        Refreshes the widget's graphics
        """

    def add_error_log_message(self, error: RobotErrorLogMessage):
        """
        Adds the given error message to the log by making a new RobotErrorMessage
        :param error: the error to add to the log
        """
        error_widget = RobotErrorMessage(error)
        self.layout.addWidget(error_widget)
