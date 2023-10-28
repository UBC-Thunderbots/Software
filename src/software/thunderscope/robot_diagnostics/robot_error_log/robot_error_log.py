from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.Qt import QtGui
from software.thunderscope.robot_diagnostics.robot_error_log.error_log_constants import (
    RobotErrorLogMessage,
)


class RobotErrorMessage(QWidget):
    def __init__(self, error: RobotErrorLogMessage):
        super(RobotErrorMessage, self).__init__()

        self.layout = QHBoxLayout()
        self.icon = QLabel()
        self.icon_size = self.width() / 7
        self.icon.setPixmap(
            error.icon.scaled(
                self.icon_size, self.icon_size, Qt.AspectRatioMode.KeepAspectRatio
            )
        )

        self.info_layout = QVBoxLayout()
        self.info_layout.setContentsMargins(
            0, self.height() / 16, 0, self.height() / 12
        )

        self.robot_id = QLabel(f"Robot {error.robot_id}")
        self.robot_id.setStyleSheet(
            "font-weight: bold;" f"font-size: {int(self.height() / 20)}px;"
        )

        self.message = QLabel(error.message)
        self.message.setStyleSheet(f"font-size: {int(self.height() / 30)}px;")

        self.info_layout.addWidget(self.robot_id)
        self.info_layout.addWidget(self.message)

        self.timestamp = QLabel(error.timestamp)

        self.layout.addWidget(self.icon)
        self.layout.addStretch(1)
        self.layout.addLayout(self.info_layout)
        self.layout.addStretch(16)
        self.layout.addWidget(self.timestamp)
        self.layout.setContentsMargins(0, 0, 0, 0)

        self.setLayout(self.layout)


class RobotErrorLog(QScrollArea):
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
        pass

    def add_error_log_message(self, error: RobotErrorLogMessage):
        print(error)
        error_widget = RobotErrorMessage(error)
        self.layout.addWidget(error_widget)
