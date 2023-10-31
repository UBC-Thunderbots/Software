from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.Qt import QtGui
from proto.import_all_protos import *
from datetime import datetime
import textwrap
import software.thunderscope.robot_diagnostics.robot_error_log_icons.error_log_constants as error_constants
from proto.robot_log_msg_pb2 import RobotLog, LogLevel
from software.thunderscope.robot_diagnostics.robot_status import RobotStatusView
from software.thunderscope.constants import LOG_LEVEL_STR_MAP


class RobotLogMessageWidget(QFrame):
    """
    A single error message from a robot.

    Displays an icon corresponding to the type of error, the robot id, the message
    and the timestamp of the error
    """

    def __init__(
        self, robot_id: int, message: str, icon: QtGui.QPixmap, timestamp=None
    ):
        super(RobotLogMessageWidget, self).__init__()

        self.layout = QHBoxLayout()
        self.icon = QLabel()

        # setting up the icon
        self.icon_size = self.width() / 10
        self.icon.setPixmap(
            icon.scaled(
                self.icon_size, self.icon_size, Qt.AspectRatioMode.KeepAspectRatio
            )
        )

        # info layout contains the robot id and message
        self.info_layout = QVBoxLayout()
        self.info_layout.setContentsMargins(
            0, self.icon_size / 4, 0, self.icon_size / 3
        )

        # robot id displayed more prominently
        self.robot_id = QLabel(f"Robot {robot_id}")
        self.robot_id.setStyleSheet(
            "font-weight: bold;" f"font-size: {int(self.height() / 25)}px;"
        )

        # message of the error
        self.message = QLabel(message)
        self.message.setStyleSheet(f"font-size: {int(self.height() / 35)}px;")

        self.info_layout.addWidget(self.robot_id)
        self.info_layout.addWidget(self.message)

        # layout for timestamp info
        self.time_layout = QVBoxLayout()
        self.time_layout.setContentsMargins(
            0, self.icon_size / 4, 0, self.icon_size / 3
        )

        # the timestamp the error took place
        self.timestamp = timestamp if timestamp else datetime.now()
        self.timestamp_label = QLabel(self.timestamp.strftime("%H:%M:%S %f"))
        # time since the error was logged (easier to understand than timestamp)
        self.time_since_label = QLabel(str(0))

        self.time_layout.addWidget(self.timestamp_label)
        self.time_layout.addWidget(self.time_since_label)

        # adding spacing to look nice
        self.layout.addWidget(self.icon)
        self.layout.addStretch(1)
        self.layout.addLayout(self.info_layout)
        self.layout.addStretch(16)
        self.layout.addLayout(self.time_layout)
        self.layout.setContentsMargins(0, 0, 0, 0)

        self.setLayout(self.layout)
        self.base_stylesheet = textwrap.dedent(
            f"""
            RobotLogMessageWidget {{
                border: 0;
                padding: {self.icon_size / 22};
            }}
            """
        )
        self.setStyleSheet(self.base_stylesheet)

    def update_time(self):
        """
        Updates the time since label for the error log message
        """
        # get the time difference from the timestamp to now
        time_diff = datetime.now() - self.timestamp
        self.time_since_label.setText(f"{int(time_diff.total_seconds())} seconds ago")

    def mouseReleaseEvent(self, event):
        """
        Mouse Release Event handler for child classes to override
        :param event: the mouse release event
        """
        super().mouseReleaseEvent(event)


class LowBatteryLogMessageWidget(RobotLogMessageWidget):
    """
    A robot error log message when a robot has low battery
    """

    def __init__(self, robot_id):
        super(LowBatteryLogMessageWidget, self).__init__(
            robot_id, "Battery voltage is low", error_constants.get_low_battery_icon()
        )


class ErrorCodeLogMessageWidget(RobotLogMessageWidget):
    """
    A robot error log message when it has an error code
    """

    def __init__(self, robot_id: int, error_code: str):
        super(ErrorCodeLogMessageWidget, self).__init__(
            robot_id,
            f"Robot encountered Error Code: {error_code}",
            error_constants.get_error_code_icon(),
        )


class RobotLogMessageWithDialogWidget(RobotLogMessageWidget):
    """
    A clickable error message from the robot
    Displays the error info, and opens up a dialog with more info when clicked
    """

    def __init__(
        self, robot_id: int, message: str, icon: QtGui.QPixmap, timestamp=None
    ):
        super(RobotLogMessageWithDialogWidget, self).__init__(
            robot_id, message, icon, timestamp
        )
        self.setStyleSheet(
            self.base_stylesheet
            + textwrap.dedent(
                f"""
                RobotLogMessageWidget:hover {{
                    background-color: rgba(0, 0, 0, 0.15);
                }}
                """
            )
        )
        self.dialog = None

    def enterEvent(self, event):
        """
        Sets the cursor to indicate that this widget is clickable upon mouse enter
        :param event: the mouse enter event
        """
        self.setCursor(Qt.CursorShape.PointingHandCursor)

    def mouseReleaseEvent(self, event):
        """
        If the error has a robot status to display, creates and opens a dialog to
        display it on mouse click
        :param event: the mouse release event
        """
        super().mouseReleaseEvent(event)
        if self.dialog:
            self.dialog.exec()


class RobotCrashLogMessageWidget(RobotLogMessageWithDialogWidget):
    """
    A robot error log message when a robot has crashed. On click, opens a dialog with last robot status
    """

    def __init__(self, crash_message: RobotCrash):
        super(RobotCrashLogMessageWidget, self).__init__(
            crash_message.robot_id,
            "Robot has Crashed",
            error_constants.get_robot_crash_icon(),
        )
        self.crash_message = crash_message
        robot_crash_text = (
            f"robot_id: {self.crash_message.robot_id}\n"
            + f"exit_signal: {self.crash_message.exit_signal}\n"
            + f"stack_dump: {self.crash_message.stack_dump}"
        )
        self.dialog = RobotCrashDialog(robot_crash_text, self.crash_message)


class FatalLogMessageWidget(RobotLogMessageWithDialogWidget):
    """
    A robot error log message when it has a fatal log message. On click,
    opens a dialog with the stack dump + extra info
    """

    def __init__(self, robot_log: RobotLog):
        super(FatalLogMessageWidget, self).__init__(
            robot_log.robot_id,
            "Fatal Log Message",
            error_constants.get_fatal_log_icon(),
            datetime.fromtimestamp(robot_log.created_timestamp.epoch_timestamp_seconds),
        )
        self.log = robot_log
        self.dialog = RobotFatalLogDialog(self.log)


class RobotFatalLogDialog(QDialog):
    """
    Dialog to show information about a fatal robot log message

    Displays the message info along with the robot ID and the file / line number that caused it
    """

    def __init__(self, fatal_log: RobotLog, parent=None):
        super().__init__(parent)

        self.setWindowTitle("Fatal Log Alert")

        self.buttonBox = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

        self.layout = QVBoxLayout()

        self.robot_id = QLabel(f"Robot ID: {fatal_log.robot_id}")
        formatted_date = datetime.fromtimestamp(
            fatal_log.created_timestamp.epoch_timestamp_seconds
        ).strftime("%H:%M:%S %f")
        self.timestamp = QLabel(f"Timestamp: {formatted_date}")
        self.log_level = QLabel(f"Log Level: {LOG_LEVEL_STR_MAP[fatal_log.log_level]}")
        self.file_info = QLabel(
            f"File Info: {fatal_log.file_name} at Line {fatal_log.line_number}"
        )
        self.message = QLabel(f"Message: {fatal_log.log_msg}")

        self.layout.addWidget(self.robot_id)
        self.layout.addWidget(self.timestamp)
        self.layout.addWidget(self.log_level)
        self.layout.addWidget(self.file_info)
        self.layout.addWidget(self.message)

        self.layout.addWidget(self.buttonBox)
        self.setLayout(self.layout)


class RobotCrashDialog(QDialog):
    """Dialog to show information about a robot before it crashed,

    Displays the message and RobotStatus.

    """

    def __init__(self, message, robot_status, parent=None):
        super().__init__(parent)

        self.setWindowTitle("Robot Crash")

        self.buttonBox = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

        self.layout = QVBoxLayout()
        self.message = QLabel(message)
        self.layout.addWidget(self.message)
        self.robot_status = RobotStatusView()
        if robot_status is not None:
            self.robot_status.update(robot_status)
            self.robot_status.toggle_visibility()
        self.layout.addWidget(self.robot_status)
        self.layout.addWidget(self.buttonBox)
        self.setLayout(self.layout)
