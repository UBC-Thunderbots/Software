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
        """
        Creates an error message with the given info

        :param robot_id: the id of the robot that threw the error
        :param message: the message of the error
        :param icon: the icon to display for the error
        :param timestamp: the timestamp the error was thrown
        """
        super(RobotLogMessageWidget, self).__init__()

        # if true, the log is in the log widget and hasn't been cleared
        self.log_open = True
        self.pinned = False

        self.layout = QHBoxLayout()
        self.icon = QLabel()

        # setting up the icon
        self.icon_size = self.width() / 15
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
        self.timestamp_label = QLabel(self.timestamp.strftime("%H:%M:%S.%f"))
        # time since the error was logged (easier to understand than timestamp)
        self.time_since_label = QLabel(str(0))

        self.time_layout.addWidget(self.timestamp_label)
        self.time_layout.addWidget(self.time_since_label)

        # close button to clear log
        self.close_button = QPushButton("X")
        self.close_button.setStyleSheet("padding: 0")
        self.close_button.setFixedHeight(self.icon_size / 2)
        self.close_button.setFixedWidth(self.icon_size / 2)
        self.close_button.clicked.connect(self.close)

        # adding spacing to look nice
        self.layout.addWidget(self.icon)
        self.layout.addStretch(1)
        self.layout.addLayout(self.info_layout)
        self.layout.addStretch(16)
        self.layout.addLayout(self.time_layout)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.addStretch(1)
        self.layout.addWidget(self.close_button)

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

    def close(self) -> None:
        """
        Sets the widget to closed
        """
        self.log_open = not self.log_open

    def update_time(self) -> None:
        """
        Updates the time since label for the error log message
        """
        # get the time difference from the timestamp to now
        time_diff = datetime.now() - self.timestamp
        self.time_since_label.setText(f"{int(time_diff.total_seconds())} seconds ago")

    def mouseReleaseEvent(self, event) -> None:
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
        """
        Creates a low battery error message for the given robot
        :param robot_id: the id of the robot whose battery is low
        """
        super(LowBatteryLogMessageWidget, self).__init__(
            robot_id, "Battery voltage is low", error_constants.get_low_battery_icon()
        )


class ErrorCodeLogMessageWidget(RobotLogMessageWidget):
    """
    A robot error log message when it has an error code
    """

    def __init__(self, robot_id: int, error_code: str):
        """
        Creates an error message that the given robot has the given error code

        :param robot_id: the id of the robot with the error code
        :param error_code: the specific error code
        """
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
        """
        Creates a error message with hover indication. Dialog is set by child classes
        :param robot_id: the id of the robot that threw the error
        :param message: the message of the error
        :param icon: the icon to display for the error
        :param timestamp: the timestamp the error was thrown
        """
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

    def enterEvent(self, event) -> None:
        """
        Sets the cursor to indicate that this widget is clickable upon mouse enter
        :param event: the mouse enter event
        """
        self.setCursor(Qt.CursorShape.PointingHandCursor)

    def mouseReleaseEvent(self, event) -> None:
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
        """
        Creates an error message for a crashed robot. Creates a dialog to display the whole crash message
        :param crash_message: the crash message containing robot id and other info
        """
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
        """
        Creates an error message for a robot with a fatal log message. Creates a dialog to display the whole log
        :param robot_log: the fatal log message
        """
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
        """
        Creates a dialog box to display a robot fatal log clearly
        :param fatal_log: the fatal log to display
        :param parent: the parent widget of the dialog
        """
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
        """
        Creates a dialog box detailing info about a robot crash + the last status of the robot before crashing
        :param message: the crash message of the robot
        :param robot_status: the last robot status of the crashed robot
        :param parent: the parent widget of the dialog
        """
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
