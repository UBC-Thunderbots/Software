from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt.QtCore import Qt, QTimer
from pyqtgraph.Qt import QtGui
from proto.import_all_protos import *
from software.py_constants import *
from software.thunderscope.robot_diagnostics.robot_error_log.error_log_constants import (
    RobotErrorLogMessage,
    RobotCrashErrorLogMessage,
    ErrorCodeLogMessage,
    LowBatteryErrorLogMessage,
)
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.robot_diagnostics.robot_status import RobotStatusView
from software.thunderscope.constants import ERROR_CODE_MESSAGES
from datetime import datetime
import time


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


class RobotErrorMessageWidget(QWidget):
    """
    A single error message from a robot.

    Displays an icon corresponding to the type of error, the robot id, the message
    and the timestamp of the error
    """

    def __init__(self, error: RobotErrorLogMessage):
        super(RobotErrorMessageWidget, self).__init__()

        self.error = error

        self.layout = QHBoxLayout()
        self.icon = QLabel()

        # setting up the icon
        self.icon_size = self.width() / 8
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
            "font-weight: bold;" f"font-size: {int(self.height() / 25)}px;"
        )

        # message of the error
        self.message = QLabel(error.message)
        self.message.setStyleSheet(f"font-size: {int(self.height() / 35)}px;")

        self.info_layout.addWidget(self.robot_id)
        self.info_layout.addWidget(self.message)

        # layout for timestamp info
        self.time_layout = QVBoxLayout()
        self.time_layout.setContentsMargins(
            0, self.height() / 16, 0, self.height() / 12
        )

        # the timestamp the error took place
        self.timestamp = error.timestamp
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

    def update_time(self):
        """
        Updates the time since label for the error log message
        """
        # get the time difference from the timestamp to now
        time_diff = datetime.now() - self.timestamp
        self.time_since_label.setText(f"{int(time_diff.total_seconds())} seconds ago")

    def mouseReleaseEvent(self, event):
        """
        If the error has a robot status to display, creates and opens a dialog to
        display it on mouse click
        :param event: the mouse release event
        """
        if hasattr(self.error, "crash_message"):
            crash_message = self.error.crash_message
            robot_crash_text = (
                f"robot_id: {crash_message.robot_id}\n"
                + f"exit_signal: {crash_message.exit_signal}\n"
                + f"stack_dump: {crash_message.stack_dump}"
            )
            dialog = RobotCrashDialog(robot_crash_text, crash_message)
            dialog.exec()


class RobotErrorLog(QScrollArea):
    """
    A log of all the errors from all robots during gameplay

    Allows for dynamically adding new entries
    """

    def __init__(self):
        super(RobotErrorLog, self).__init__()

        self.robot_status_buffer = ThreadSafeBuffer(10, RobotStatus)
        self.robot_crash_buffer = ThreadSafeBuffer(10, RobotCrash)

        self.robot_last_crash_time_s = {}

        # when this robot has a battery warning, this is set to True
        # which prevents spamming the same battery warning
        # set back to False if battery is back above warning level
        self.low_battery_log_disabled = False

        self.layout = QVBoxLayout()
        self.layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        self.error_log_messages = []

        # for a QScrollArea, widgets cannot be added to it directly
        # doing so causes no scrolling to happen, and all the components get smaller
        # instead, widgets are added to the layout which is set for a container
        # the container is set as the current QScrollArea's widget
        self.container = QFrame(self)
        self.container.setLayout(self.layout)
        self.setWidget(self.container)
        self.setWidgetResizable(True)

        QTimer.singleShot(3000, self.test)
        QTimer.singleShot(3100, self.test)
        QTimer.singleShot(3200, self.test)

    def test(self):
        self.robot_crash_buffer.put(RobotCrash(robot_id=5, status=RobotStatus()))
        self.robot_status_buffer.put(
            RobotStatus(
                robot_id=3, error_code=[ErrorCode.LOW_CAP, ErrorCode.LOW_BATTERY,]
            )
        )
        self.robot_status_buffer.put(
            RobotStatus(robot_id=2, power_status=PowerStatus(battery_voltage=0))
        )

    def refresh(self):
        """
        Refreshes the widget's graphics

        Gets the RobotCrash and RobotStatus messages from the buffers and adds
        the corresponding log messages if needed

        Updates the time since of the existing messages
        """
        # update time since for all existing logs
        for error_message in self.error_log_messages:
            error_message.update_time()

        # get the robot crash message
        robot_crash = self.robot_crash_buffer.get(block=False, return_cached=False)

        # if there's a crash, log it
        if robot_crash is not None:
            if (
                robot_crash.robot_id not in self.robot_last_crash_time_s
                or time.time() - self.robot_last_crash_time_s[robot_crash.robot_id] > 0
            ):
                self.add_error_log_message(RobotCrashErrorLogMessage(robot_crash))
            # prevent repeated crash logs by having a buffer time
            self.robot_last_crash_time_s[robot_crash.robot_id] = time.time()

        # get a RobotStatus message
        robot_status = self.robot_status_buffer.get(block=False, return_cached=False)

        # empty out the buffer to get statuses for all current robots at once
        while robot_status is not None:
            # if there's an error code, add to log
            for code in robot_status.error_code:
                if (
                    code != ErrorCode.NO_ERROR
                    and code != ErrorCode.LOW_BATTERY
                    and code in ERROR_CODE_MESSAGES.keys()
                ):
                    self.add_error_log_message(
                        ErrorCodeLogMessage(
                            robot_status.robot_id, ERROR_CODE_MESSAGES[code]
                        )
                    )

            # if the battery voltage is too low, log it
            if (
                robot_status.power_status.battery_voltage <= BATTERY_WARNING_VOLTAGE
                and not self.low_battery_log_disabled
            ):
                self.add_error_log_message(
                    LowBatteryErrorLogMessage(robot_status.robot_id)
                )
                # prevent spamming logs by disabling logs
                self.low_battery_log_disabled = True
            elif robot_status.power_status.battery_voltage > BATTERY_WARNING_VOLTAGE:
                # battery voltage is now above threshold again, so start logging low battery
                self.low_battery_log_disabled = False

            # get the next robot status and loop again
            robot_status = self.robot_status_buffer.get(
                block=False, return_cached=False
            )

    def add_error_log_message(self, error: RobotErrorLogMessage):
        """
        Adds the given error message to the log by making a new RobotErrorMessage
        :param error: the error to add to the log
        """
        error_widget = RobotErrorMessageWidget(error)
        self.error_log_messages.append(error_widget)
        self.layout.addWidget(error_widget)
