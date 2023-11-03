from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt.QtCore import Qt, QTimer
from proto.import_all_protos import *
from software.py_constants import *
from software.thunderscope.robot_diagnostics.error_log_widgets import (
    RobotLogMessageWidget,
    RobotCrashLogMessageWidget,
    ErrorCodeLogMessageWidget,
    LowBatteryLogMessageWidget,
    FatalLogMessageWidget,
)
from proto.robot_log_msg_pb2 import RobotLog, LogLevel
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.constants import (
    ERROR_CODE_MESSAGES,
    ROBOT_CRASH_TIMEOUT_S,
    ROBOT_FATAL_TIMEOUT_S,
)

import time


class RobotErrorLog(QScrollArea):
    """
    A log of all the errors from all robots during gameplay

    Allows for dynamically adding new entries
    """

    def __init__(self):
        super(RobotErrorLog, self).__init__()

        self.robot_status_buffer = ThreadSafeBuffer(10, RobotStatus)
        self.robot_crash_buffer = ThreadSafeBuffer(10, RobotCrash)
        self.robot_log_buffer = ThreadSafeBuffer(10, RobotLog)

        self.robot_last_crash_time_s = {}
        self.robot_last_fatal_time_s = {}

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

        # TODO : REMOVE BEFORE MERGE
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

        self.robot_log_buffer.put(
            RobotLog(
                robot_id=2,
                created_timestamp=Timestamp(epoch_timestamp_seconds=20),
                log_level=LogLevel.FATAL,
                line_number=20,
                file_name="file.txt",
                log_msg="test",
            )
        )

        self.robot_log_buffer.put(
            RobotLog(
                robot_id=2,
                created_timestamp=Timestamp(epoch_timestamp_seconds=20),
                log_level=LogLevel.DEBUG,
                line_number=20,
                file_name="file.txt",
                log_msg="test",
            )
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

        # get the robot log message
        robot_log = self.robot_log_buffer.get(block=False, return_cached=False)

        if robot_log is not None:
            if (
                robot_log.log_level == LogLevel.FATAL
                or robot_log.log_level == LogLevel.CONTRACT
            ):
                if (
                    robot_log.robot_id not in self.robot_last_fatal_time_s
                    or time.time() - self.robot_last_fatal_time_s[robot_log.robot_id]
                    > ROBOT_FATAL_TIMEOUT_S
                ):
                    self.add_error_log_message(FatalLogMessageWidget(robot_log))
                self.robot_last_fatal_time_s[robot_log.robot_id] = time.time()

        # get the robot crash message
        robot_crash = self.robot_crash_buffer.get(block=False, return_cached=False)

        # if there's a crash, log it
        if robot_crash is not None:
            if (
                robot_crash.robot_id not in self.robot_last_crash_time_s
                or time.time() - self.robot_last_crash_time_s[robot_crash.robot_id]
                > ROBOT_CRASH_TIMEOUT_S
            ):
                self.add_error_log_message(RobotCrashLogMessageWidget(robot_crash))
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
                        ErrorCodeLogMessageWidget(
                            robot_status.robot_id, ERROR_CODE_MESSAGES[code]
                        )
                    )

            # if the battery voltage is too low, log it
            if (
                robot_status.power_status.battery_voltage <= BATTERY_WARNING_VOLTAGE
                and not self.low_battery_log_disabled
            ):
                self.add_error_log_message(
                    LowBatteryLogMessageWidget(robot_status.robot_id)
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

    def add_error_log_message(self, error_widget: RobotLogMessageWidget):
        """
        Adds the given error message to the log by making a new RobotErrorMessage
        :param error_widget: the error widget to add to the log
        """
        self.error_log_messages.append(error_widget)
        self.layout.addWidget(error_widget)
