from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt.QtCore import Qt, QTimer
from proto.import_all_protos import *
from typing import Dict, List
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
    ROBOT_LOG_WIDGET_TIMEOUT,
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
        self.robot_crash_buffer = ThreadSafeBuffer(1000, RobotCrash)
        self.robot_log_buffer = ThreadSafeBuffer(10, RobotLog)

        self.robot_last_crash_time_s = {}
        self.robot_last_fatal_time_s = {}

        # when a robot has a battery warning, its map entry is set to True
        # which prevents spamming the same battery warning
        # set back to False if battery is back above warning level
        self.low_battery_log_disabled: Dict[int, bool] = {}

        # when the robot has an error code, its added to tne list keyed to the robot id
        # which prevents spamming the same error code log
        # list set back to empty if no error code
        self.error_code_log_disabled: Dict[int, List[ErrorCode]] = {}

        self.layout = QVBoxLayout()
        self.layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        self.error_log_messages: Dict[RobotLogMessageWidget, float] = {}

        # for a QScrollArea, widgets cannot be added to it directly
        # doing so causes no scrolling to happen, and all the components get smaller
        # instead, widgets are added to the layout which is set for a container
        # the container is set as the current QScrollArea's widget
        self.container = QFrame(self)
        self.container.setLayout(self.layout)
        self.setWidget(self.container)
        self.setWidgetResizable(True)

    def refresh(self) -> None:
        """
        Refreshes the widget's graphics

        Gets the RobotCrash and RobotStatus messages from the buffers and adds
        the corresponding log messages if needed

        Updates the time since of the existing messages
        """
        # remove all closed widgets first
        self.__remove_closed_or_old_widgets()

        # update time since for all existing logs
        for error_message in self.error_log_messages.keys():
            error_message.update_time()

        # add log widgets for fatal log messages
        self.__refresh_fatal_logs()

        # add log widgets for robot crash messages
        self.__refresh_robot_crash()

        # get a RobotStatus message
        robot_status = self.robot_status_buffer.get(block=False, return_cached=False)

        # empty out the buffer to get statuses for all current robots at once
        while robot_status is not None:

            # add log widgets for error code and low battery warnings
            self.__refresh_robot_status_error_code(robot_status)
            self.__refresh_robot_status_battery_voltage(robot_status)

            # get the next robot status and loop again
            robot_status = self.robot_status_buffer.get(
                block=False, return_cached=False
            )

    def __remove_closed_or_old_widgets(self):
        """
        Removes all log widgets which have been closed or have timed out from the log
        """
        widgets_to_delete = []
        # get all log widgets which have timed out or are old
        for error_message, start_time in self.error_log_messages.items():
            if not error_message.log_open or (
                not error_message.pinned
                and time.time() - start_time > ROBOT_LOG_WIDGET_TIMEOUT
            ):
                widgets_to_delete.append(error_message)

        # delete them from dict and remove them from layout
        for widget in widgets_to_delete:
            self.layout.removeWidget(widget)
            del self.error_log_messages[widget]

    def __refresh_fatal_logs(self) -> None:
        """
        Gets a robot log from the buffer
        If fatal or contract and last log is older than timeout, adds a log to the widget and
        Records new last log time to prevent spamming
        """
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

    def __refresh_robot_crash(self) -> None:
        """
        Gets a robot crash from the buffer
        If last crash is older than timeout, adds a log to the widget and
        Records new last crash time to prevent spamming
        """
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

    def __refresh_robot_status_error_code(self, robot_status: RobotStatus) -> None:
        """
        Adds a log to the widget for each of the error codes in the RobotStatus message
        Doesn't log if this robot has already logged a specific error code type (to prevent spam)
        Clears the types of error codes already added for a robot if no error
        :param robot_status: the RobotStatus message
        """
        # initialize the list if not in dict yet
        if robot_status.robot_id not in self.error_code_log_disabled:
            self.error_code_log_disabled[robot_status.robot_id] = []

        # if there's an error code, add to log
        for code in robot_status.error_code:
            if (
                code != ErrorCode.NO_ERROR
                and code != ErrorCode.LOW_BATTERY
                and code in ERROR_CODE_MESSAGES.keys()
                and code not in self.error_code_log_disabled[robot_status.robot_id]
            ):
                self.add_error_log_message(
                    ErrorCodeLogMessageWidget(
                        robot_status.robot_id, ERROR_CODE_MESSAGES[code]
                    )
                )
                # add to list for this robot id to prevent spamming
                self.error_code_log_disabled[robot_status.robot_id].append(code)
            # if there is no more errors, clear the list so far
            elif code == ErrorCode.NO_ERROR:
                self.error_code_log_disabled[robot_status.robot_id] = []

    def __refresh_robot_status_battery_voltage(self, robot_status: RobotStatus) -> None:
        """
        If the current robot's battery is lower than the threshold, adds a log
        Records that this robot has already had a log added to prevent spamming
        Resets the spamming flag once voltage for this robot goes back above threshold
        :param robot_status: the RobotStatus message
        """
        # if the battery voltage is too low and a voltage log hasn't been sent for this robot yet, log it
        if robot_status.power_status.battery_voltage <= BATTERY_WARNING_VOLTAGE and (
            robot_status.robot_id not in self.low_battery_log_disabled
            or not self.low_battery_log_disabled[robot_status.robot_id]
        ):
            self.add_error_log_message(
                LowBatteryLogMessageWidget(robot_status.robot_id)
            )
            # prevent spamming logs by disabling logs
            self.low_battery_log_disabled[robot_status.robot_id] = True
        elif robot_status.power_status.battery_voltage > BATTERY_WARNING_VOLTAGE:
            # battery voltage is now above threshold again, so start logging low battery
            self.low_battery_log_disabled[robot_status.robot_id] = False

    def add_error_log_message(self, error_widget: RobotLogMessageWidget) -> None:
        """
        Adds the given error message to the log by making a new RobotErrorMessage
        :param error_widget: the error widget to add to the log
        """
        self.error_log_messages[error_widget] = time.time()
        self.layout.addWidget(error_widget)
