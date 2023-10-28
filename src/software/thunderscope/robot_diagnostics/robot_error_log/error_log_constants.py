from proto.import_all_protos import *
from pyqtgraph.Qt import QtGui
from datetime import datetime


class RobotErrorLogMessage:
    """
    Represents a robot error log message with a type, the message to display,
    the id of the robot, the icon to display, and the last robot status if needed
    """

    def __init__(self, message: str, robot_id: int, icon: QtGui.QPixmap):
        self.message = message
        self.robot_id = robot_id
        self.icon = icon
        self.timestamp = datetime.now()


class LowBatteryErrorLogMessage(RobotErrorLogMessage):
    """
    Represents a robot error log message when a robot has low battery
    """

    def __init__(self, robot_id: int):
        super(LowBatteryErrorLogMessage, self).__init__(
            "Battery voltage is low", robot_id, get_low_battery_icon()
        )


class RobotCrashErrorLogMessage(RobotErrorLogMessage):
    """
    Represents a robot error log message when a robot has crashed
    """

    def __init__(self, crash_message: RobotCrash):
        super(RobotCrashErrorLogMessage, self).__init__(
            "Robot has Crashed", crash_message.robot_id, get_robot_crash_icon()
        )
        self.crash_message = crash_message


class ErrorCodeLogMessage(RobotErrorLogMessage):
    """
   Represents a robot error log message when has an error code
   """

    def __init__(self, robot_id: int, error_code: str):
        super(ErrorCodeLogMessage, self).__init__(
            f"Robot encountered Error Code: {error_code}",
            robot_id,
            get_error_code_icon(),
        )


class ErrorLogIconLoader:
    """
    Stores icons for the robot error log widget

    Since they are class level variables, they are initialized only once
    when they are first accessed
    Which saves the cost of loading them every time
    """

    ERROR_CODE_ICON = None
    LOW_BATTERY_ICON = None
    ROBOT_CRASH_ICON = None


def get_error_code_icon():
    """
    Loads the Error Code icon pixmap as a ErrorLogIconLoader attribute
    :return: the icon pixmap
    """
    if not ErrorLogIconLoader.ERROR_CODE_ICON:
        ErrorLogIconLoader.ERROR_CODE_ICON = QtGui.QPixmap(
            "software/thunderscope/robot_diagnostics/robot_error_log/error_code.png"
        )

    return ErrorLogIconLoader.ERROR_CODE_ICON


def get_low_battery_icon():
    """
    Loads the Low Battery icon pixmap as a ErrorLogIconLoader attribute
    :return: the icon pixmap
    """
    if not ErrorLogIconLoader.LOW_BATTERY_ICON:
        ErrorLogIconLoader.LOW_BATTERY_ICON = QtGui.QPixmap(
            "software/thunderscope/robot_diagnostics/robot_error_log/low_battery.png"
        )

    return ErrorLogIconLoader.LOW_BATTERY_ICON


def get_robot_crash_icon():
    """
    Loads the Robot Crash icon pixmap as a ErrorLogIconLoader attribute
    :return: the icon pixmap
    """
    if not ErrorLogIconLoader.ROBOT_CRASH_ICON:
        ErrorLogIconLoader.ROBOT_CRASH_ICON = QtGui.QPixmap(
            "software/thunderscope/robot_diagnostics/robot_error_log/crash.png"
        )

    return ErrorLogIconLoader.ROBOT_CRASH_ICON
