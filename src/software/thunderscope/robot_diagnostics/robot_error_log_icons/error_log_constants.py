from pyqtgraph.Qt import QtGui


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
    FATAL_LOG_ICON = None


def get_error_code_icon() -> QtGui.QPixmap:
    """
    Loads the Error Code icon pixmap as a ErrorLogIconLoader attribute
    :return: the icon pixmap
    """
    if not ErrorLogIconLoader.ERROR_CODE_ICON:
        ErrorLogIconLoader.ERROR_CODE_ICON = QtGui.QPixmap(
            "software/thunderscope/robot_diagnostics/robot_error_log_icons/error_code.png"
        )

    return ErrorLogIconLoader.ERROR_CODE_ICON


def get_low_battery_icon() -> QtGui.QPixmap:
    """
    Loads the Low Battery icon pixmap as a ErrorLogIconLoader attribute
    :return: the icon pixmap
    """
    if not ErrorLogIconLoader.LOW_BATTERY_ICON:
        ErrorLogIconLoader.LOW_BATTERY_ICON = QtGui.QPixmap(
            "software/thunderscope/robot_diagnostics/robot_error_log_icons/low_battery.png"
        )

    return ErrorLogIconLoader.LOW_BATTERY_ICON


def get_robot_crash_icon() -> QtGui.QPixmap:
    """
    Loads the Robot Crash icon pixmap as a ErrorLogIconLoader attribute
    :return: the icon pixmap
    """
    if not ErrorLogIconLoader.ROBOT_CRASH_ICON:
        ErrorLogIconLoader.ROBOT_CRASH_ICON = QtGui.QPixmap(
            "software/thunderscope/robot_diagnostics/robot_error_log_icons/crash.png"
        )

    return ErrorLogIconLoader.ROBOT_CRASH_ICON


def get_fatal_log_icon() -> QtGui.QPixmap:
    """
    Loads the Fatal Log icon pixmap as a ErrorLogIconLoader attribute
    :return: the icon pixmap
    """
    if not ErrorLogIconLoader.FATAL_LOG_ICON:
        ErrorLogIconLoader.FATAL_LOG_ICON = QtGui.QPixmap(
            "software/thunderscope/robot_diagnostics/robot_error_log_icons/fatal_log.png"
        )

    return ErrorLogIconLoader.FATAL_LOG_ICON
