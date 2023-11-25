from pyqtgraph.Qt import QtGui


class MotorFaultIconLoader:
    """
    Stores icons for Motor Fault Visulization

    Since they are class level variables, they are initialized only once
    when they are first accessed
    Which saves the cost of loading them every time
    """

    WARNING_ICON = None
    STOPPED_ICON = None
    NO_FAULT_ICON = None


def get_warning_icon() -> QtGui.QPixmap:
    """
    Loads the Motor Warning icon pixmap as a MotorFaultIconLoader attribute
    :return: the icon pixmap
    """
    if not MotorFaultIconLoader.WARNING_ICON:
        MotorFaultIconLoader.WARNING_ICON = QtGui.QPixmap(
            "software/thunderscope/robot_diagnostics/motor_fault_icons/warning.png"
        )

    return MotorFaultIconLoader.WARNING_ICON


def get_stopped_icon() -> QtGui.QPixmap:
    """
    Loads the Motor Stopped icon pixmap as a MotorFaultIconLoader attribute
    :return: the icon pixmap
    """
    if not MotorFaultIconLoader.STOPPED_ICON:
        MotorFaultIconLoader.STOPPED_ICON = QtGui.QPixmap(
            "software/thunderscope/robot_diagnostics/motor_fault_icons/stopped.png"
        )

    return MotorFaultIconLoader.STOPPED_ICON


def get_no_fault_icon() -> QtGui.QPixmap:
    """
    Loads the Motor No Fault icon pixmap as a MotorFaultIconLoader attribute
    :return: the icon pixmap
    """
    if not MotorFaultIconLoader.NO_FAULT_ICON:
        MotorFaultIconLoader.NO_FAULT_ICON = QtGui.QPixmap(
            "software/thunderscope/robot_diagnostics/motor_fault_icons/no_fault.png"
        )

    return MotorFaultIconLoader.NO_FAULT_ICON
