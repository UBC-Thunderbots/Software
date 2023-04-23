from pyqtgraph.Qt import QtGui


class MotorFaultIconLoader:

    WARNING_ICON = None
    STOPPED_ICON = None
    NO_FAULT_ICON = None


def get_warning_icon():
    if not MotorFaultIconLoader.WARNING_ICON:
        MotorFaultIconLoader.WARNING_ICON = QtGui.QPixmap(
            "software/thunderscope/robot_diagnostics/motor_fault_icons/warning.png"
        )

    return MotorFaultIconLoader.WARNING_ICON


def get_stopped_icon():
    if not MotorFaultIconLoader.STOPPED_ICON:
        MotorFaultIconLoader.STOPPED_ICON = QtGui.QPixmap(
            "software/thunderscope/robot_diagnostics/motor_fault_icons/stopped.png"
        )

    return MotorFaultIconLoader.STOPPED_ICON


def get_no_fault_icon():
    if not MotorFaultIconLoader.NO_FAULT_ICON:
        MotorFaultIconLoader.NO_FAULT_ICON = QtGui.QPixmap(
            "software/thunderscope/robot_diagnostics/motor_fault_icons/no_fault.png"
        )

    return MotorFaultIconLoader.NO_FAULT_ICON
