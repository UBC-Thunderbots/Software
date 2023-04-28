from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt.QtCore import Qt, QPoint, QByteArray, QBuffer, QIODeviceBase
from pyqtgraph.Qt import QtGui
from software.thunderscope.robot_diagnostics.motor_fault_icons.motor_fault_icon_loader import (
    get_no_fault_icon,
    get_stopped_icon,
    get_warning_icon,
)
from software.thunderscope.common.common_widgets import get_string_val


class MotorFaultView(QWidget):
    """
    Class to visualise information about motor faults from robot status

    Displays if any of the motors have a fault and / or are disabled
    Displays specific faults for each motor in a tooltip
    """

    WIDGET_SIZE_SCALE = 2.2

    def __init__(self):
        """
        Initializes the motor fault view widget

        Sets all motor fault info to default and sets the tooltip to empty
        Initialises the main label and the fault count notification label
        """
        super().__init__()
        self.layout = QHBoxLayout()

        self.drive_enabled = True
        self.fault_count = 0
        self.motor_faults = {
            "Front Left": None,
            "Front Right": None,
            "Back Left": None,
            "Back Right": None,
        }
        self.motor_fault_tooltip = ""

        self.motor_fault_display = QLabel()
        self.motor_fault_display.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.fault_count_label = QLabel(self.motor_fault_display)
        self.fault_count_label.setFixedWidth(self.fault_count_label.sizeHint().height())
        self.fault_count_label.setStyleSheet(
            "color: white;"
            "font-size: 10pt;"
            "background-color: red;"
            "border: 1px solid black"
        )
        self.fault_count_label.move(0, 0)
        self.fault_count_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.update_ui()

        self.layout.addWidget(self.motor_fault_display)

        self.layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(self.layout)

    def event(self, event):
        """
        Overridden event function which intercepts Mouse Enter and Leave events
        Upon Enter, displays a tooltip with all the current motor faults if any
        Upon Leave, hides the tooltip
        :param event: event to check
        """
        if str(event.type()) == "Type.Enter":
            QToolTip.showText(
                QPoint(
                    int(event.globalPosition().x()), int(event.globalPosition().y()),
                ),
                self.motor_fault_tooltip
                if self.motor_fault_tooltip
                else "No Motor Faults :)",
                msecShowTime=20000,
            )
        elif str(event.type()) == "Type.Leave":
            QToolTip.hideText()

        return super().event(event)

    def add_faults_to_tooltip(self, enum_descriptor):
        """
        Adds detailed information about faults for each motor to the tooltip text
        Fault info set by the refresh function
        :param enum_descriptor: descriptor to translate from enum indexes to string values
        """
        for key in self.motor_faults.keys():
            self.drive_enabled = (
                self.drive_enabled and self.motor_faults[key].drive_enabled
            )

            self.fault_count += len(self.motor_faults[key].motor_fault)

            self.motor_fault_tooltip += f"<b>{key} </b>"

            motor_faults = self.motor_faults[key].motor_fault

            if motor_faults:

                self.motor_fault_tooltip += (
                    "<ul>"
                    + "".join(
                        f"<li><p>{get_string_val(enum_descriptor, fault)}</p></li>"
                        for fault in motor_faults
                    )
                    + "</ul>"
                )
            else:
                self.motor_fault_tooltip += f"<ul><li>No Motor Faults :)</li></ul>"

    def update_ui(self):
        """
        Updates the main UI according to the current motor states

        If any motor has a fault,
            - the notification label is visible and displays the current number of faults
            - the color and text of the main display changes
                - if all motors are still enabled, color is yellow and text is "Warning"
                - if any motor is disabled, color is red and text is "Error"
        """
        if self.fault_count:
            self.fault_count_label.setText(str(self.fault_count))
            self.fault_count_label.show()
        else:
            self.fault_count_label.hide()

        self.motor_fault_display.setStyleSheet("background: green;")
        self.motor_fault_display.setText("No Fault")

        if self.fault_count:
            self.motor_fault_display.setStyleSheet(
                "background: yellow;" if self.drive_enabled else "background: red;"
            )
            self.motor_fault_display.setText(
                "Warning" if self.drive_enabled else "Error"
            )

    def refresh(self, message, enum_descriptor):
        """
        Converts the given message into a map of motor name to its fault info
        And calls functions to update the main UI and the tooltip
        :param message: the message to update the widget with
        :param enum_descriptor: descriptor to translate from enum indexes to string values
        """
        self.motor_fault_tooltip = ""
        self.drive_enabled = True
        self.fault_count = 0

        self.motor_faults = {
            "Front Left": message.front_left,
            "Front Right": message.front_right,
            "Back Left": message.back_left,
            "Back Right": message.back_right,
        }

        self.add_faults_to_tooltip(enum_descriptor)
        self.update_ui()
