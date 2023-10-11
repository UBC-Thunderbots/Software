from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt.QtCore import Qt, QPoint, QByteArray, QBuffer, QIODeviceBase, QEvent
from pyqtgraph.Qt import QtGui
from software.thunderscope.robot_diagnostics.motor_fault_icons.motor_fault_icon_loader import (
    get_no_fault_icon,
    get_stopped_icon,
    get_warning_icon,
)
from software.thunderscope.common.proto_parameter_tree_util import get_string_val
from software.thunderscope.common.common_widgets import display_tooltip
from Typing import Any

class MotorFaultView(QWidget):
    """
    Class to visualise information about motor faults from robot status

    Displays if any of the motors have a fault and / or are disabled
    Displays specific faults for each motor in a tooltip
    """

    def __init__(self) -> None:
        """
        Initializes the motor fault view widget

        Sets all motor fault info to default and sets the tooltip to empty
        Initialises the main label and the fault count notification label
        """
        super().__init__()
        self.layout = QHBoxLayout()

        self.enabled = None
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
            "border: 1px solid black;"
        )
        self.fault_count_label.move(0, 0)
        self.fault_count_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.fault_count_label.hide()

        self.update_ui()

        self.layout.addWidget(self.motor_fault_display)

        self.layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(self.layout)

    def event(self, event: QEvent) -> bool:
        """
        Overridden event function which intercepts all events
        On hover, displays a tooltip with all the current motor faults if any
        If tooltip text is empty (uninitialized), displays "No Signal Yet"
        :param event: event to check
        """
        display_tooltip(
            event,
            self.motor_fault_tooltip if self.motor_fault_tooltip else "No Signal Yet",
        )

        return super().event(event)

    def add_faults_to_tooltip(self, enum_descriptor: Any) -> None:
        """
        Adds detailed information about faults for each motor to the tooltip text
        Adds an icon for each motor to indicate if it has stopped, has a warning, or is fine
        Adds list of faults for each motor to tooltip
        Fault info set by the refresh function
        :param enum_descriptor: descriptor to translate from enum indexes to string values
        """
        for key in self.motor_faults.keys():
            self.enabled = self.enabled and self.motor_faults[key].enabled

            motor_faults = self.motor_faults[key].motor_faults

            byte_array = QByteArray()
            buffer = QBuffer(byte_array)
            buffer.open(QIODeviceBase.OpenModeFlag.ReadWrite)

            if self.enabled and motor_faults:
                get_warning_icon().save(buffer, "PNG")
            elif motor_faults:
                get_stopped_icon().save(buffer, "PNG")
            else:
                get_no_fault_icon().save(buffer, "PNG")

            image_data = byte_array.toBase64()

            self.fault_count += len(self.motor_faults[key].motor_faults)

            self.motor_fault_tooltip += (
                "<p>"
                "<img src='data:image/png;base64, "
                f"{str(image_data.data(), encoding='utf-8')}'"
                'width="10" height="10"/>'
                "&nbsp;"
                f"<b>{key}</b>"
            )

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

    def update_ui(self) -> None:
        """
        Updates the main UI according to the current motor states

        If the widget has not received a signal yet (self.enabled = None),
        sets the UI to the default uninitialized state

        Else If any motor has a fault,
        - the notification label is visible and displays the current number of faults
        - the color and text of the main display changes
            - if all motors are still enabled, color is yellow and text is "Warning"
            - if any motor is disabled, color is red and text is "Error"
        """
        if self.enabled is None:
            self.motor_fault_display.setStyleSheet("background: grey; color: white;")
            self.motor_fault_display.setText("No Signal")
        else:
            if self.fault_count:
                self.motor_fault_display.setStyleSheet(
                    "background: yellow; color: black;"
                    if self.enabled
                    else "background: red;"
                )
                self.motor_fault_display.setText("Warning" if self.enabled else "Error")

                self.fault_count_label.setText(str(self.fault_count))
                self.fault_count_label.show()
            else:
                self.motor_fault_display.setStyleSheet("background: green;")
                self.motor_fault_display.setText("No Fault")

                self.fault_count_label.hide()

    def refresh(self, motor_status: MotorStatus, enum_descriptor: Any) -> None:
        """
        Converts the given message into a map of motor name to its fault info
        And calls functions to update the main UI and the tooltip
        :param motor_status: the MotorStatus message to update the widget with
        :param enum_descriptor: descriptor to translate from enum indexes to string values
        """
        self.motor_fault_tooltip = ""
        self.enabled = True
        self.fault_count = 0

        self.motor_faults = {
            "Front Left": motor_status.front_left,
            "Front Right": motor_status.front_right,
            "Back Left": motor_status.back_left,
            "Back Right": motor_status.back_right,
            "Dribbler": motor_status.dribbler,
        }

        self.add_faults_to_tooltip(enum_descriptor)
        self.update_ui()
