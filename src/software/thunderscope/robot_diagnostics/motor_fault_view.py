from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt.QtCore import Qt, QPoint, QEvent
from pyqtgraph.Qt import QtGui
from enum import Enum
from software.thunderscope.robot_diagnostics.motor_fault_icons.motor_fault_icon_loader import (
    get_no_fault_icon,
    get_stopped_icon,
    get_warning_icon,
)
from software.thunderscope.common.common_widgets import get_string_val


class MotorFaults(Enum):
    NO_FAULT = 0
    NO_STOP_FAULT = 1
    STOP_FAULT = 2


class MotorFaultWidget(QWidget):

    WIDGET_SIZE_SCALE = 2.2

    def __init__(self, index):
        super().__init__()

        self.index = index
        self.status = MotorFaults.NO_FAULT
        self.tooltip_visible = False

        self.layout = QHBoxLayout()
        self.container_widget = QLabel()
        self.container_widget.setFixedWidth(
            self.container_widget.sizeHint().height() * self.WIDGET_SIZE_SCALE
        )
        self.container_widget.setFixedHeight(
            self.container_widget.sizeHint().height() * self.WIDGET_SIZE_SCALE
        )
        self.container_widget.setScaledContents(True)

        self.layout.addWidget(self.container_widget)
        self.layout.setContentsMargins(0, 0, 0, 0)

        self.index_label = QLabel(self.container_widget)
        self.index_label.setText(index)
        self.index_label.setStyleSheet(
            "color: black;" "font-weight: bold;" "font-size: 12pt"
        )
        self.index_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.index_label.setFixedWidth(self.index_label.sizeHint().height())
        self.index_label.move(
            self.container_widget.sizeHint().height() * self.WIDGET_SIZE_SCALE / 2
            - self.index_label.sizeHint().height() / 2,
            self.container_widget.sizeHint().height() * self.WIDGET_SIZE_SCALE / 2
            - self.index_label.sizeHint().height() / 2,
        )

        self.fault_count_label = QLabel(self.container_widget)
        self.fault_count_label.setFixedWidth(self.fault_count_label.sizeHint().height())
        self.fault_count_label.setStyleSheet(
            "color: white;"
            "font-size: 8pt;"
            "background-color: red;"
            "border: 1px solid black"
        )
        self.fault_count_label.move(
            self.container_widget.sizeHint().height() * self.WIDGET_SIZE_SCALE * 1.2
            - self.index_label.sizeHint().height(),
            self.container_widget.sizeHint().height() * self.WIDGET_SIZE_SCALE * 1.2
            - self.index_label.sizeHint().height()
        )
        self.fault_count_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.fault_count_label.hide()

        self.setLayout(self.layout)
        self.refresh_fault_icon()

    def refresh(self, fault_values, is_drive_enabled, enum_descriptor):
        # self.container_widget.setToolTip(
        #     "<b>Hello</b> <i>Qt!</i>" + ", ".join(
        #         get_string_val(enum_descriptor, fault) for fault in fault_values
        #     )
        #     if fault_values
        #     else "No Motor Faults :)"
        # )

        if fault_values:
            self.fault_count_label.setText(str(len(fault_values)))
            self.fault_count_label.show()
        else:
            self.fault_count_label.hide()

        new_status = self.get_new_status(fault_values, is_drive_enabled)

        if new_status != self.status:
            self.status = new_status
            self.refresh_fault_icon()

    def get_new_status(self, fault_values, is_drive_enabled):
        if not is_drive_enabled:
            return MotorFaults.STOP_FAULT
        if fault_values:
            return MotorFaults.NO_STOP_FAULT
        else:
            return MotorFaults.NO_FAULT

    def refresh_fault_icon(self):
        if self.status == MotorFaults.NO_FAULT:
            self.container_widget.setPixmap(get_no_fault_icon())
        elif self.status == MotorFaults.NO_STOP_FAULT:
            self.container_widget.setPixmap(get_warning_icon())
        elif self.status == MotorFaults.STOP_FAULT:
            self.container_widget.setPixmap(get_stopped_icon())

    def event(self, event):
        if event.type() == QEvent.HoverEnter:
            QToolTip.showText(
                QPoint(
                    int(event.globalPosition().x()),
                    int(event.globalPosition().y()),
                ),
                "<b>Hello</b> <i>Qt!</i>",
                msecShowTime=5000
            )
        elif event.type() == QEvent.HoverLeave:
            QToolTip.hideText()


class MotorFaultView(QWidget):
    def __init__(self):
        super().__init__()
        self.layout = QHBoxLayout()

        self.front_left_motor = MotorFaultWidget("FL")
        self.front_right_motor = MotorFaultWidget("FR")
        self.back_left_motor = MotorFaultWidget("BL")
        self.back_right_motor = MotorFaultWidget("BR")

        self.layout.addWidget(self.front_left_motor)
        self.layout.addWidget(self.front_right_motor)
        self.layout.addWidget(self.back_left_motor)
        self.layout.addWidget(self.back_right_motor)

        self.layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(self.layout)

    def refresh(self, message, enum_descriptor):
        self.front_left_motor.refresh(
            message.front_left.motor_fault, message.front_left.drive_enabled, enum_descriptor
        )
        self.front_right_motor.refresh(
            message.front_right.motor_fault, message.front_right.drive_enabled, enum_descriptor
        )
        self.back_left_motor.refresh(
            message.back_left.motor_fault, message.back_left.drive_enabled, enum_descriptor
        )
        self.back_right_motor.refresh(
            message.back_right.motor_fault, message.back_right.drive_enabled, enum_descriptor
        )
