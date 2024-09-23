from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.Qt.QtWidgets import *
from proto.import_all_protos import *

import software.python_bindings as tbots_cpp

from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.robot_diagnostics.diagnostics_input_widget import ControlMode
from software.thunderscope.common import common_widgets


class DriveAndDribblerWidget(QWidget):
    def __init__(self, proto_unix_io: ProtoUnixIO) -> None:
        """Initialize the widget to control the robot's motors"""
        super().__init__()

        self.proto_unix_io = proto_unix_io

        self.motor_control = MotorControl()

        self.constants = tbots_cpp.create2021RobotConstants()
        layout = QVBoxLayout()

        # Add widgets to layout
        layout.addWidget(self.__setup_direct_velocity())
        layout.addWidget(self.__setup_dribbler())

        self.setLayout(layout)

    def set_x_velocity_slider(self, value: float) -> None:
        """Set the value of the slider controlling the robot's X velocity

        :param value: the value to set the slider to in m/s
        """
        self.x_velocity_slider.setValue(value)

    def set_y_velocity_slider(self, value: float) -> None:
        """Set the value of the slider controlling the robot's Y velocity

        :param value: the value to set the slider to in m/s
        """
        self.y_velocity_slider.setValue(value)

    def set_angular_velocity_slider(self, value: float) -> None:
        """Set the value of the slider controlling the robot's angular velocity

        :param value: the value to set the slider to in rad/s
        """
        self.angular_velocity_slider.setValue(value)

    def set_dribbler_velocity_slider(self, value: float) -> None:
        """Set the value of the slider controlling the robot's dribbler velocity

        :param value: the value to set the slider to in RPM
        """
        self.dribbler_speed_rpm_slider.setValue(value)

    def refresh(self) -> None:
        """Update the currently persisted MotorControl proto based on the widget's slider values
        and sends out the proto
        """
        self.motor_control.dribbler_speed_rpm = int(
            self.dribbler_speed_rpm_slider.value()
        )

        self.motor_control.direct_velocity_control.velocity.x_component_meters = (
            self.x_velocity_slider.value()
        )

        self.motor_control.direct_velocity_control.velocity.y_component_meters = (
            self.y_velocity_slider.value()
        )

        self.motor_control.direct_velocity_control.angular_velocity.radians_per_second = self.angular_velocity_slider.value()

        self.proto_unix_io.send_proto(MotorControl, self.motor_control)

    def __value_change_handler(self, value: float) -> str:
        """Convert the given float value to a string label

        :param value: float value to be converted
        """
        return "%.2f" % float(value)

    def __setup_direct_velocity(self) -> QGroupBox:
        """Create a widget to control the direct velocity of the robot's motors

        :returns: a QGroupBox containing sliders and controls for controlling the
                  direct velocity of the robot's motors
        """
        group_box = QGroupBox("Drive")
        dbox = QVBoxLayout()

        (
            x_layout,
            self.x_velocity_slider,
            self.x_velocity_label,
        ) = common_widgets.create_float_slider(
            "X (m/s)",
            2,
            -self.constants.robot_max_speed_m_per_s,
            self.constants.robot_max_speed_m_per_s,
            1,
        )
        (
            y_layout,
            self.y_velocity_slider,
            self.y_velocity_label,
        ) = common_widgets.create_float_slider(
            "Y (m/s)",
            2,
            -self.constants.robot_max_speed_m_per_s,
            self.constants.robot_max_speed_m_per_s,
            1,
        )
        (
            dps_layout,
            self.angular_velocity_slider,
            self.angular_velocity_label,
        ) = common_widgets.create_float_slider(
            "θ (rad/s)",
            2,
            -self.constants.robot_max_ang_speed_rad_per_s,
            self.constants.robot_max_ang_speed_rad_per_s,
            1,
        )

        # add listener functions for sliders to update label with slider value
        self.x_velocity_slider.valueChanged.connect(
            lambda: self.x_velocity_label.setText(
                self.__value_change_handler(self.x_velocity_slider.value())
            )
        )
        self.y_velocity_slider.valueChanged.connect(
            lambda: self.y_velocity_label.setText(
                self.__value_change_handler(self.y_velocity_slider.value())
            )
        )
        self.angular_velocity_slider.valueChanged.connect(
            lambda: self.angular_velocity_label.setText(
                self.__value_change_handler(self.angular_velocity_slider.value())
            )
        )

        self.stop_and_reset_direct = QPushButton("Stop and Reset")
        self.stop_and_reset_direct.clicked.connect(self.__reset_direct_sliders)

        dbox.addLayout(x_layout)
        dbox.addLayout(y_layout)
        dbox.addLayout(dps_layout)
        dbox.addWidget(
            self.stop_and_reset_direct, alignment=Qt.AlignmentFlag.AlignCenter
        )

        group_box.setLayout(dbox)

        return group_box

    def __setup_dribbler(self) -> QGroupBox:
        """Create a widget to control the dribbler speed

        :returns: a QGroupBox containing a slider and controls for controlling the
                  robot's dribbler speed
        """
        group_box = QGroupBox("Dribbler")
        dbox = QVBoxLayout()

        (
            dribbler_layout,
            self.dribbler_speed_rpm_slider,
            self.dribbler_speed_rpm_label,
        ) = common_widgets.create_float_slider(
            "RPM",
            1,
            self.constants.indefinite_dribbler_speed_rpm,
            -self.constants.indefinite_dribbler_speed_rpm,
            1,
        )

        self.dribbler_speed_rpm_slider.valueChanged.connect(
            lambda: self.dribbler_speed_rpm_label.setText(
                self.__value_change_handler(self.dribbler_speed_rpm_slider.value())
            )
        )

        self.stop_and_reset_dribbler = QPushButton("Stop and Reset")
        self.stop_and_reset_dribbler.clicked.connect(self.__reset_dribbler_slider)

        dbox.addLayout(dribbler_layout)
        dbox.addWidget(
            self.stop_and_reset_dribbler, alignment=Qt.AlignmentFlag.AlignCenter
        )
        group_box.setLayout(dbox)

        return group_box

    def update_widget_accessibility(self, mode: ControlMode) -> None:
        """Disables or enables all sliders and buttons depending on the given control mode.
        Sliders are enabled in DIAGNOSTICS mode, and disabled in HANDHELD mode

        :param mode: the current control mode
        """
        if mode == ControlMode.DIAGNOSTICS:
            # enable all sliders by adding listener to update label with slider value
            common_widgets.enable_slider(self.x_velocity_slider)
            common_widgets.enable_slider(self.y_velocity_slider)
            common_widgets.enable_slider(self.angular_velocity_slider)
            common_widgets.enable_slider(self.dribbler_speed_rpm_slider)

            # enable buttons
            common_widgets.enable_button(self.stop_and_reset_dribbler)
            common_widgets.enable_button(self.stop_and_reset_direct)

        elif mode == ControlMode.HANDHELD:
            self.__reset_all_sliders()

            # disable all sliders by adding listener to keep slider value the same
            common_widgets.disable_slider(self.x_velocity_slider)
            common_widgets.disable_slider(self.y_velocity_slider)
            common_widgets.disable_slider(self.angular_velocity_slider)
            common_widgets.disable_slider(self.dribbler_speed_rpm_slider)

            # disable buttons
            common_widgets.disable_button(self.stop_and_reset_dribbler)
            common_widgets.disable_button(self.stop_and_reset_direct)

    def __reset_direct_sliders(self) -> None:
        """Reset direct sliders back to 0"""
        self.x_velocity_slider.setValue(0)
        self.y_velocity_slider.setValue(0)
        self.angular_velocity_slider.setValue(0)

    def __reset_dribbler_slider(self) -> None:
        """Reset the dribbler slider back to 0"""
        self.dribbler_speed_rpm_slider.setValue(0)

    def __reset_all_sliders(self) -> None:
        """Reset all sliders back to 0"""
        self.__reset_direct_sliders()
        self.__reset_dribbler_slider()
