from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.Qt.QtWidgets import *
import time
import software.python_bindings as tbots_cpp
from software.thunderscope.common import common_widgets
from proto.import_all_protos import *
from software.thunderscope.proto_unix_io import ProtoUnixIO
from enum import IntEnum


class DriveMode(IntEnum):
    """Enum for the 2 drive modes (direct velocity and per-motor)"""
    VELOCITY = 0
    MOTOR = 1


class DriveAndDribblerWidget(QWidget):
    def __init__(self, proto_unix_io: ProtoUnixIO) -> None:
        """Initialize the widget to control the robot's motors

        :param proto_unix_io: the proto_unix_io object
        """
        self.input_a = time.time()
        self.constants = tbots_cpp.create2021RobotConstants()
        QWidget.__init__(self)

        layout = QVBoxLayout()
        self.drive_widget = QStackedWidget()

        self.proto_unix_io = proto_unix_io

        # create swappable widget system using stacked widgets
        self.direct_velocity_widget = self.setup_direct_velocity("Drive - Direct Velocity")
        self.per_motor_widget = self.setup_per_motor("Drive - Per Motor")
        self.drive_widget.addWidget(self.direct_velocity_widget)
        self.drive_widget.addWidget(self.per_motor_widget)

        layout.addWidget(self.setup_drive_switch_radio("Drive Mode Switch"))
        layout.addWidget(self.drive_widget)
        layout.addWidget(self.setup_dribbler("Dribbler"))

        self.drive_mode = DriveMode.MOTOR
        self.use_direct_velocity.setChecked(True)
        self.setLayout(layout)
        self.toggle_drive_mode(True)
        self.toggle_dribbler_sliders(True)

    def refresh(self) -> None:
        """Refresh the widget and send the a MotorControl message with the current values"""
        motor_control = MotorControl()
        motor_control.dribbler_speed_rpm = int(self.dribbler_speed_rpm_slider.value())

        if not self.drive_mode == DriveMode.VELOCITY:
            motor_control.ClearField("direct_per_wheel_control")
            motor_control.direct_velocity_drive.velocity.x_component_meters = (
                self.x_velocity_slider.value()
            )
            motor_control.direct_velocity_drive.velocity.y_component_meters = (
                self.y_velocity_slider.value()
            )
            motor_control.direct_velocity_drive.angular_velocity.radians_per_second = (
                self.angular_velocity_slider.value()
            )
        else:
            motor_control.ClearField("direct_velocity_control")
            motor_control.direct_per_wheel_drive.front_left_wheel_velocity = (
                self.front_left_motor_slider.value()
            )
            motor_control.direct_per_wheel_drive.front_right_wheel_velocity = (
                self.front_right_motor_slider.value()
            )
            motor_control.direct_per_wheel_drive.back_left_wheel_velocity = (
                self.back_left_motor_slider.value()
            )
            motor_control.direct_per_wheel_drive.back_right_wheel_velocity = (
                self.back_right_motor_slider.value()
            )

        self.proto_unix_io.send_proto(MotorControl, motor_control)

    def value_change(self, value: float) -> str:
        """Converts the given float value to a string label

        :param value: float value to be converted
        """
        value = float(value)
        value_str = "%.2f" % value
        return value_str

    def setup_drive_switch_radio(self, title: str) -> QGroupBox:
        """Create a radio button widget to switch between per-motor and velocity drive modes

        :param title: vbox name
        """
        group_box = QGroupBox(title)
        vbox = QVBoxLayout()
        self.connect_options_group = QButtonGroup()
        radio_button_names = ["Velocity Control", "Per Motor Control"]
        self.connect_options_box, self.connect_options = common_widgets.create_radio(
            radio_button_names, self.connect_options_group
        )
        self.use_direct_velocity = self.connect_options[DriveMode.VELOCITY]
        self.use_per_motor = self.connect_options[DriveMode.MOTOR]
        self.use_direct_velocity.clicked.connect(
            lambda: self.toggle_drive_mode(True)
        )
        self.use_per_motor.clicked.connect(
            lambda: self.toggle_drive_mode(False)
        )
        vbox.addWidget(self.connect_options_box)
        group_box.setLayout(vbox)

        return group_box

    def setup_direct_velocity(self, title: str) -> QGroupBox:
        """Create a widget to control the direct velocity of the robot's motors

        :param title: the name of the slider
        """
        group_box = QGroupBox(title)
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
        common_widgets.enable_slider(
            self.x_velocity_slider, self.x_velocity_label, self.value_change
        )
        common_widgets.enable_slider(
            self.y_velocity_slider, self.y_velocity_label, self.value_change
        )
        common_widgets.enable_slider(
            self.angular_velocity_slider, self.angular_velocity_label, self.value_change
        )

        self.stop_and_reset_direct = QPushButton("Stop and Reset")
        self.stop_and_reset_direct.clicked.connect(self.reset_direct_sliders)

        dbox.addLayout(x_layout)
        dbox.addLayout(y_layout)
        dbox.addLayout(dps_layout)
        dbox.addWidget(
            self.stop_and_reset_direct, alignment=Qt.AlignmentFlag.AlignCenter
        )

        group_box.setLayout(dbox)

        return group_box

    def setup_per_motor(self, title: str) -> QGroupBox:
        """Create a widget to control the rotation rate of each motor

        :param title: the name of the group box
        """

        group_box = QGroupBox(title)
        dbox = QVBoxLayout()

        (
            fl_layout,
            self.front_left_motor_slider,
            self.front_left_motor_label,
        ) = common_widgets.create_float_slider(
            "Front Left Motor (m/s)",
            2,
            -self.constants.robot_max_speed_m_per_s,
            self.constants.robot_max_speed_m_per_s,
            1,
        )
        (
            fr_layout,
            self.front_right_motor_slider,
            self.front_right_motor_label,
        ) = common_widgets.create_float_slider(
            "Front Right Motor (m/s)",
            2,
            -self.constants.robot_max_speed_m_per_s,
            self.constants.robot_max_speed_m_per_s,
            1,
        )
        (
            bl_layout,
            self.back_left_motor_slider,
            self.back_left_motor_label,
        ) = common_widgets.create_float_slider(
            "Back Left Motor (m/s)",
            2,
            -self.constants.robot_max_speed_m_per_s,
            self.constants.robot_max_speed_m_per_s,
            1,
        )
        (
            br_layout,
            self.back_right_motor_slider,
            self.back_right_motor_label,
        ) = common_widgets.create_float_slider(
            "Back Right Motor (m/s)",
            2,
            -self.constants.robot_max_speed_m_per_s,
            self.constants.robot_max_speed_m_per_s,
            1,
        )

        # Add listener functions for each motor's slider to update labels with slider values
        common_widgets.enable_slider(
            self.front_left_motor_slider, self.front_left_motor_label, self.value_change
        )
        common_widgets.enable_slider(
            self.front_right_motor_slider, self.front_right_motor_label, self.value_change
        )
        common_widgets.enable_slider(
            self.back_left_motor_slider, self.back_left_motor_label, self.value_change
        )
        common_widgets.enable_slider(
            self.back_right_motor_slider, self.back_right_motor_label, self.value_change
        )

        # Stop and Reset button for per-motor sliders
        self.stop_and_reset_per_motor = QPushButton("Stop and Reset")
        self.stop_and_reset_per_motor.clicked.connect(self.reset_motor_sliders)

        # Adding layouts to the box
        dbox.addLayout(fl_layout)
        dbox.addLayout(fr_layout)
        dbox.addLayout(bl_layout)
        dbox.addLayout(br_layout)
        dbox.addWidget(
            self.stop_and_reset_per_motor, alignment=Qt.AlignmentFlag.AlignCenter
        )

        group_box.setLayout(dbox)

        return group_box

    def setup_dribbler(self, title: str) -> QGroupBox:
        """Create a widget to control the dribbler RPM

        :param title: the name of the slider
        """
        group_box = QGroupBox(title)
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

        # add listener function to update label with slider value
        common_widgets.enable_slider(
            self.dribbler_speed_rpm_slider,
            self.dribbler_speed_rpm_label,
            self.value_change,
        )

        self.stop_and_reset_dribbler = QPushButton("Stop and Reset")
        self.stop_and_reset_dribbler.clicked.connect(self.reset_dribbler_slider)

        dbox.addLayout(dribbler_layout)
        dbox.addWidget(
            self.stop_and_reset_dribbler, alignment=Qt.AlignmentFlag.AlignCenter
        )
        group_box.setLayout(dbox)

        return group_box

    def toggle_drive_mode(self, use_drive_mode: IntEnum) -> None:
        """Switches between 'Direct Velocity' and 'Per Motor' drive modes.

        :param use_drive_mode: DriveMode.VELOCITY or DriveMode.MOTOR, switch to that mode.
        """
        # reset sliders
        self.reset_motor_sliders()
        self.reset_direct_sliders()
        self.disconnect_direct_sliders()
        self.disconnect_motor_sliders()

        if use_drive_mode == DriveMode.VELOCITY:
            # Show the direct velocity widget
            self.drive_widget.setCurrentWidget(self.direct_velocity_widget)

            # Enable direct sliders and disable motor sliders
            common_widgets.enable_slider(
                self.x_velocity_slider, self.x_velocity_label, self.value_change
            )
            common_widgets.enable_slider(
                self.y_velocity_slider, self.y_velocity_label, self.value_change
            )
            common_widgets.enable_slider(
                self.angular_velocity_slider, self.angular_velocity_label, self.value_change
            )

            common_widgets.disable_slider(self.front_left_motor_slider)
            common_widgets.disable_slider(self.front_right_motor_slider)
            common_widgets.disable_slider(self.back_left_motor_slider)
            common_widgets.disable_slider(self.back_right_motor_slider)

            common_widgets.change_button_state(self.stop_and_reset_direct, True)
            common_widgets.change_button_state(self.stop_and_reset_per_motor, False)

        else:
            # Show the per motor widget
            self.drive_widget.setCurrentWidget(self.per_motor_widget)

            # Enable motor sliders and disable direct sliders
            common_widgets.enable_slider(
                self.front_left_motor_slider, self.front_left_motor_label, self.value_change
            )
            common_widgets.enable_slider(
                self.front_right_motor_slider, self.front_right_motor_label, self.value_change
            )
            common_widgets.enable_slider(
                self.back_left_motor_slider, self.back_left_motor_label, self.value_change
            )
            common_widgets.enable_slider(
                self.back_right_motor_slider, self.back_right_motor_label, self.value_change
            )

            common_widgets.disable_slider(self.x_velocity_slider)
            common_widgets.disable_slider(self.y_velocity_slider)
            common_widgets.disable_slider(self.angular_velocity_slider)

            common_widgets.change_button_state(self.stop_and_reset_per_motor, True)
            common_widgets.change_button_state(self.stop_and_reset_direct, False)

    def toggle_dribbler_sliders(self, enable: bool) -> None:
        """Enables or disables dribbler sliders.

        :param enable: True to enable, False to disable
        """
        if enable:
            common_widgets.enable_slider(
                self.dribbler_speed_rpm_slider,
                self.dribbler_speed_rpm_label,
                self.value_change,
            )
            common_widgets.change_button_state(self.stop_and_reset_dribbler, True)
        else:
            common_widgets.disable_slider(self.dribbler_speed_rpm_slider)
            common_widgets.change_button_state(self.stop_and_reset_dribbler, False)

    def disconnect_direct_sliders(self) -> None:
        """Disconnect listener for changing values for motor sliders"""
        self.x_velocity_slider.valueChanged.disconnect()
        self.y_velocity_slider.valueChanged.disconnect()
        self.angular_velocity_slider.valueChanged.disconnect()

    def disconnect_dribbler_sliders(self) -> None:
        self.dribbler_speed_rpm_slider.valueChanged.disconnect()

    def disconnect_motor_sliders(self) -> None:
        self.front_left_motor_slider.valueChanged.disconnect()
        self.front_right_motor_slider.valueChanged.disconnect()
        self.back_left_motor_slider.valueChanged.disconnect()
        self.back_right_motor_slider.valueChanged.disconnect()

    def reset_direct_sliders(self) -> None:
        """Reset direct sliders back to 0"""
        self.x_velocity_slider.setValue(0)
        self.y_velocity_slider.setValue(0)
        self.angular_velocity_slider.setValue(0)

    def reset_motor_sliders(self) -> None:
        """Reset direct sliders back to 0"""
        self.front_left_motor_slider.setValue(0)
        self.front_right_motor_slider.setValue(0)
        self.back_left_motor_slider.setValue(0)
        self.back_right_motor_slider.setValue(0)

    def reset_dribbler_slider(self) -> None:
        """Reset the dribbler slider back to 0"""
        self.dribbler_speed_rpm_slider.setValue(0)

    def reset_all_sliders(self) -> None:
        """Reset all sliders back to 0"""
        self.reset_direct_sliders()
        self.reset_motor_sliders()
        self.reset_dribbler_slider()
