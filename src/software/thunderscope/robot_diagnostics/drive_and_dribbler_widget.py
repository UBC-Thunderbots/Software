from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.Qt.QtWidgets import *
from proto.import_all_protos import *
from enum import IntEnum
import software.python_bindings as tbots_cpp

from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.common import common_widgets


class ControlMode(IntEnum):
    """Enum for the 2 drive modes (direct velocity and per-motor)"""

    VELOCITY = 0
    MOTOR = 1


class DriveAndDribblerWidget(QWidget):
    """This widget provides an interface for controlling our robots'
    drive and dribbler functionalities. It has sliders for manipulating
    the direct velocity of the robot's motors as well as the speed of the
    dribbler. The slider values are used to create and send MotorControl
    messages to the robots.
    """

    def __init__(self, proto_unix_io: ProtoUnixIO) -> None:
        """Initialize the DriveAndDribblerWidget

        :param proto_unix_io: ProtoUnixIO to send messages to the robot
        """
        super().__init__()

        self.proto_unix_io = proto_unix_io

        self.constants = tbots_cpp.create2021RobotConstants()

        self.enabled = True
        self.control_mode = ControlMode.VELOCITY

        layout = QVBoxLayout()
        layout.addWidget(self.__setup_drive_switch_radio())
        layout.addWidget(self.__setup_direct_velocity_widgets())
        layout.addWidget(self.__setup_per_motor_widgets())
        layout.addWidget(self.__setup_dribbler_widgets())
        self.setLayout(layout)
        self.use_direct_velocity.click()

    def enable(self) -> None:
        """Enable all sliders and buttons in the DriveAndDribblerWidget"""
        if self.enabled:
            return

        self.enabled = True

        common_widgets.enable_slider(self.x_velocity_slider)
        common_widgets.enable_slider(self.y_velocity_slider)
        common_widgets.enable_slider(self.angular_velocity_slider)
        common_widgets.enable_slider(self.front_left_motor_slider)
        common_widgets.enable_slider(self.front_right_motor_slider)
        common_widgets.enable_slider(self.back_left_motor_slider)
        common_widgets.enable_slider(self.back_left_motor_slider)
        common_widgets.enable_slider(self.dribbler_speed_rpm_slider)
        common_widgets.enable_button(self.stop_and_reset_dribbler)
        common_widgets.enable_button(self.stop_and_reset_direct)

    def disable(self) -> None:
        """Disable all sliders and buttons in the DriveAndDribblerWidget"""
        if not self.enabled:
            return

        self.enabled = False

        self.__reset_all_sliders()
        common_widgets.disable_slider(self.x_velocity_slider)
        common_widgets.disable_slider(self.y_velocity_slider)
        common_widgets.disable_slider(self.angular_velocity_slider)
        common_widgets.disable_slider(self.front_left_motor_slider)
        common_widgets.disable_slider(self.front_right_motor_slider)
        common_widgets.disable_slider(self.back_left_motor_slider)
        common_widgets.disable_slider(self.back_left_motor_slider)
        common_widgets.disable_slider(self.dribbler_speed_rpm_slider)
        common_widgets.disable_button(self.stop_and_reset_dribbler)
        common_widgets.disable_button(self.stop_and_reset_direct)

    def override_slider_values(self, motor_control: MotorControl) -> None:
        """Set the widget's sliders to match the values in the given MotorControl

        :param motor_control: the MotorControl values to override the widget's sliders with
        """
        self.x_velocity_slider.setValue(
            motor_control.direct_velocity_control.velocity.x_component_meters
        )
        self.y_velocity_slider.setValue(
            motor_control.direct_velocity_control.velocity.y_component_meters
        )
        self.angular_velocity_slider.setValue(
            motor_control.direct_velocity_control.angular_velocity.radians_per_second
        )
        self.front_left_motor_slider.setValue(
            motor_control.direct_velocity_control.front_left_wheel_velocity
        )
        self.front_right_motor_slider.setValue(
            motor_control.direct_velocity_control.front_right_wheel_velocity
        )
        self.back_left_motor_slider.setValue(
            motor_control.direct_velocity_control.back_left_wheel_velocity
        )
        self.back_right_motor_slider.setValue(
            motor_control.direct_velocity_control.back_right_wheel_velocity
        )

        self.dribbler_speed_rpm_slider.setValue(motor_control.dribbler_speed_rpm)

    def refresh(self) -> None:
        """Refresh the widget and send the MotorControl message with the current values depending on the ControlMode"""
        motor_control = MotorControl()
        motor_control.dribbler_speed_rpm = int(self.dribbler_speed_rpm_slider.value())
        if self.control_mode == ControlMode.VELOCITY:
            motor_control.direct_velocity_control.velocity.x_component_meters = (
                self.x_velocity_slider.value()
            )
            motor_control.direct_velocity_control.velocity.y_component_meters = (
                self.y_velocity_slider.value()
            )
            motor_control.direct_velocity_control.angular_velocity.radians_per_second =\
                self.angular_velocity_slider.value()
        else:
            motor_control.direct_per_wheel_control.front_left_wheel_velocity = (
                self.front_left_motor_slider.value()
            )
            motor_control.direct_per_wheel_control.front_right_wheel_velocity = (
                self.front_right_motor_slider.value()
            )
            motor_control.direct_per_wheel_control.back_left_wheel_velocity = (
                self.back_left_motor_slider.value()
            )
            motor_control.direct_per_wheel_control.back_right_wheel_velocity = (
                self.back_right_motor_slider.value()
            )

        self.proto_unix_io.send_proto(MotorControl, motor_control)

    def __setup_direct_velocity_widgets(self) -> QGroupBox:
        """Create a widget to control the direct velocity of the robot's motors

        :returns: a QGroupBox containing sliders and controls for controlling the
                  direct velocity of the robot's motors
        """
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

        self.x_velocity_slider.floatValueChanged.connect(
            lambda new_value: self.x_velocity_label.setText("%.2f" % new_value)
        )
        self.y_velocity_slider.floatValueChanged.connect(
            lambda new_value: self.y_velocity_label.setText("%.2f" % new_value)
        )
        self.angular_velocity_slider.floatValueChanged.connect(
            lambda new_value: self.angular_velocity_label.setText("%.2f" % new_value)
        )

        self.stop_and_reset_direct = QPushButton("Stop and Reset")
        self.stop_and_reset_direct.clicked.connect(self.__reset_direct_sliders)

        vbox = QVBoxLayout()
        vbox.addLayout(x_layout)
        vbox.addLayout(y_layout)
        vbox.addLayout(dps_layout)
        vbox.addWidget(
            self.stop_and_reset_direct, alignment=Qt.AlignmentFlag.AlignCenter
        )

        group_box = QGroupBox("Velocity Control")
        group_box.setLayout(vbox)
        self.direct_velocity_widget = group_box

        return group_box

    def __setup_per_motor_widgets(self) -> QGroupBox:
        """
        :returns: a QGroupBox containing sliders and controls for controlling individual
        speed of the robot's motors.
        """
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

        self.front_left_motor_slider.floatValueChanged.connect(
            lambda new_value: self.front_left_motor_label.setText("%.2f" % new_value)
        )
        self.front_right_motor_slider.floatValueChanged.connect(
            lambda new_value: self.front_right_motor_label.setText("%.2f" % new_value)
        )
        self.back_left_motor_slider.floatValueChanged.connect(
            lambda new_value: self.back_left_motor_label.setText("%.2f" % new_value)
        )
        self.back_right_motor_slider.floatValueChanged.connect(
            lambda new_value: self.back_right_motor_label.setText("%.2f" % new_value)
        )

        self.stop_and_reset_per_motor = QPushButton("Stop and Reset")
        self.stop_and_reset_per_motor.clicked.connect(self.__reset_motor_sliders)

        vbox = QVBoxLayout()
        vbox.addLayout(fl_layout)
        vbox.addLayout(fr_layout)
        vbox.addLayout(bl_layout)
        vbox.addLayout(br_layout)
        vbox.addWidget(
            self.stop_and_reset_per_motor, alignment=Qt.AlignmentFlag.AlignCenter
        )

        group_box = QGroupBox("Per Motor Control")
        group_box.setLayout(vbox)
        self.per_motor_widget = group_box

        return group_box

    def __setup_drive_switch_radio(self) -> QGroupBox:
        """Create a radio button widget to switch between per-motor and velocity drive modes

        :returns: The group box of the radio button switch.
        """
        group_box = QGroupBox()
        vbox = QVBoxLayout()
        self.connect_options_group = QButtonGroup()
        radio_button_names = ["Velocity Control", "Per Motor Control"]
        self.connect_options_box, self.connect_options = common_widgets.create_radio(
            radio_button_names, self.connect_options_group
        )
        self.use_direct_velocity = self.connect_options[ControlMode.VELOCITY]
        self.use_per_motor = self.connect_options[ControlMode.MOTOR]
        self.use_direct_velocity.clicked.connect(
            lambda: self.toggle_control_mode(ControlMode.VELOCITY)
        )
        self.use_per_motor.clicked.connect(
            lambda: self.toggle_control_mode(ControlMode.MOTOR)
        )
        vbox.addWidget(self.connect_options_box)
        group_box.setLayout(vbox)

        return group_box

    def __setup_dribbler_widgets(self) -> QGroupBox:
        """Create a widget to control the dribbler speed

        :returns: a QGroupBox containing a slider and controls for controlling the
                  robot's dribbler speed
        """
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

        self.dribbler_speed_rpm_slider.floatValueChanged.connect(
            lambda new_value: self.dribbler_speed_rpm_label.setText("%.2f" % new_value)
        )

        self.stop_and_reset_dribbler = QPushButton("Stop and Reset")
        self.stop_and_reset_dribbler.clicked.connect(self.__reset_dribbler_slider)

        vbox = QVBoxLayout()
        vbox.addLayout(dribbler_layout)
        vbox.addWidget(
            self.stop_and_reset_dribbler, alignment=Qt.AlignmentFlag.AlignCenter
        )

        group_box = QGroupBox("Dribbler")
        group_box.setLayout(vbox)

        return group_box

    def toggle_control_mode(self, use_control_mode: IntEnum) -> None:
        """Switches between 'Direct Velocity' and 'Per Motor' drive modes.

        :param use_control_mode: ControlMode.VELOCITY or ControlMode.MOTOR, switch to that mode.
        """
        self.control_mode = use_control_mode
        # reset sliders
        self.__reset_motor_sliders()
        self.__reset_direct_sliders()

        motor_control = MotorControl()
        if use_control_mode == ControlMode.VELOCITY:
            # Show the direct velocity widget
            motor_control.ClearField("direct_per_wheel_control")
            self.direct_velocity_widget.setVisible(True)
            self.per_motor_widget.setVisible(False)
        else:
            # Show the per motor widget
            motor_control.ClearField("direct_velocity_control")
            self.direct_velocity_widget.setVisible(False)
            self.per_motor_widget.setVisible(True)
        self.proto_unix_io.send_proto(MotorControl, motor_control)

    def __reset_direct_sliders(self) -> None:
        """Reset the direct velocity sliders back to 0"""
        self.x_velocity_slider.setValue(0)
        self.y_velocity_slider.setValue(0)
        self.angular_velocity_slider.setValue(0)

    def __reset_motor_sliders(self) -> None:
        """Reset direct sliders back to 0"""
        self.front_left_motor_slider.setValue(0)
        self.front_right_motor_slider.setValue(0)
        self.back_left_motor_slider.setValue(0)
        self.back_right_motor_slider.setValue(0)

    def __reset_dribbler_slider(self) -> None:
        """Reset the dribbler speed slider back to 0"""
        self.dribbler_speed_rpm_slider.setValue(0)

    def __reset_all_sliders(self) -> None:
        """Reset all sliders back to 0"""
        self.__reset_direct_sliders()
        self.__reset_dribbler_slider()
        self.__reset_direct_sliders()
