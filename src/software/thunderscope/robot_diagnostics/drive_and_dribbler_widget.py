from threading import Thread
from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.Qt.QtWidgets import *
from proto.import_all_protos import *

import software.python_bindings as tbots_cpp

from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.common import common_widgets

import evdev
from evdev import ecodes


class KeyboardMagic:
    def __init__(self, drive_and_dribble_widget):
        self.drive_and_dribble_widget = drive_and_dribble_widget
        self.device = evdev.InputDevice("/dev/input/event3")
        self.thread = Thread(target=self.read_one_loop)
        self.thread.start()

        self.motor_control = MotorControl()
        self.motor_control.direct_velocity_control.velocity.x_component_meters = 0
        self.motor_control.direct_velocity_control.velocity.y_component_meters = 0

    def read_one_loop(self):
        for event in self.device.read_loop():
            #motor_control.direct_velocity_control.velocity.x_component_meters = (
            #    -move_axis_x * self.constants.robot_max_speed_m_per_s * speed_factor
            #)

            #motor_control.direct_velocity_control.velocity.y_component_meters = (
            #    -move_axis_y * self.constants.robot_max_speed_m_per_s * speed_factor
            #)

            if event.code == ecodes.KEY_A and event.value == 1:
                self.motor_control.direct_velocity_control.velocity.x_component_meters = -3.0
            elif event.code == ecodes.KEY_A and event.value == 0:
                self.motor_control.direct_velocity_control.velocity.x_component_meters =0 

            if event.code == ecodes.KEY_S and event.value == 1:
                self.motor_control.direct_velocity_control.velocity.y_component_meters = -3.0
            elif event.code == ecodes.KEY_S and event.value == 0:
                self.motor_control.direct_velocity_control.velocity.y_component_meters =0 

            if event.code == ecodes.KEY_W and event.value == 1:
                self.motor_control.direct_velocity_control.velocity.y_component_meters = 3.0
            elif event.code == ecodes.KEY_W and event.value == 0:
                self.motor_control.direct_velocity_control.velocity.y_component_meters =0 

            if event.code == ecodes.KEY_D and event.value == 1:
                self.motor_control.direct_velocity_control.velocity.x_component_meters = 3.0
            elif event.code == ecodes.KEY_D and event.value == 0:
                self.motor_control.direct_velocity_control.velocity.x_component_meters =0 

            #if event.code == ecodes.KEY_Q and event.value == 1:
            #    self.motor_control.direct_velocity_control.velocity.x_component_meters = 0
            #    self.motor_control.direct_velocity_control.velocity.y_component_meters = 0
            #elif event.code == ecodes.KEY_Q and event.value == 0:
            #    self.motor_control.direct_velocity_control.velocity.x_component_meters =0 

            self.drive_and_dribble_widget.override_slider_values(self.motor_control)

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

        self.keyboard_magic = KeyboardMagic(self)
        self.constants = tbots_cpp.create2021RobotConstants()

        self.enabled = True

        layout = QVBoxLayout()
        layout.addWidget(self.__setup_direct_velocity_widgets())
        layout.addWidget(self.__setup_dribbler_widgets())
        self.setLayout(layout)

    def enable(self) -> None:
        """Enable all sliders and buttons in the DriveAndDribblerWidget"""
        if self.enabled:
            return

        self.enabled = True

        common_widgets.enable_slider(self.x_velocity_slider)
        common_widgets.enable_slider(self.y_velocity_slider)
        common_widgets.enable_slider(self.angular_velocity_slider)
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
        self.dribbler_speed_rpm_slider.setValue(motor_control.dribbler_speed_rpm)

    def refresh(self) -> None:
        """Send out a MotorControl proto with the currently set direct velocity and
        dribbler speed values
        """
        motor_control = MotorControl()
        motor_control.direct_velocity_control.velocity.x_component_meters = (
            self.x_velocity_slider.value()
        )
        motor_control.direct_velocity_control.velocity.y_component_meters = (
            self.y_velocity_slider.value()
        )
        motor_control.direct_velocity_control.angular_velocity.radians_per_second = (
            self.angular_velocity_slider.value()
        )
        motor_control.dribbler_speed_rpm = int(self.dribbler_speed_rpm_slider.value())

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

        group_box = QGroupBox("Drive")
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

    def __reset_direct_sliders(self) -> None:
        """Reset the direct velocity sliders back to 0"""
        self.x_velocity_slider.setValue(0)
        self.y_velocity_slider.setValue(0)
        self.angular_velocity_slider.setValue(0)

    def __reset_dribbler_slider(self) -> None:
        """Reset the dribbler speed slider back to 0"""
        self.dribbler_speed_rpm_slider.setValue(0)

    def __reset_all_sliders(self) -> None:
        """Reset all sliders back to 0"""
        self.__reset_direct_sliders()
        self.__reset_dribbler_slider()
