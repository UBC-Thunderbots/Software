import time
from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.Qt.QtWidgets import *
from proto.import_all_protos import *

import software.python_bindings as tbots_cpp

from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.robot_diagnostics.diagnostics_input_widget import ControlMode
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.common import common_widgets


class DriveAndDribblerWidget(QWidget):
    def __init__(
            self,
            proto_unix_io: ProtoUnixIO
    ) -> None:
        """
        Initialize the widget to control the robot's motors
        """

        super(DriveAndDribblerWidget, self).__init__()

        self.proto_unix_io = proto_unix_io

        self.motor_control = MotorControl()

        self.input_a = time.time()
        self.constants = tbots_cpp.create2021RobotConstants()
        QWidget.__init__(self)
        layout = QVBoxLayout()

        # Add widgets to layout
        layout.addWidget(self.__setup_direct_velocity())
        layout.addWidget(self.__setup_dribbler())

        self.setLayout(layout)

    def set_x_velocity_slider(self, value: float):
        self.x_velocity_slider.setValue(value)

    def set_y_velocity_slider(self, value: float):
        self.y_velocity_slider.setValue(value)

    def set_angular_velocity_slider(self, value: float):
        self.angular_velocity_slider.setValue(value)

    def set_dribbler_velocity_slider(self, value: float):
        self.dribbler_speed_rpm_slider.setValue(value)

    def refresh(self, mode: ControlMode) -> None:
        """
        Refresh the widget's sliders
        Enable or disable this widgets sliders and buttons based on mode value
        Collect motor control values and persist in the primitive field
        """

        # Update this widgets accessibility to the user based on the ControlMode parameter
        self.update_widget_accessibility(mode)

        self.motor_control.dribbler_speed_rpm = int(
            self.dribbler_speed_rpm_slider.value()
        )

        self.motor_control.direct_velocity_control.velocity.x_component_meters = (
            self.x_velocity_slider.value()
        )

        self.motor_control.direct_velocity_control.velocity.y_component_meters = (
            self.y_velocity_slider.value()
        )

        self.motor_control.direct_velocity_control.angular_velocity.radians_per_second = (
            self.angular_velocity_slider.value()
        )

        if mode == ControlMode.DIAGNOSTICS:
            self.proto_unix_io.send_proto(MotorControl, self.motor_control)

    def __value_change_handler(self, value: float) -> str:
        """
        Converts the given float value to a string label
        :param value: float value to be converted
        """
        return "%.2f" % float(value)

    def __setup_direct_velocity(self) -> QGroupBox:
        """
        Create a widget to control the direct velocity of the robot's motors
        """

        group_box = QGroupBox("Drive")
        dbox = QVBoxLayout()

        dbox.setContentsMargins(0, 0, 0, 0)

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
            self.x_velocity_slider,
            self.x_velocity_label,
            self.__value_change_handler
        )
        common_widgets.enable_slider(
            self.y_velocity_slider,
            self.y_velocity_label,
            self.__value_change_handler
        )
        common_widgets.enable_slider(
            self.angular_velocity_slider,
            self.angular_velocity_label,
            self.__value_change_handler,
        )

        self.stop_and_reset_direct = common_widgets.create_push_button("Stop and Reset")
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
        """
        Create a widget to control the dribbler RPM
        """
        group_box = QGroupBox("Dribbler")
        dbox = QVBoxLayout()

        dbox.setContentsMargins(0, 0, 0, 0)

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
            self.__value_change_handler,
        )

        self.stop_and_reset_dribbler = common_widgets.create_push_button(
            "Stop and Reset"
        )
        self.stop_and_reset_dribbler.clicked.connect(self.__reset_dribbler_slider)

        dbox.addLayout(dribbler_layout)
        dbox.addWidget(
            self.stop_and_reset_dribbler, alignment=Qt.AlignmentFlag.AlignCenter
        )
        group_box.setLayout(dbox)

        return group_box

    def update_widget_accessibility(self, mode: ControlMode) -> None:
        """
        Disables or enables all sliders and buttons depending on ControlMode parameter.
        Sliders are enabled in DIAGNOSTICS mode, and disabled in HANDHELD mode
        Updates listener functions and stylesheets accordingly
        :param mode: ControlMode enum parameter
        """
        if mode == ControlMode.DIAGNOSTICS:
            # disconnect all sliders
            self.__disconnect_sliders()

            # enable all sliders by adding listener to update label with slider value
            common_widgets.enable_slider(
                self.x_velocity_slider,
                self.x_velocity_label,
                self.__value_change_handler,
            )
            common_widgets.enable_slider(
                self.y_velocity_slider,
                self.y_velocity_label,
                self.__value_change_handler,
            )
            common_widgets.enable_slider(
                self.angular_velocity_slider,
                self.angular_velocity_label,
                self.__value_change_handler,
            )
            common_widgets.enable_slider(
                self.dribbler_speed_rpm_slider,
                self.dribbler_speed_rpm_label,
                self.__value_change_handler,
            )

            # enable buttons
            common_widgets.enable_button(self.stop_and_reset_dribbler)
            common_widgets.enable_button(self.stop_and_reset_direct)

        elif mode == ControlMode.HANDHELD:
            # reset slider values and disconnect
            self.__reset_all_sliders()
            # self.__disconnect_sliders()

            # disable all sliders by adding listener to keep slider value the same
            # common_widgets.disable_slider(self.x_velocity_slider)
            # common_widgets.disable_slider(self.y_velocity_slider)
            # common_widgets.disable_slider(self.angular_velocity_slider)
            # common_widgets.disable_slider(self.dribbler_speed_rpm_slider)

            # disable buttons
            common_widgets.disable_button(self.stop_and_reset_dribbler)
            common_widgets.disable_button(self.stop_and_reset_direct)

    def __disconnect_sliders(self) -> None:
        """
        Disconnect listener for changing values for all sliders
        """
        self.x_velocity_slider.valueChanged.disconnect()
        self.y_velocity_slider.valueChanged.disconnect()
        self.angular_velocity_slider.valueChanged.disconnect()
        self.dribbler_speed_rpm_slider.valueChanged.disconnect()

    def __reset_direct_sliders(self) -> None:
        """Reset direct sliders back to 0
        """
        self.x_velocity_slider.setValue(0)
        self.y_velocity_slider.setValue(0)
        self.angular_velocity_slider.setValue(0)

    def __reset_dribbler_slider(self) -> None:
        """
        Reset the dribbler slider back to 0
        """
        self.dribbler_speed_rpm_slider.setValue(0)

    def __reset_all_sliders(self) -> None:
        """
        Reset all sliders back to 0
        """
        self.__reset_direct_sliders()
        self.__reset_dribbler_slider()
