from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.Qt.QtWidgets import *
import time

from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.common import common_widgets
from proto.import_all_protos import *

# TODO (#2683) get these values from robot constants
MAX_DRIBBLER_RPM = 10000
MIN_DRIBBLER_RPM = -10000

MAX_LINEAR_SPEED_MPS = 5
MIN_LINEAR_SPEED_MPS = -5

MAX_ANGULAR_SPEED_RAD_PER_S = 20
MIN_ANGULAR_SPEED_RAD_PER_S = -20


class DriveAndDribblerWidget(QWidget):
    def __init__(self, proto_unix_io):
        """Initialize the widget to control the robot's motors

        :param proto_unix_io: the proto_unix_io object

        """
        self.input_a = time.time()

        QWidget.__init__(self)
        layout = QVBoxLayout()

        self.proto_unix_io = proto_unix_io

        # Initialize tab screen
        self.tabs = QTabWidget()
        direct_velocity_tab = QWidget()

        self.enabled = False

        # Add tabs
        self.tabs.addTab(direct_velocity_tab, "Direct Velocity Control")

        # Create direct velocity tab
        self.direct_velocity_layout = QVBoxLayout()
        self.direct_velocity_layout.addWidget(self.setup_direct_velocity("Drive"))
        direct_velocity_tab.setLayout(self.direct_velocity_layout)

        # Add tabs to widget
        layout.addWidget(self.tabs)
        layout.addWidget(self.setup_dribbler("Dribbler"))
        self.tabs.currentChanged.connect(self.reset_all_sliders)

        self.enabled = True

        self.setLayout(layout)

    def refresh(self):
        """Refresh the widget and send the a MotorControl message with the current values
        """
        motor_control = MotorControl()
        motor_control.dribbler_speed_rpm = int(
            self.dribbler_speed_rpm_slider.value() / 1000.0
        )

        # If we are on the direct per-wheel control tab
        # If we are on the direct velocity control tab
        if self.tabs.currentIndex() == 0:
            motor_control.direct_velocity_control.velocity.x_component_meters = (
                self.x_velocity_slider.value() / 1000.0
            )
            motor_control.direct_velocity_control.velocity.y_component_meters = (
                self.y_velocity_slider.value() / 1000.0
            )
            motor_control.direct_velocity_control.angular_velocity.radians_per_second = (
                self.angular_velocity_slider.value() / 1000.0
            )

        self.proto_unix_io.send_proto(MotorControl, motor_control)

    def value_change(self, slider):
        """Change the slider's value by 0.1 per step

        :param title: the name of the slider

        """
        value = slider.value()
        value = float(value)
        value = value / 1000.0
        value_str = "%.1f" % value
        return value_str

    def setup_direct_velocity(self, title):
        """Create a widget to control the direct velocity of the robot's motors

        :param title: the name of the slider

        """

        group_box = QGroupBox(title)
        dbox = QVBoxLayout()

        (
            x_layout,
            self.x_velocity_slider,
            self.x_velocity_label,
        ) = common_widgets.create_slider(
            "X (m/s)", MIN_LINEAR_SPEED_MPS * 1000, MAX_LINEAR_SPEED_MPS * 1000, 1
        )
        (
            y_layout,
            self.y_velocity_slider,
            self.y_velocity_label,
        ) = common_widgets.create_slider(
            "Y (m/s)", MIN_LINEAR_SPEED_MPS * 1000, MAX_LINEAR_SPEED_MPS * 1000, 1
        )
        (
            dps_layout,
            self.angular_velocity_slider,
            self.angular_velocity_label,
        ) = common_widgets.create_slider(
            "θ (°/s)",
            MIN_ANGULAR_SPEED_RAD_PER_S * 1000,
            MAX_ANGULAR_SPEED_RAD_PER_S * 1000,
            1,
        )

        # add listener functions for sliders to update label with slider value
        self.enable_slider(self.x_velocity_slider, self.x_velocity_label)
        self.enable_slider(self.y_velocity_slider, self.y_velocity_label)
        self.enable_slider(self.angular_velocity_slider, self.angular_velocity_label)

        self.stop_and_reset_direct = common_widgets.create_push_button("Stop and Reset")
        self.stop_and_reset_direct.clicked.connect(self.reset_all_sliders)

        dbox.addLayout(x_layout)
        dbox.addLayout(y_layout)
        dbox.addLayout(dps_layout)
        dbox.addWidget(self.stop_and_reset_direct, alignment=Qt.AlignmentFlag.AlignCenter)

        group_box.setLayout(dbox)

        return group_box

    def setup_dribbler(self, title):
        """Create a widget to control the dribbler RPM

        :param title: the name of the slider

        """

        group_box = QGroupBox(title)
        dbox = QVBoxLayout()

        (
            dribbler_layout,
            self.dribbler_speed_rpm_slider,
            self.dribbler_speed_rpm_label,
        ) = common_widgets.create_slider(
            "RPM", MIN_DRIBBLER_RPM * 1000, MAX_DRIBBLER_RPM * 1000, 1000
        )

        # add listener function to update label with slider value
        self.enable_slider(self.dribbler_speed_rpm_slider, self.dribbler_speed_rpm_label)

        self.stop_and_reset_dribbler = common_widgets.create_push_button("Stop and Reset")
        self.stop_and_reset_dribbler.clicked.connect(
            lambda: self.dribbler_speed_rpm_slider.setValue(0)
        )

        dbox.addLayout(dribbler_layout)
        dbox.addWidget(self.stop_and_reset_dribbler, alignment=Qt.AlignmentFlag.AlignCenter)
        group_box.setLayout(dbox)

        return group_box

    def toggle_all(self, enable):
        """
        Disables or enables all sliders and buttons depending on boolean parameter

        Updates listener functions and stylesheets accordingly
        :param enable: boolean parameter, True is enable and False is disable
        """
        if enable:
            if not self.enabled:
                # disconnect all sliders
                self.disconnect_sliders()

                # enable all sliders by adding listener to update label with slider value
                self.enable_slider(self.x_velocity_slider, self.x_velocity_label)
                self.enable_slider(self.y_velocity_slider, self.y_velocity_label)
                self.enable_slider(self.angular_velocity_slider, self.angular_velocity_label)
                self.enable_slider(self.dribbler_speed_rpm_slider, self.dribbler_speed_rpm_label)

                # enable buttons
                common_widgets.change_button_state(self.stop_and_reset_dribbler, True)
                common_widgets.change_button_state(self.stop_and_reset_direct, True)

                # change enabled field
                self.enabled = True
        else:
            if self.enabled:
                # reset slider values and disconnect
                self.reset_all_sliders()
                self.disconnect_sliders()

                # disable all sliders by adding listener to keep slider value the same
                self.disable_slider(self.x_velocity_slider)
                self.disable_slider(self.y_velocity_slider)
                self.disable_slider(self.angular_velocity_slider)
                self.disable_slider(self.dribbler_speed_rpm_slider)

                # disable buttons
                common_widgets.change_button_state(self.stop_and_reset_dribbler, False)
                common_widgets.change_button_state(self.stop_and_reset_direct, False)

                # change enabled field
                self.enabled = False

    def disable_slider(self, slider):
        """
        Disables a slider by getting the current value and setting the slider to that
        value upon every value change

        This results in slider value not changing even when slider is moved

        :param slider: slider widget to be disabled
        """
        old_val = slider.value()
        slider.valueChanged.connect(
            lambda: slider.setValue(old_val)
        )

        slider.setStyleSheet("color: Grey")

    def enable_slider(self, slider, label):
        """
        Enables a slider by connecting a function to update label upon value change
        :param slider: slider widget to be enabled
        :param label: label widget corresponding to the slider
        """
        slider.valueChanged.connect(
            lambda: label.setText(
                self.value_change(slider)
            )
        )

    def disconnect_sliders(self):
        """
        Disconnect listener for changing values for all sliders
        """
        self.x_velocity_slider.valueChanged.disconnect()
        self.y_velocity_slider.valueChanged.disconnect()
        self.angular_velocity_slider.valueChanged.disconnect()
        self.dribbler_speed_rpm_slider.valueChanged.disconnect()

    def reset_direct_sliders(self):
        """Reset direct sliders back to 0
        """
        self.x_velocity_slider.setValue(0)
        self.y_velocity_slider.setValue(0)
        self.angular_velocity_slider.setValue(0)

    def reset_all_sliders(self):
        """
        Reset all sliders back to 0
        """
        self.reset_direct_sliders()
        self.dribbler_speed_rpm_slider.setValue(0)
