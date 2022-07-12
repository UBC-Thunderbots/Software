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
        per_wheel_tab = QWidget()
        direct_velocity_tab = QWidget()

        # Add tabs
        self.tabs.addTab(per_wheel_tab, "Direct Per-Wheel Control")
        self.tabs.addTab(direct_velocity_tab, "Direct Velocity Control")

        # Create per wheel tab
        self.per_wheel_layout = QVBoxLayout()
        self.per_wheel_layout.addWidget(self.setup_direct_per_wheel("Drive"))
        per_wheel_tab.setLayout(self.per_wheel_layout)

        # Create direct velocity tab
        self.direct_velocity_layout = QVBoxLayout()
        self.direct_velocity_layout.addWidget(self.setup_direct_velocity("Drive"))
        direct_velocity_tab.setLayout(self.direct_velocity_layout)

        # Add tabs to widget
        layout.addWidget(self.tabs)
        layout.addWidget(self.setup_dribbler("Dribbler"))
        self.tabs.currentChanged.connect(self.reset_all_sliders)

        self.setLayout(layout)

    def refresh(self):
        """Refresh the widget and send the a MotorControl message with the current values
        """
        motor_control = MotorControl()
        motor_control.dribbler_speed_rpm = int(
            self.dribbler_speed_rpm_slider.value() / 1000.0
        )

        # If we are on the direct per-wheel control tab
        if self.tabs.currentIndex() == 0:
            motor_control.direct_per_wheel_control.front_left_wheel_velocity = (
                self.front_left_velocity_slider.value() / 1000.0
            )
            motor_control.direct_per_wheel_control.front_right_wheel_velocity = (
                self.front_right_velocity_slider.value() / 1000.0
            )
            motor_control.direct_per_wheel_control.back_left_wheel_velocity = (
                self.back_left_velocity_slider.value() / 1000.0
            )
            motor_control.direct_per_wheel_control.back_right_wheel_velocity = (
                self.back_right_velocity_slider.value() / 1000.0
            )

        # If we are on the direct velocity control tab
        elif self.tabs.currentIndex() == 1:
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

    def setup_direct_per_wheel(self, title):
        """Create a widget to change the RPM per wheel

        :param title: the name of the slider

        """
        group_box = QGroupBox(title)
        dbox = QVBoxLayout()

        # set up the sliders
        (
            front_left_layout,
            self.front_left_velocity_slider,
            self.front_left_label,
        ) = common_widgets.create_slider(
            "Front Left", MIN_LINEAR_SPEED_MPS * 1000, MAX_LINEAR_SPEED_MPS * 1000, 1
        )
        (
            front_right_velocity_layout,
            self.front_right_velocity_slider,
            self.front_right_velocity_label,
        ) = common_widgets.create_slider(
            "Front Right", MIN_LINEAR_SPEED_MPS * 1000, MAX_LINEAR_SPEED_MPS * 1000, 1
        )
        (
            back_left_velocity_layout,
            self.back_left_velocity_slider,
            self.back_left_velocity_label,
        ) = common_widgets.create_slider(
            "Back Left", MIN_LINEAR_SPEED_MPS * 1000, MAX_LINEAR_SPEED_MPS * 1000, 1
        )
        (
            back_right_velocity_layout,
            self.back_right_velocity_slider,
            self.back_right_velocity_label,
        ) = common_widgets.create_slider(
            "Back Right", MIN_LINEAR_SPEED_MPS * 1000, MAX_LINEAR_SPEED_MPS * 1000, 1
        )

        self.front_left_velocity_slider.valueChanged.connect(
            lambda: self.front_left_label.setText(
                self.value_change(self.front_left_velocity_slider)
            )
        )
        self.front_right_velocity_slider.valueChanged.connect(
            lambda: self.front_right_velocity_label.setText(
                self.value_change(self.front_right_velocity_slider)
            )
        )
        self.back_left_velocity_slider.valueChanged.connect(
            lambda: self.back_left_velocity_label.setText(
                self.value_change(self.back_left_velocity_slider)
            )
        )
        self.back_right_velocity_slider.valueChanged.connect(
            lambda: self.back_right_velocity_label.setText(
                self.value_change(self.back_right_velocity_slider)
            )
        )

        # set up the stop and reset button
        stop_and_reset = common_widgets.create_push_button("Stop and Reset")
        stop_and_reset.clicked.connect(self.reset_all_sliders)

        # add widget
        dbox.addLayout(front_left_velocity_layout)
        dbox.addLayout(front_right_velocity_layout)
        dbox.addLayout(back_left_velocity_layout)
        dbox.addLayout(back_right_velocity_layout)

        dbox.addWidget(stop_and_reset, alignment=Qt.AlignmentFlag.AlignCenter)

        group_box.setLayout(dbox)

        return group_box

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

        self.x_velocity_slider.valueChanged.connect(
            lambda: self.x_velocity_label.setText(
                self.value_change(self.x_velocity_slider)
            )
        )
        self.y_velocity_slider.valueChanged.connect(
            lambda: self.y_velocity_label.setText(
                self.value_change(self.y_velocity_slider)
            )
        )
        self.angular_velocity_slider.valueChanged.connect(
            lambda: self.angular_velocity_label.setText(
                self.value_change(self.angular_velocity_slider)
            )
        )

        stop_and_reset = common_widgets.create_push_button("Stop and Reset")
        stop_and_reset.clicked.connect(self.reset_all_sliders)

        dbox.addLayout(x_layout)
        dbox.addLayout(y_layout)
        dbox.addLayout(dps_layout)
        dbox.addWidget(stop_and_reset, alignment=Qt.AlignmentFlag.AlignCenter)

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
        self.dribbler_speed_rpm_slider.valueChanged.connect(
            lambda: self.dribbler_speed_rpm_label.setText(
                self.value_change(self.dribbler_speed_rpm_slider)
            )
        )

        stop_and_reset = common_widgets.create_push_button("Stop and Reset")
        stop_and_reset.clicked.connect(
            lambda: self.dribbler_speed_rpm_slider.setValue(0)
        )

        dbox.addLayout(dribbler_layout)
        dbox.addWidget(stop_and_reset, alignment=Qt.AlignmentFlag.AlignCenter)
        group_box.setLayout(dbox)

        return group_box

    def reset_all_sliders(self):
        """Reset all sliders back to 0
        """

        self.front_left_velocity_slider.setValue(0)
        self.front_right_velocity_slider.setValue(0)
        self.back_left_velocity_slider.setValue(0)
        self.back_right_velocity_slider.setValue(0)

        self.x_velocity_slider.setValue(0)
        self.y_velocity_slider.setValue(0)
        self.angular_velocity_slider.setValue(0)
