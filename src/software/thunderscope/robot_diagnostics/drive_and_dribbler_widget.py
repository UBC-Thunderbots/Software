from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.Qt.QtWidgets import *

from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.common import common_widgets

MIN_MOTOR_RPM = -100
MAX_MOTOR_RPM = 100

MAX_DRIBBLER_RPM = 100
MIN_DRIBBLER_RPM = -100

MAX_LINEAR_SPEED_MPS = 5
MIN_LINEAR_SPEED_MPS = -5

MAX_ANGULAR_SPEED_RPM = 100
MIN_ANGULAR_SPEED_RPM = -100


class DriveAndDribblerWidget(QWidget):
    def __init__(self):
        """Initialize the widget to control the robot's motors
        """
        QWidget.__init__(self)
        layout = QVBoxLayout()

        # Initialize tab screen
        tabs = QTabWidget()
        per_wheel_tab = QWidget()
        direct_velocity_tab = QWidget()

        # Add tabs
        tabs.addTab(per_wheel_tab, "Direct Per-Wheel Control")
        tabs.addTab(direct_velocity_tab, "Direct Velocity Control")

        # Create first tab
        per_wheel_tab.grid = QVBoxLayout()
        per_wheel_tab.grid.addWidget(self.setup_direct_per_wheel("Drive"))
        per_wheel_tab.grid.addStretch(1)
        per_wheel_tab.setLayout(per_wheel_tab.grid)

        # Create second tab
        direct_velocity_tab.grid2 = QVBoxLayout()
        direct_velocity_tab.grid2.addWidget(self.setup_direct_velocity("Drive"))
        direct_velocity_tab.grid2.addStretch(1)
        direct_velocity_tab.setLayout(direct_velocity_tab.grid2)

        # Add tabs to widget
        layout.addWidget(tabs)
        layout.addWidget(self.setup_dribbler("Dribbler"))
        tabs.currentChanged.connect(self.reset_all_sliders)
        self.setLayout(layout)

    def value_change(self, slider):
        """Change the slider's value by 0.1 per step

        :param title: the name of the slider

        """
        value = slider.value()
        value = float(value)
        value = value / 100.0
        value_str = "%.1f" % value
        return value_str

    def setup_direct_per_wheel(self, title):
        """Create a widget to change the RPM per wheel

        :param title: the name of the slider

        """
        groupBox = QGroupBox(title)
        dbox = QVBoxLayout()

        # set up the sliders
        (
            front_left_groupbox,
            self.front_left_slider,
            self.front_left_label,
        ) = common_widgets.create_slider("Front Left", MIN_MOTOR_RPM, MAX_MOTOR_RPM, 1)
        (
            front_right_groupbox,
            self.front_right_slider,
            self.front_right_label,
        ) = common_widgets.create_slider("Front Right", MIN_MOTOR_RPM, MAX_MOTOR_RPM, 1)
        (
            back_left_groupbox,
            self.back_left_slider,
            self.back_left_label,
        ) = common_widgets.create_slider("Back Left", MIN_MOTOR_RPM, MAX_MOTOR_RPM, 1)
        (
            back_right_groupbox,
            self.back_right_slider,
            self.back_right_label,
        ) = common_widgets.create_slider("Back Right", MIN_MOTOR_RPM, MAX_MOTOR_RPM, 1)

        self.front_left_slider.valueChanged.connect(
            lambda: self.front_left_label.setText(
                self.value_change(self.front_left_slider)
            )
        )
        self.front_right_slider.valueChanged.connect(
            lambda: self.front_right_label.setText(
                self.value_change(self.front_right_slider)
            )
        )
        self.back_left_slider.valueChanged.connect(
            lambda: self.back_left_label.setText(
                self.value_change(self.back_left_slider)
            )
        )
        self.back_right_slider.valueChanged.connect(
            lambda: self.back_right_label.setText(
                self.value_change(self.back_right_slider)
            )
        )

        # set up the stop and reset button
        stop_and_reset = common_widgets.create_push_button("Stop and Reset")
        stop_and_reset.clicked.connect(self.reset_all_sliders)

        # add widget
        dbox.addWidget(front_left_groupbox)
        dbox.addWidget(front_right_groupbox)
        dbox.addWidget(back_left_groupbox)
        dbox.addWidget(back_right_groupbox)
        dbox.addWidget(stop_and_reset, alignment=Qt.AlignmentFlag.AlignCenter)

        groupBox.setLayout(dbox)

        return groupBox

    def setup_direct_velocity(self, title):
        """Create a widget to control the direct velocity of the robot's motors

        :param title: the name of the slider

        """

        groupBox = QGroupBox(title)
        dbox = QVBoxLayout()

        xms, self.slider_xms, self.label_xms = common_widgets.create_slider(
            "X (m/s)", MIN_LINEAR_SPEED_MPS, MAX_LINEAR_SPEED_MPS, 10
        )
        yms, self.slider_yms, self.label_ymx = common_widgets.create_slider(
            "Y (m/s)", MIN_LINEAR_SPEED_MPS, MAX_LINEAR_SPEED_MPS, 10
        )
        degree, self.slider_rpm, self.label_rpm = common_widgets.create_slider(
            "θ (°/s)", MIN_ANGULAR_SPEED_RPM, MAX_ANGULAR_SPEED_RPM, 1
        )

        self.slider_xms.valueChanged.connect(
            lambda: self.label_xms.setText(self.value_change(self.slider_xms))
        )
        self.slider_yms.valueChanged.connect(
            lambda: self.label_yms.setText(self.value_change(self.slider_yms))
        )
        self.slider_rpm.valueChanged.connect(
            lambda: self.label_rpm.setText(self.value_change(self.slider_rpm))
        )

        stop_and_reset = common_widgets.create_push_button("Stop and Reset")
        stop_and_reset.clicked.connect(self.reset_all_sliders)

        dbox.addWidget(xms)
        dbox.addWidget(yms)
        dbox.addWidget(degree)
        dbox.addWidget(stop_and_reset, alignment=Qt.AlignmentFlag.AlignCenter)

        groupBox.setLayout(dbox)

        return groupBox

    def setup_dribbler(self, title):
        """Create a widget to control the dribbler RPM

        :param title: the name of the slider

        """

        groupBox = QGroupBox(title)
        dbox = QVBoxLayout()

        (
            dribbler,
            self.slider_dribbler,
            self.label_dribbler,
        ) = common_widgets.create_slider("RPM", MIN_DRIBBLER_RPM, MAX_DRIBBLER_RPM, 1)
        self.slider_dribbler.valueChanged.connect(
            lambda: self.label_dribbler.setText(self.value_change(self.slider_dribbler))
        )

        stop_and_reset = common_widgets.create_push_button("Stop and Reset")
        stop_and_reset.clicked.connect(lambda: self.sliderDribbler.setValue(0))

        dbox.addWidget(dribbler)
        dbox.addWidget(stop_and_reset, alignment=Qt.AlignmentFlag.AlignCenter)
        groupBox.setLayout(dbox)

        return groupBox

    def reset_all_sliders(self):
        """Reset all sliders back to 0
        """

        self.front_left_slider.setValue(0)
        self.front_right_slider.setValue(0)
        self.back_left_slider.setValue(0)
        self.back_right_slider.setValue(0)

        self.slider_xms.setValue(0)
        self.slider_yms.setValue(0)
        self.slider_rpm.setValue(0)
