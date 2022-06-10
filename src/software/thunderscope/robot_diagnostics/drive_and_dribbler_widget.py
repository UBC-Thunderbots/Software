from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.Qt.QtWidgets import *

from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

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
        tabs.setStyleSheet("background-color: black; color:white")

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

    def create_slider(self, title, min, max, step):
        """Creates a slider for the widget

        :param title: the name of the slider
        :param min: the minimum value of the slider
        :param max: the maximum value of the slider
        :param step: singleStep of the slider

        """
        groupBox = QGroupBox(title)

        # set up the slide
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        slider.setSingleStep(step)
        slider.setMinimum(min)
        slider.setMaximum(max)

        # create a label to display slide's value
        label = QLabel()
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        slider.valueChanged.connect(lambda: label.setText(self.valuechange(slider)))

        # add widgets
        vbox = QGridLayout()
        vbox.addWidget(slider, 0, 0)
        vbox.addWidget(label, 0, 1)
        groupBox.setLayout(vbox)

        return groupBox, slider

    def value_change(self, slider):
        """Change the slider's value by 0.1 per step

        :param title: the name of the slider

        """
        value = slider.value()
        value = float(value)
        value = value / 100.0
        value_str = "%.1f" % value
        return value_str

    def push_button(self, title):
        """Create a push button

        :param title: the name of the button

        """
        push_button = QPushButton(title)
        push_button.setFixedWidth(150)

        return push_button

    def setup_direct_per_wheel(self, title):
        """Create a widget to change the RPM per wheel

        :param title: the name of the slider

        """
        groupBox = QGroupBox(title)
        dbox = QVBoxLayout()

        # set up the sliders
        front_left_groupbox, self.front_left_slider = self.create_slider(
            "Front Left", MIN_MOTOR_RPM, MAX_MOTOR_RPM, 1
        )
        front_right_groupbox, self.front_right_slider = self.create_slider(
            "Front Right", MIN_MOTOR_RPM, MAX_MOTOR_RPM, 1
        )
        back_left_groupbox, self.back_left_slider = self.create_slider(
            "Back Left", MIN_MOTOR_RPM, MAX_MOTOR_RPM, 1
        )
        back_right_groupbox, self.back_right_slider = self.create_slider(
            "Back Right", MIN_MOTOR_RPM, MAX_MOTOR_RPM, 1
        )

        # set up the stop and reset button
        stop_and_reset = self.pushButton("Stop and Reset")
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

        xms, self.slider_xms = self.create_slider(
            "X (m/s)", MIN_LINEAR_SPEED_MPS, MAX_LINEAR_SPEED_MPS, 10
        )
        yms, self.slider_yms = self.create_slider(
            "Y (m/s)", MIN_LINEAR_SPEED_MPS, MAX_LINEAR_SPEED_MPS, 10
        )
        degree, self.slider_rpm = self.create_slider(
            "θ (°/s)", MIN_ANGULAR_SPEED_RPM, MAX_ANGULAR_SPEED_RPM, 1
        )

        stop_and_reset = self.pushButton("Stop and Reset")
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

        dribbler, self.sliderDribbler = self.create_slider(
            "RPM", MIN_DRIBBLER_RPM, MAX_DRIBBLER_RPM, 1
        )

        stop_and_reset = self.pushButton("Stop and Reset")
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
