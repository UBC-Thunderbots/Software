import pyqtgraph as pg
from pyqtgraph.Qt.QtCore import *
from pyqtgraph.Qt.QtWidgets import *
from software.py_constants import *
from proto.import_all_protos import *
from enum import Enum
import software.thunderscope.common.common_widgets as common_widgets
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class ChickerCommandMode(Enum):
    KICK = 1
    CHIP = 2
    AUTOKICK = 3
    AUTOCHIP = 4


class ChickerWidget(QWidget):
    def __init__(self, proto_unix_io):
        """Handles the robot diagnostics input to create a PowerControl message
        to be sent to the robots.

        NOTE: The powerboards run in regulation mode, which means that they are
        always charged and do not need to be explicitly charged. 

        The powerboard also has an internal cooldown, so spamming kick or chip
        will not work until the capacitors charge up and the cooldown is over.

        :param proto_unix_io: proto_unix_io object to send messages to the robot

        """

        super(ChickerWidget, self).__init__()

        vbox_layout = QVBoxLayout()
        self.radio_buttons_group = QButtonGroup()
        self.proto_unix_io = proto_unix_io

        # push button group box
        self.push_button_box, self.push_buttons = common_widgets.create_button(
            ["Kick", "Chip"]
        )
        self.kick_button = self.push_buttons[0]
        self.chip_button = self.push_buttons[1]

        vbox_layout.addWidget(self.push_button_box)

        # radio button group box
        self.radio_button_box, self.radio_buttons = common_widgets.create_radio(
            ["No Auto", "Auto Kick", "Auto Chip"], self.radio_buttons_group
        )
        self.no_auto_button = self.radio_buttons[0]
        self.auto_kick_button = self.radio_buttons[1]
        self.auto_chip_button = self.radio_buttons[2]

        # adding onclick functions for buttons

        # kick button and chip button connected to respective functions
        self.kick_button.clicked.connect(self.kick_clicked)
        self.chip_button.clicked.connect(self.chip_clicked)

        # no auto button enables both buttons, while auto kick and auto chip disable both buttons
        self.no_auto_button.toggled.connect(lambda: self.toggle_should_enable_buttons(True))
        self.auto_kick_button.toggled.connect(lambda: self.toggle_should_enable_buttons(False))
        self.auto_chip_button.toggled.connect(lambda: self.toggle_should_enable_buttons(False))

        vbox_layout.addWidget(self.radio_button_box)
        self.no_auto_button.setChecked(True)

        # set buttons to be initially enabled
        self.buttons_enable = True
        self.buttons_should_enable = True

        # sliders
        (
            self.geneva_slider_layout,
            self.geneva_slider,
            self.geneva_label,
        ) = common_widgets.create_slider("Geneva Position", 1, NUM_GENEVA_ANGLES, 1)
        vbox_layout.addLayout(self.geneva_slider_layout)

        (
            self.power_slider_layout,
            self.power_slider,
            self.power_label,
        ) = common_widgets.create_slider("Power", 1, 10, 1)
        vbox_layout.addLayout(self.power_slider_layout)

        self.setLayout(vbox_layout)

        # to manage the state of radio buttons - to make sure message is only sent once
        self.radio_checkable = {"no_auto": True, "auto_kick": True, "auto_chip": True}

        # initial values
        self.geneva_value = 3
        self.power_value = 1

    def kick_clicked(self):
        """
        If buttons are enabled, sends a Kick command and disables buttons

        Attaches a callback to re-enable buttons after 3 seconds
        """
        # if button is enabled
        if self.buttons_enable:
            # send kick primitive
            self.send_command(ChickerCommandMode.KICK)
            self.disable_kick_chip_button()

            # set and start timer to re-enable kick button after 3 seconds
            self.start_timer_once(self.enable_kick_chip_button, 3 * MILLISECONDS_PER_SECOND)

    def chip_clicked(self):
        """
        If buttons are enabled, sends a Chip command and disables buttons

        Attaches a callback to re-enable buttons after 3 seconds
        """

        # if button is enabled
        if self.buttons_enable:
            # send chip primitive
            self.send_command(ChickerCommandMode.CHIP)
            self.disable_kick_chip_button()

            # set and start timer to re-enable chip button after 3 seconds
            self.start_timer_once(self.enable_kick_chip_button, 3 * MILLISECONDS_PER_SECOND)

    def start_timer_once(self, function, duration):
        """Starts a QTimer to call the given function once after the given duration

        :param function: function to call after duration
        :param duration: duration to call function after
        :returns: None

        """

        timer = QTimer(self)
        timer.setTimerType(Qt.TimerType.PreciseTimer)
        timer.timeout.connect(function)
        timer.setSingleShot(True)
        timer.start(duration)

    def disable_kick_chip_button(self):
        """
        If buttons are enabled, disables them
        """
        if self.buttons_enable:
            self.buttons_enable = False

    def enable_kick_chip_button(self):
        """
        If buttons are disabled and buttons should be enabled, enables them
        """
        if not self.buttons_enable and self.buttons_should_enable:
            self.buttons_enable = True

    def toggle_should_enable_buttons(self, enable):
        """
        Toggles if buttons are clickable or not based on boolean parameter

        :param enable: boolean to indicate whether buttons should be made clickable or not
        """
        self.buttons_should_enable = enable

        if enable:
            self.enable_kick_chip_button()
        else:
            self.disable_kick_chip_button()

    def send_command(self, command):
        """Sends a [auto]kick or [auto]chip primitive

        :param command: enum int value to indicate what primitive to send
        :returns: None

        """

        # gets slider values
        geneva_value = self.geneva_slider.value()

        power_value = self.power_slider.value()

        power_control = PowerControl()
        power_control.geneva_slot = geneva_value

        # sends kick, chip, autokick, or autchip primitive
        if command == ChickerCommandMode.KICK:
            power_control.chicker.kick_speed_m_per_s = power_value
        elif command == ChickerCommandMode.CHIP:
            power_control.chicker.chip_distance_meters = power_value
        elif command == ChickerCommandMode.AUTOKICK:
            power_control.chicker.auto_chip_or_kick.autokick_speed_m_per_s = power_value
        elif command == ChickerCommandMode.AUTOCHIP:
            power_control.chicker.auto_chip_or_kick.autochip_distance_meters = (
                power_value
            )

        # sends proto
        self.proto_unix_io.send_proto(PowerControl, power_control)

    def change_button_state(self, button, enable):
        """Change button color and clickable state.

        :param button: button to change the state of
        :param enable: bool: if True: enable this button, if False: disable
        :returns: None

        """
        if enable:
            button.setStyleSheet("background-color: White")
            button.setCheckable(True)
        else:
            button.setStyleSheet("background-color: Grey")
            button.setCheckable(False)

    def refresh(self):

        # gets slider values and sets label to that value
        geneva_value = self.geneva_slider.value()
        self.geneva_label.setText(Slot.Name(geneva_value))

        power_value = self.power_slider.value()
        self.power_label.setText(str(power_value))

        # refreshes button state based on enable boolean
        self.change_button_state(self.kick_button, self.buttons_enable)
        self.change_button_state(self.chip_button, self.buttons_enable)

        # If auto is enabled, we want to populate the autochip or kick message
        if self.auto_kick_button.isChecked():
            self.send_command(ChickerCommandMode.AUTOKICK)
            self.buttons_should_enable = False
        elif self.auto_chip_button.isChecked():
            self.send_command(ChickerCommandMode.AUTOCHIP)
            self.buttons_should_enable = False
        elif self.no_auto_button.isChecked():
            self.buttons_should_enable = True
            pass
