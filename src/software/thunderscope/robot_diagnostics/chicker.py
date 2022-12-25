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

        # Initialising the buttons

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

        # set buttons to be initially enabled
        self.kick_chip_buttons_enable = True

        # indicating that no auto button is selected by default
        self.no_auto_button.setChecked(True)
        self.no_auto_selected = True

        # adding onclick functions for buttons

        # kick button and chip button connected to send_command_and_timeout with their respective commands
        self.kick_button.clicked.connect(
            lambda: self.send_command_and_timeout(ChickerCommandMode.KICK)
        )
        self.chip_button.clicked.connect(
            lambda: self.send_command_and_timeout(ChickerCommandMode.CHIP)
        )

        # no auto button enables both buttons, while auto kick and auto chip disable both buttons
        self.no_auto_button.toggled.connect(
            lambda: self.set_should_enable_buttons(True)
        )
        self.auto_kick_button.toggled.connect(
            lambda: self.set_should_enable_buttons(False)
        )
        self.auto_chip_button.toggled.connect(
            lambda: self.set_should_enable_buttons(False)
        )

        vbox_layout.addWidget(self.radio_button_box)

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

    def send_command_and_timeout(self, command):
        """
        If buttons are enabled, sends a Kick command and disables buttons

        Attaches a callback to re-enable buttons after 3 seconds

        :param command: Command to send. One of ChickerCommandMode.KICK or ChickerCommandMode.CHIP
        """
        # if button is enabled
        if self.kick_chip_buttons_enable:
            # send kick primitive
            self.send_command(command)
            self.disable_kick_chip_buttons()

            # set and start timer to re-enable buttons after 3 seconds
            QTimer.singleShot(
                3 * MILLISECONDS_PER_SECOND, self.enable_kick_chip_buttons
            )

    def disable_kick_chip_buttons(self):
        """
        Disables the buttons
        """
        self.kick_chip_buttons_enable = False

    def enable_kick_chip_buttons(self):
        """
        If buttons should be enabled, enables them
        """
        if self.no_auto_selected:
            self.kick_chip_buttons_enable = True

    def set_should_enable_buttons(self, enable):
        """
        Changes if buttons are clickable or not based on boolean parameter

        :param enable: boolean to indicate whether buttons should be made clickable or not
        """
        self.no_auto_selected = enable

        if enable:
            self.enable_kick_chip_buttons()
        else:
            self.disable_kick_chip_buttons()

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

        # send empty proto
        # this is due to a bug in robot_communication where if a new PowerControl message is not sent,
        # the previous, cached message is resent to the robot repeatedly
        # so sending an empty message overwrites the cache and prevents spamming commands
        # if buffer is full, blocks execution until buffer has space
        power_control = PowerControl()
        self.proto_unix_io.send_proto(PowerControl, power_control, True)

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
        self.change_button_state(self.kick_button, self.kick_chip_buttons_enable)
        self.change_button_state(self.chip_button, self.kick_chip_buttons_enable)

        # If auto is enabled, we want to populate the autochip or kick message
        if self.auto_kick_button.isChecked():
            self.send_command(ChickerCommandMode.AUTOKICK)
        elif self.auto_chip_button.isChecked():
            self.send_command(ChickerCommandMode.AUTOCHIP)
