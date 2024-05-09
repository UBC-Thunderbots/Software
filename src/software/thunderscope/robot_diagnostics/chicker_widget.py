import pyqtgraph as pg
from pyqtgraph.Qt.QtCore import *
from pyqtgraph.Qt.QtWidgets import *
from software.py_constants import *
from proto.import_all_protos import *
from enum import Enum
import software.thunderscope.common.common_widgets as common_widgets
from software.thunderscope.robot_diagnostics.diagnostics_input_widget import ControlMode
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.proto_unix_io import ProtoUnixIO


class ChickerCommandMode(Enum):
    KICK = 1
    CHIP = 2
    AUTOKICK = 3
    AUTOCHIP = 4


class ChickerWidget(QWidget):
    def __init__(
            self,
            proto_unix_io
    ) -> None:
        """Handles the robot diagnostics input to create a PowerControl message
        to be sent to the robots.

        NOTE: The powerboards run in regulation mode, which means that they are
        always charged and do not need to be explicitly charged.

        The powerboard also has an internal cooldown, so spamming kick or chip
        will not work until the capacitors charge up and the cooldown is over.

        :param proto_unix_io: proto_unix_io object to send messages to the robot

        """

        super(ChickerWidget, self).__init__()

        self.proto_unix_io = proto_unix_io

        # initial values
        self.__initialize_default_power_control_values()

        vbox_layout = QVBoxLayout()
        self.setLayout(vbox_layout)

        # Initializing power slider for kicking
        (
            self.kick_power_slider_layout,
            self.kick_power_slider,
            self.kick_power_label,
        ) = common_widgets.create_slider(
            "Power (m/s)", 1, 10, 1
        )
        vbox_layout.addLayout(self.kick_power_slider_layout)

        # Initializing distance slider for chipping
        (
            self.chip_distance_slider_layout,
            self.chip_distance_slider,
            self.chip_distance_label,
        ) = common_widgets.create_slider(
            "Chip Distance (m)", 1, 10, 1
        )
        vbox_layout.addLayout(self.kick_power_slider_layout)

        # Initializing kick & chip buttons
        self.push_button_box, self.push_buttons = common_widgets.create_buttons(
            ["Kick", "Chip"]
        )
        self.kick_button = self.push_buttons[0]
        self.chip_button = self.push_buttons[1]

        # set buttons to be initially enabled
        self.kick_chip_buttons_enable = True

        vbox_layout.addWidget(self.push_button_box)

        # Initializing auto kick & chip buttons
        self.button_clickable_map = {
            "no_auto": True,
            "auto_kick": True,
            "auto_chip": True,
        }
        self.radio_buttons_group = QButtonGroup()
        self.radio_button_box, self.radio_buttons = common_widgets.create_radio(
            ["No Auto", "Auto Kick", "Auto Chip"], self.radio_buttons_group
        )
        self.no_auto_button = self.radio_buttons[0]
        self.auto_kick_button = self.radio_buttons[1]
        self.auto_chip_button = self.radio_buttons[2]

        # Set no auto button to be selected by default on launch
        self.no_auto_button.setChecked(True)
        self.no_auto_selected = True

        # Initialize on-click handlers for kick & chip buttons.
        self.kick_button.clicked.connect(
            lambda: self.send_command_and_timeout(ChickerCommandMode.KICK)
        )
        self.chip_button.clicked.connect(
            lambda: self.send_command_and_timeout(ChickerCommandMode.CHIP)
        )

        # Initialize on-click handlers for no auto, auto kick and auto chip buttons.
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

    def send_command_and_timeout(self, command: ChickerCommandMode) -> None:
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
            QTimer.singleShot(CHICKER_TIMEOUT, self.enable_kick_chip_buttons)

    def disable_kick_chip_buttons(self) -> None:
        """
        Disables the buttons
        """
        self.kick_chip_buttons_enable = False

    def enable_kick_chip_buttons(self) -> None:
        """
        If buttons should be enabled, enables them
        """
        if self.no_auto_selected:
            self.kick_chip_buttons_enable = True

            # clears the proto buffer when buttons are re-enabled
            # just to start fresh and clear any unwanted protos
            self.__initialize_default_power_control_values()

    def set_should_enable_buttons(self, enable: bool) -> None:
        """
        Changes if buttons are clickable or not based on boolean parameter

        :param enable: boolean to indicate whether buttons should be made clickable or not
        """
        self.no_auto_selected = enable

        if enable:
            self.enable_kick_chip_buttons()
        else:
            self.disable_kick_chip_buttons()

    def send_command(self, command: ChickerCommandMode) -> None:
        """Sends a [auto]kick or [auto]chip primitive

        :param command: enum int value to indicate what primitive to send
        :returns: None

        """

        # gets slider values
        kick_power_value = self.kick_power_slider.value()
        chip_distance_value = self.chip_distance_slider.value()

        # sends kick, chip, autokick, or autochip primitive
        if command == ChickerCommandMode.KICK:
            self.power_control.chicker.kick_speed_m_per_s = kick_power_value
        elif command == ChickerCommandMode.CHIP:
            self.power_control.chicker.chip_distance_meters = chip_distance_value
        elif command == ChickerCommandMode.AUTOKICK:
            self.power_control.chicker.auto_chip_or_kick.autokick_speed_m_per_s = (
                kick_power_value
            )
        elif command == ChickerCommandMode.AUTOCHIP:
            self.power_control.chicker.auto_chip_or_kick.autochip_distance_meters = (
                chip_distance_value
            )

        self.proto_unix_io.send_proto(PowerControl, self.power_control)

        # clears the proto buffer for kick or chip commands
        # so only one kick / chip is sent
        if command == ChickerCommandMode.KICK or command == ChickerCommandMode.CHIP:
            self.__initialize_default_power_control_values()

    def __initialize_default_power_control_values(self) -> None:
        """
        Sends an empty proto to the proto unix io buffer
        This is due to a bug in robot_communication where if a new PowerControl message is not sent,
        The previous, cached message is resent to the robot repeatedly which we don't want for kick/chip
        So sending an empty message overwrites the cache and prevents spamming commands
        If buffer is full, blocks execution until buffer has space
        """
        self.power_control = PowerControl()
        self.power_control.geneva_slot = 3

    def refresh_button_state(self, button: QPushButton) -> None:
        """Change button color and clickable state.

        :param button: button to change the state of
        :returns: None

        """
        if self.kick_chip_buttons_enable:
            button.setStyleSheet("background-color: White")
            button.setEnabled(True)
        else:
            button.setStyleSheet("background-color: Grey")
            button.setEnabled(False)

    def update_widget_accessibility(self, mode: ControlMode):
        self.auto_kick_button.setEnabled(mode == ControlMode.DIAGNOSTICS)
        self.auto_chip_button.setEnabled(mode == ControlMode.DIAGNOSTICS)
        self.set_should_enable_buttons(mode == ControlMode.DIAGNOSTICS)

    def refresh(self, mode: ControlMode) -> None:

        # Update this widgets accessibility to the user based on the ControlMode parameter
        self.update_widget_accessibility(mode)

        # get kick power value slider value and set the label to that value
        kick_power_value = self.kick_power_slider.value()
        self.kick_power_label.setText(str(kick_power_value))

        # get chip distance value slider value and set the label to that value
        chip_distance_value = self.chip_distance_slider.value()
        self.chip_distance_label.setText(str(chip_distance_value))

        # refresh button state to reflect to user current status
        self.refresh_button_state(self.kick_button)
        self.refresh_button_state(self.chip_button)

        # If auto is enabled, we want to populate the autochip or kick message
        if self.auto_kick_button.isChecked():
            self.send_command(ChickerCommandMode.AUTOKICK)
        elif self.auto_chip_button.isChecked():
            self.send_command(ChickerCommandMode.AUTOCHIP)
