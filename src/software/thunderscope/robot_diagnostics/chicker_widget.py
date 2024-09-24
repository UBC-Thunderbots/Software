from pyqtgraph.Qt.QtCore import *
from pyqtgraph.Qt.QtWidgets import *
from software.py_constants import *
from proto.import_all_protos import *
from enum import Enum
import software.thunderscope.common.common_widgets as common_widgets
from software.thunderscope.constants import DiagnosticsConstants
from software.thunderscope.robot_diagnostics.diagnostics_input_widget import ControlMode
from software.thunderscope.proto_unix_io import ProtoUnixIO


class ChickerCommandMode(Enum):
    KICK = 1
    CHIP = 2
    AUTOKICK = 3
    AUTOCHIP = 4


class ChickerWidget(QWidget):
    def __init__(self, proto_unix_io: ProtoUnixIO) -> None:
        """Handles the robot diagnostics input to create a PowerControl message
        to be sent to the robots.

        NOTE: The powerboards run in regulation mode, which means that they are
        always charged and do not need to be explicitly charged.

        The powerboard also has an internal cooldown, so spamming kick or chip
        will not work until the capacitors charge up and the cooldown is over.

        :param proto_unix_io: proto_unix_io object to send messages to the robot
        """
        super().__init__()

        self.proto_unix_io: ProtoUnixIO = proto_unix_io

        self.__initialize_default_power_control_values()

        chicker_widget_vbox_layout = QVBoxLayout()
        self.setLayout(chicker_widget_vbox_layout)

        (
            self.kick_power_slider_layout,
            self.kick_power_slider,
            self.kick_power_label,
        ) = common_widgets.create_slider(
            "Power (m/s)",
            DiagnosticsConstants.MIN_KICK_POWER,
            DiagnosticsConstants.MAX_KICK_POWER,
            DiagnosticsConstants.KICK_POWER_STEPPER,
        )

        (
            self.chip_distance_slider_layout,
            self.chip_distance_slider,
            self.chip_distance_label,
        ) = common_widgets.create_float_slider(
            "Chip Distance (m)",
            1,
            DiagnosticsConstants.MIN_CHIP_POWER,
            DiagnosticsConstants.MAX_CHIP_POWER,
            int(DiagnosticsConstants.CHIP_DISTANCE_STEPPER),
        )

        self.kick_power_slider.valueChanged.connect(
            lambda: self.kick_power_label.setText(str(self.kick_power_slider.value()))
        )

        self.chip_distance_slider.valueChanged.connect(
            lambda: self.chip_distance_label.setText(str(self.chip_distance_slider.value()))
        )

        kick_chip_sliders_hbox_layout = QHBoxLayout()
        kick_chip_sliders_hbox_layout.addLayout(self.kick_power_slider_layout)
        kick_chip_sliders_hbox_layout.addLayout(self.chip_distance_slider_layout)

        kick_chip_sliders_box = QGroupBox()
        kick_chip_sliders_box.setLayout(kick_chip_sliders_hbox_layout)
        kick_chip_sliders_box.setTitle("Kick Power and Chip Distance")

        chicker_widget_vbox_layout.addWidget(kick_chip_sliders_box)

        # Initializing kick & chip buttons
        (
            self.kick_chip_buttons_box,
            self.kick_chip_buttons,
        ) = common_widgets.create_buttons(["Kick", "Chip"])

        self.kick_chip_buttons_box.setTitle("Single Kick and Chip")

        self.kick_button = self.kick_chip_buttons[0]
        self.chip_button = self.kick_chip_buttons[1]

        chicker_widget_vbox_layout.addWidget(self.kick_chip_buttons_box)

        # Initializing auto kick & chip buttons
        self.radio_buttons_group = QButtonGroup()
        (
            self.auto_kick_chip_buttons_box,
            self.auto_kick_chip_buttons,
        ) = common_widgets.create_radio(
            ["No Auto", "Auto Kick", "Auto Chip"], self.radio_buttons_group
        )
        self.auto_kick_chip_buttons_box.setTitle("Auto Kick and Chip")

        self.no_auto_button = self.auto_kick_chip_buttons[0]
        self.auto_kick_button = self.auto_kick_chip_buttons[1]
        self.auto_chip_button = self.auto_kick_chip_buttons[2]

        # Set no auto button to be selected by default on launch
        self.no_auto_button.setChecked(True)

        self.no_auto_button.toggled.connect(
            self.update_kick_chip_buttons_accessibility
        )

        self.kick_button.clicked.connect(
            lambda: self.send_command_and_timeout(ChickerCommandMode.KICK)
        )
        self.chip_button.clicked.connect(
            lambda: self.send_command_and_timeout(ChickerCommandMode.CHIP)
        )

        chicker_widget_vbox_layout.addWidget(self.auto_kick_chip_buttons_box)

    def set_kick_power_slider_value(self, value: float) -> None:
        """Set the kick power slider to a specific value.

        :param value: The value to set for the slider
        """
        self.kick_power_slider.setValue(value)

    def set_chip_distance_slider_value(self, value: float) -> None:
        """Set the chip distance slider to a specific value.

        :param value: the value to set for the slider
        """
        self.chip_distance_slider.setValue(value)

    def send_command_and_timeout(self, command: ChickerCommandMode) -> None:
        """If the kick/chip buttons are enabled, sends a kick/chip command and disables the buttons.
        The buttons will be re-enabled after CHICKER_TIMEOUT.

        :param command: Command to send. One of ChickerCommandMode.KICK or ChickerCommandMode.CHIP
        """
        if self.kick_button.isEnabled() and self.chip_button.isEnabled():
            common_widgets.disable_button(self.kick_button)
            common_widgets.disable_button(self.chip_button)

            self.send_command(command)

            # set and start timer to re-enable buttons after CHICKER_TIMEOUT
            QTimer.singleShot(int(CHICKER_TIMEOUT), self.update_kick_chip_buttons_accessibility)

    def update_kick_chip_buttons_accessibility(self) -> None:
        """Enables or disables the kick/chip buttons depending on whether autokick/autochip is on"""
        if self.no_auto_button.isChecked():
            common_widgets.enable_button(self.kick_button)
            common_widgets.enable_button(self.chip_button)
        else:
            common_widgets.disable_button(self.kick_button)
            common_widgets.disable_button(self.chip_button)

    def send_command(self, command: ChickerCommandMode) -> None:
        """Sends a [auto]kick or [auto]chip primitive

        :param command: enum int value to indicate what primitive to send
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
        """Sends an empty proto to the proto unix io buffer
        This is due to a bug in robot_communication where if a new PowerControl message is not sent,
        The previous, cached message is resent to the robot repeatedly which we don't want for kick/chip
        So sending an empty message overwrites the cache and prevents spamming commands
        If buffer is full, blocks execution until buffer has space
        """
        self.power_control = PowerControl()
        self.power_control.geneva_slot = 3
        self.proto_unix_io.send_proto(PowerControl, self.power_control, True)

    def update_widget_accessibility(self, mode: ControlMode):
        """Disables or enables all sliders and buttons depending on the given control mode.
        Sliders and buttons are enabled in DIAGNOSTICS mode, and disabled in HANDHELD mode

        :param mode: the current control mode
        """
        if mode == ControlMode.DIAGNOSTICS:
            common_widgets.enable_slider(self.kick_power_slider)
            common_widgets.enable_slider(self.chip_distance_slider)
            common_widgets.enable_button(self.no_auto_button)
            common_widgets.enable_button(self.auto_kick_button)
            common_widgets.enable_button(self.auto_chip_button)
            self.update_kick_chip_buttons_accessibility()
        elif mode == ControlMode.HANDHELD:
            common_widgets.disable_slider(self.kick_power_slider)
            common_widgets.disable_slider(self.chip_distance_slider)
            common_widgets.disable_button(self.no_auto_button)
            common_widgets.disable_button(self.auto_kick_button)
            common_widgets.disable_button(self.auto_chip_button)
            common_widgets.disable_button(self.kick_button)
            common_widgets.disable_button(self.chip_button)

    def refresh(self) -> None:
        """Update the currently persisted PowerControl proto based on the widget's
        slider values and sends out autokick/autochip commands if they are enabled
        """
        # If auto is enabled, we want to populate the autochip or kick message
        if self.auto_kick_button.isChecked():
            self.send_command(ChickerCommandMode.AUTOKICK)
        elif self.auto_chip_button.isChecked():
            self.send_command(ChickerCommandMode.AUTOCHIP)
