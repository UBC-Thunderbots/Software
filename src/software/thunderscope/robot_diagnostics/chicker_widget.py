from pyqtgraph.Qt.QtCore import *
from pyqtgraph.Qt.QtWidgets import *
from software.py_constants import *
from proto.import_all_protos import *
from enum import Enum
import software.thunderscope.common.common_widgets as common_widgets
from software.thunderscope.constants import DiagnosticsConstants
from software.thunderscope.proto_unix_io import ProtoUnixIO


class ChickerCommandMode(Enum):
    """The types of chicker commands that we can send to the robots"""

    KICK = 1
    CHIP = 2
    AUTOKICK = 3
    AUTOCHIP = 4
    KICK_PULSE_WIDTH = 5
    CHIP_PULSE_WIDTH = 6
    AUTOKICK_PULSE_WIDTH = 7
    AUTOCHIP_PULSE_WIDTH = 8


class ChickerWidget(QWidget):
    """This widget provides an interface to send PowerControl messages for
    kicking and chipping actions to the robot. It has sliders for adjusting
    the kick power and chip distance, as well as buttons for sending single
    kick/chip commands or turning on autokick/autochip.

    NOTE: The powerboards run in regulation mode, which means that they are
    always charged and do not need to be explicitly charged.

    The powerboard also has an internal cooldown, so spamming kick or chip
    will not work until the capacitors charge up and the cooldown is over.
    """

    def __init__(self, proto_unix_io: ProtoUnixIO) -> None:
        """Initialize the ChickerWidget

        :param proto_unix_io: ProtoUnixIO to send messages to the robot
        """
        super().__init__()

        self.proto_unix_io: ProtoUnixIO = proto_unix_io

        self.enabled = True

        self.kick_chip_locked = False

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

        (
            self.kicker_pulse_width_slider_layout,
            self.kicker_pulse_width_slider,
            self.kicker_pulse_width_label,
        ) = common_widgets.create_slider(
            "Kicker Pulse Width",
            DiagnosticsConstants.MIN_PULSE_WIDTH,
            DiagnosticsConstants.MAX_PULSE_WIDTH,
            DiagnosticsConstants.PULSE_WIDTH_STEPPER,
        )

        (
            self.chipper_pulse_width_slider_layout,
            self.chipper_pulse_width_slider,
            self.chipper_pulse_width_label,
        ) = common_widgets.create_slider(
            "Chipper Pulse Width",
            DiagnosticsConstants.MIN_PULSE_WIDTH,
            DiagnosticsConstants.MAX_PULSE_WIDTH,
            DiagnosticsConstants.PULSE_WIDTH_STEPPER,
        )

        self.kick_power_slider.valueChanged.connect(
            lambda new_value: self.kick_power_label.setText(str(new_value))
        )

        self.chip_distance_slider.floatValueChanged.connect(
            lambda new_value: self.chip_distance_label.setText(str(new_value))
        )

        self.kicker_pulse_width_slider.valueChanged.connect(
            lambda new_value: self.kicker_pulse_width_label.setText(str(new_value))
        )

        self.chipper_pulse_width_slider.valueChanged.connect(
            lambda new_value: self.chipper_pulse_width_label.setText(str(new_value))
        )

        kick_chip_sliders_hbox_layout = QHBoxLayout()
        kick_chip_sliders_hbox_layout.addLayout(self.kick_power_slider_layout)
        kick_chip_sliders_hbox_layout.addLayout(self.chip_distance_slider_layout)
        kick_chip_sliders_hbox_layout.addLayout(self.kicker_pulse_width_slider_layout)
        kick_chip_sliders_hbox_layout.addLayout(self.chipper_pulse_width_slider_layout)

        kick_chip_sliders_box = QGroupBox()
        kick_chip_sliders_box.setLayout(kick_chip_sliders_hbox_layout)
        kick_chip_sliders_box.setTitle("Kick Power, Chip Distance, and Pulse Widths")

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

        self.no_auto_button.setChecked(True)
        self.no_auto_button.toggled.connect(
            self.__update_kick_chip_buttons_accessibility
        )

        self.power_mode_buttons_group = QButtonGroup()
        (
            self.power_mode_buttons_box,
            self.power_mode_buttons,
        ) = common_widgets.create_radio(
            ["Power (m/s or m)", "Power (Pulse Width)"], self.power_mode_buttons_group
        )
        self.power_mode_buttons_box.setTitle("Power Mode")

        self.meters_per_second_button = self.power_mode_buttons[0]
        self.pulse_width_button = self.power_mode_buttons[1]

        self.meters_per_second_button.clicked.connect(self.hide_pulse_width_show_power)
        self.pulse_width_button.clicked.connect(self.hide_power_show_pulse_width)

        self.meters_per_second_button.setChecked(True)
        self.hide_pulse_width_show_power()

        self.kick_button.clicked.connect(
            lambda: self.send_command_and_timeout(
                lambda: ChickerCommandMode.KICK_PULSE_WIDTH
                if self.pulse_width_button.isChecked()
                else ChickerCommandMode.KICK
            )
        )
        self.chip_button.clicked.connect(
            lambda: self.send_command_and_timeout(
                lambda: ChickerCommandMode.CHIP_PULSE_WIDTH
                if self.pulse_width_button.isChecked()
                else ChickerCommandMode.CHIP
            )
        )

        chicker_widget_vbox_layout.addWidget(self.auto_kick_chip_buttons_box)
        chicker_widget_vbox_layout.addWidget(self.power_mode_buttons_box)

    def hide_pulse_width_show_power(self):
        """Replace the pulse_width kicker/chipper sliders with the m/s and m ones"""
        for i in range(self.kicker_pulse_width_slider_layout.count()):
            self.kicker_pulse_width_slider_layout.itemAt(i).widget().hide()

        for i in range(self.chipper_pulse_width_slider_layout.count()):
            self.chipper_pulse_width_slider_layout.itemAt(i).widget().hide()

        for i in range(self.kick_power_slider_layout.count()):
            self.kick_power_slider_layout.itemAt(i).widget().show()

        for i in range(self.chip_distance_slider_layout.count()):
            self.chip_distance_slider_layout.itemAt(i).widget().show()

    def hide_power_show_pulse_width(self):
        """Replace the m/s and m kicker/chipper sliders with the pulse_width ones"""
        for i in range(self.kick_power_slider_layout.count()):
            self.kick_power_slider_layout.itemAt(i).widget().hide()

        for i in range(self.chip_distance_slider_layout.count()):
            self.chip_distance_slider_layout.itemAt(i).widget().hide()

        for i in range(self.kicker_pulse_width_slider_layout.count()):
            self.kicker_pulse_width_slider_layout.itemAt(i).widget().show()

        for i in range(self.chipper_pulse_width_slider_layout.count()):
            self.chipper_pulse_width_slider_layout.itemAt(i).widget().show()

    def send_command_and_timeout(self, command: ChickerCommandMode) -> None:
        """If the kick/chip buttons are enabled and unlocked, send a kick/chip command
        and then disable and lock the buttons. The buttons will be re-enabled and unlocked
        after CHICKER_TIMEOUT.

        :param command: the chicker command to send; should be either
                        ChickerCommandMode.KICK or ChickerCommandMode.CHIP
        """
        if not self.kick_chip_locked:
            self.kick_chip_locked = True

            common_widgets.disable_button(self.kick_button)
            common_widgets.disable_button(self.chip_button)

            self.send_command(command)

            def unlock_kick_chip():
                self.kick_chip_locked = False
                self.__update_kick_chip_buttons_accessibility()

            QTimer.singleShot(int(CHICKER_TIMEOUT), unlock_kick_chip)

    def send_command(self, command: ChickerCommandMode) -> None:
        """Send a [auto]kick or [auto]chip primitive with the currently set
        kick power or chip distance

        :param command: the type of chicker command to send
        """
        # Get slider values
        kick_power = self.kick_power_slider.value()
        chip_dist = self.chip_distance_slider.value()
        kick_pulse_width = self.kicker_pulse_width_slider.value()
        chip_pulse_width = self.chipper_pulse_width_slider.value()

        # Send kick, chip, autokick, or autochip primitive
        power_control = PowerControl()
        power_control.geneva_slot = 3

        if command == ChickerCommandMode.KICK:
            power_control.chicker.kick_speed_m_per_s = kick_power
        elif command == ChickerCommandMode.CHIP:
            power_control.chicker.chip_distance_meters = chip_dist
        elif command == ChickerCommandMode.AUTOKICK:
            power_control.chicker.auto_chip_or_kick.autokick_speed_m_per_s = kick_power
        elif command == ChickerCommandMode.AUTOCHIP:
            power_control.chicker.auto_chip_or_kick.autochip_distance_meters = chip_dist
        elif command == ChickerCommandMode.KICK_PULSE_WIDTH:
            power_control.chicker.kick_pulse_width = kick_pulse_width
        elif command == ChickerCommandMode.CHIP_PULSE_WIDTH:
            power_control.chicker.chip_pulse_width = chip_pulse_width
        elif command == ChickerCommandMode.AUTOKICK_PULSE_WIDTH:
            power_control.chicker.auto_chip_or_kick.autokick_pulse_width = (
                kick_pulse_width
            )
        elif command == ChickerCommandMode.AUTOCHIP_PULSE_WIDTH:
            power_control.chicker.auto_chip_or_kick.autochip_pulse_width = (
                chip_pulse_width
            )

        self.proto_unix_io.send_proto(PowerControl, power_control)

        if (
            command == ChickerCommandMode.KICK
            or command == ChickerCommandMode.CHIP
            or command == ChickerCommandMode.KICK_PULSE_WIDTH
            or command == ChickerCommandMode.CHIP_PULSE_WIDTH
        ):
            # We need to send an empty message to the PowerControl proto buffer to prevent the
            # spamming of kick/chip commands. This is because if no new messages are received in the
            # buffer, the last sent message will be repeatedly resent to the robot, which we don't
            # want for kick/chip.
            power_control = PowerControl()
            power_control.geneva_slot = 3
            self.proto_unix_io.send_proto(PowerControl, power_control, True)

    def enable(self) -> None:
        """Enable all sliders and buttons in the ChickerWidget"""
        if self.enabled:
            return

        self.enabled = True

        common_widgets.enable_slider(self.kick_power_slider)
        common_widgets.enable_slider(self.chip_distance_slider)
        common_widgets.enable_slider(self.kicker_pulse_width_slider)
        common_widgets.enable_slider(self.chipper_pulse_width_slider)
        common_widgets.enable_button(self.no_auto_button)
        common_widgets.enable_button(self.auto_kick_button)
        common_widgets.enable_button(self.auto_chip_button)
        common_widgets.enable_button(self.meters_per_second_button)
        common_widgets.enable_button(self.pulse_width_button)
        self.__update_kick_chip_buttons_accessibility()

    def disable(self) -> None:
        """Disable all sliders and buttons in the ChickerWidget"""
        if not self.enabled:
            return

        self.enabled = False

        common_widgets.disable_slider(self.kick_power_slider)
        common_widgets.disable_slider(self.chip_distance_slider)
        common_widgets.disable_slider(self.kicker_pulse_width_slider)
        common_widgets.disable_slider(self.chipper_pulse_width_slider)
        common_widgets.disable_button(self.no_auto_button)
        common_widgets.disable_button(self.auto_kick_button)
        common_widgets.disable_button(self.auto_chip_button)
        common_widgets.disable_button(self.kick_button)
        common_widgets.disable_button(self.chip_button)
        common_widgets.disable_button(self.meters_per_second_button)
        common_widgets.disable_button(self.pulse_width_button)

    def override_slider_values(self, kick_power: int, chip_distance: float) -> None:
        """Set the widget's sliders to match the given values.

        :param kick_power: the value to set the kick power slider to
        :param chip_distance: the value to set the chip distance slider to
        """
        self.kick_power_slider.setValue(kick_power)
        self.chip_distance_slider.setValue(chip_distance)

    def refresh(self) -> None:
        """Send out autokick/autochip commands if enabled"""
        if self.meters_per_second_button.isChecked():
            if self.auto_kick_button.isChecked():
                self.send_command(ChickerCommandMode.AUTOKICK)
            elif self.auto_chip_button.isChecked():
                self.send_command(ChickerCommandMode.AUTOCHIP)
        elif self.pulse_width_button.isChecked():
            if self.auto_kick_button.isChecked():
                self.send_command(ChickerCommandMode.AUTOKICK_PULSE_WIDTH)
            elif self.auto_chip_button.isChecked():
                self.send_command(ChickerCommandMode.AUTOCHIP_PULSE_WIDTH)

    def __update_kick_chip_buttons_accessibility(self) -> None:
        """Enable or disable the kick/chip buttons depending on whether autokick/autochip is on"""
        if (
            self.enabled
            and not self.kick_chip_locked
            and self.no_auto_button.isChecked()
        ):
            common_widgets.enable_button(self.kick_button)
            common_widgets.enable_button(self.chip_button)
        else:
            common_widgets.disable_button(self.kick_button)
            common_widgets.disable_button(self.chip_button)
