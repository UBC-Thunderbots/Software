import pyqtgraph as pg
from pyqtgraph.Qt.QtCore import *
from pyqtgraph.Qt.QtWidgets import *
from software.py_constants import *
from threading import Timer
from proto.import_all_protos import *
import software.thunderscope.common.common_widgets as common_widgets

from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


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

        # QObject.connect(self.kick_button, SIGNAL ('clicked()'), self.kick_clicked())

        self.kick_button.clicked.connect(self.kick_clicked)
        self.chip_button.clicked.connect(self.chip_clicked)

        vbox_layout.addWidget(self.push_button_box)

        # radio button group box
        self.radio_button_box, self.radio_buttons = common_widgets.create_radio(
            ["No Auto", "Auto Kick", "Auto Chip"], self.radio_buttons_group
        )
        self.no_auto_button = self.radio_buttons[0]
        self.auto_kick_button = self.radio_buttons[1]
        self.auto_chip_button = self.radio_buttons[2]

        self.no_auto_button.toggled.connect(self.buttons_enable)
        self.auto_kick_button.toggled.connect(self.buttons_disable)
        self.auto_chip_button.toggled.connect(self.buttons_disable)

        vbox_layout.addWidget(self.radio_button_box)
        self.no_auto_button.setChecked(True)

        self.kick_button_enable = True
        self.chip_button_enable = True

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
        if self.kick_button_enable:
            self.send_kick_or_chip(True)
            self.toggle_kick_button()
            t = Timer(3, lambda s: s.toggle_kick_button(), [self])
            t.start()

    def chip_clicked(self):
        if self.chip_button_enable:
            self.send_kick_or_chip(False)
            self.toggle_chip_button()
            t = Timer(3, lambda s: s.toggle_chip_button(), [self])
            t.start()

    def toggle_kick_button(self):
        self.kick_button_enable = not self.kick_button_enable

    def toggle_chip_button(self):
        self.chip_button_enable = not self.chip_button_enable

    def buttons_disable(self):
        self.kick_button_enable = False
        self.chip_button_enable = False

    def buttons_enable(self):
        self.kick_button_enable = True
        self.chip_button_enable = True

    def send_kick_or_chip(self, kick):
        geneva_value = self.geneva_slider.value()
        self.geneva_label.setText(Slot.Name(geneva_value))

        power_value = self.power_slider.value()
        self.power_label.setText(str(power_value))

        power_control = PowerControl()
        power_control.geneva_slot = geneva_value

        if kick:
            power_control.chicker.auto_chip_or_kick.autokick_speed_m_per_s = power_value
        else:
            power_control.chicker.auto_chip_or_kick.autochip_distance_meters = (
                power_value
            )

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

        # slider values
        self.change_button_state(self.kick_button, self.kick_button_enable)
        self.change_button_state(self.chip_button, self.chip_button_enable)

        # If auto is enabled, we want to populate the autochip or kick message
        if self.auto_kick_button.isChecked():
            self.send_kick_or_chip(True)
        elif self.auto_chip_button.isChecked():
            self.send_kick_or_chip(False)
        elif self.no_auto_button.isChecked():
            pass
