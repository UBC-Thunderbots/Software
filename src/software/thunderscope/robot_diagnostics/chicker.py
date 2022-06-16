import pyqtgraph as pg
from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.Qt.QtWidgets import *
from software.py_constants import *
import software.thunderscope.common.common_widgets as common_widgets

from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class ChickerWidget(QWidget):
    def __init__(self):

        """The Chicker Widget grid is laid out in the following way:

                    ┌────────┐     ┌──────┐      ┌──────┐
                    │ Charge │     │ Kick │      │ Chip │
                    └────────┘     └──────┘      └──────┘

                    ┌────────┐   ┌──────────┐ ┌─────────┐
                    │No Auto │   │Auto Kick │ │Auto Chip│
                    └────────┘   └──────────┘ └─────────┘

                    ┌───────────────────────────────────┐
                    │           Geneva Slider           │
                    └───────────────────────────────────┘

                    ┌───────────────────────────────────┐
                    │           Power Slider            │
                    └───────────────────────────────────┘
        """

        super(ChickerWidget, self).__init__()

        vbox_layout = QVBoxLayout()
        self.radio_buttons_group = QButtonGroup()

        # push button group box
        self.push_button_box, self.push_buttons = common_widgets.create_button(
            ["Charge", "Kick", "Chip"]
        )
        self.charge_button = self.push_buttons[0]
        self.kick_button = self.push_buttons[1]
        self.chip_button = self.push_buttons[2]
        vbox_layout.addWidget(self.push_button_box)

        # radio button group box
        self.radio_button_box, self.radio_buttons = common_widgets.create_radio(
            ["No Auto", "Auto Kick", "Auto Chip"], self.radio_buttons_group
        )
        self.no_auto_button = self.radio_buttons[0]
        self.auto_kick_button = self.radio_buttons[1]
        self.auto_chip_button = self.radio_buttons[2]

        vbox_layout.addWidget(self.radio_button_box)
        self.no_auto_button.setChecked(True)

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
        ) = common_widgets.create_slider("Power", 1, 100, 10)
        vbox_layout.addLayout(self.power_slider_layout)

        self.setLayout(vbox_layout)

        # to manage the state of radio buttons - to make sure message is only sent once
        self.radio_checkable = {"no_auto": True, "auto_kick": True, "auto_chip": True}
        self.charged = False

        # initial values
        self.geneva_value = 3
        self.power_value = 1

        self.geneva_slider.setValue(self.geneva_value)
        self.power_slider.setValue(self.power_value)

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

        # set 'Charge'/'Discharge' text based on self.charged value
        if self.charged:
            self.charge_button.setText("Discharge")
        else:
            self.charge_button.setText("Charge")

        # slider values
        self.geneva_value = self.geneva_slider.value()
        self.geneva_label.setText(str(self.geneva_value))
        self.power_value = self.power_slider.value()
        self.power_label.setText(str(self.power_value))

        # button colors
        if not self.charged:
            self.change_button_state(self.charge_button, True)
            self.change_button_state(self.chip_button, False)
            self.change_button_state(self.kick_button, False)
        else:
            self.change_button_state(self.charge_button, True)
            self.change_button_state(self.chip_button, True)
            self.change_button_state(self.kick_button, True)

        # check all buttons
        if self.charge_button.isChecked():
            self.charge_button.toggle()
            if self.charged:
                self.charged = False
            else:
                self.charged = True

        if self.kick_button.isChecked():
            self.kick_button.toggle()
            self.charged = False

        if self.chip_button.isChecked():
            self.chip_button.toggle()
            self.charged = False

        # radio colors
        if self.no_auto_button.isChecked():
            self.change_button_state(self.charge_button, True)

            if self.charged:
                self.change_button_state(self.chip_button, True)
                self.change_button_state(self.kick_button, True)

        elif self.auto_kick_button.isChecked() or self.auto_chip_button.isChecked():
            self.change_button_state(self.chip_button, False)
            self.change_button_state(self.kick_button, False)
            self.change_button_state(self.charge_button, False)
