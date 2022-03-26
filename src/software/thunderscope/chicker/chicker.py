import pyqtgraph as pg
import software.thunderscope.constants as constants
import software.thunderscope.common_widgets as common_widgets
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import *


class ChickerWidget(QWidget):
    def __init__(self, parent=None):
        super(ChickerWidget, self).__init__(parent)

        """
        The Chicker Widget grid is laid out in the following way:
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
        # grid --> groupBox --> button/slider
        grid = QGridLayout()
        self.radio_buttons = QButtonGroup()

        self.charge_button_list = common_widgets.create_button("Charge", False)
        self.charge_button = self.charge_button_list[1]
        grid.addWidget(self.charge_button_list[0], 0, 0)

        self.kick_button_list = common_widgets.create_button("Kick")
        self.kick_button = self.kick_button_list[1]
        grid.addWidget(self.kick_button_list[0], 0, 1)

        self.chip_button_list = common_widgets.create_button("Chip")
        self.chip_button = self.chip_button_list[1]
        grid.addWidget(self.chip_button_list[0], 0, 2)

        self.no_auto_list = common_widgets.create_radio("No Auto", self.radio_buttons)
        self.no_auto_button = self.no_auto_list[1]
        grid.addWidget(self.no_auto_list[0], 1, 0)

        self.auto_kick_list = common_widgets.create_radio(
            "Auto Kick", self.radio_buttons
        )
        self.auto_kick_button = self.auto_kick_list[1]
        grid.addWidget(self.auto_kick_list[0], 1, 1)

        self.auto_chip_list = common_widgets.create_radio(
            "Auto Chip", self.radio_buttons
        )
        self.auto_chip_button = self.auto_chip_list[1]
        grid.addWidget(self.auto_chip_list[0], 1, 2)

        self.geneva_slider_list = common_widgets.create_slider("Geneva Slider", 1, 5, 1)
        self.geneva_slider = self.geneva_slider_list[1]
        grid.addWidget(self.geneva_slider_list[0], 2, 0, 1, 3)

        self.power_slider_list = common_widgets.create_slider(
            "Power Slider", 1, 100, 10
        )
        self.power_slider = self.power_slider_list[1]
        grid.addWidget(self.power_slider_list[0], 3, 0, 1, 3)

        self.setLayout(grid)
        self.grid = grid
        # state of radio buttons - to make sure message is only sent once
        self.radio_checkable = {"no_auto": True, "auto_kick": True, "auto_chip": True}
        self.charged = False
        self.geneva_value = 1
        self.power_value = 1

    def change_button_state(self, button, enable):
        """
        Change button color and clickable state.

        :param button: button to change the state of
        :param enable: bool: if True: enable this button, if False: disable
        :return:
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
        self.power_value = self.power_slider.value()

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
                print("Discharge clicked")
                self.charged = False
            else:
                print("Charge clicked")
                self.charged = True
            print("Geneva:", self.geneva_value, "Power:", self.power_value)

        if self.kick_button.isChecked():
            self.kick_button.toggle()
            self.charged = False
            print("Kick clicked")
            print("Geneva:", self.geneva_value, "Power:", self.power_value)

        if self.chip_button.isChecked():
            self.chip_button.toggle()
            self.charged = False
            print("Chip clicked")
            print("Geneva:", self.geneva_value, "Power:", self.power_value)

        # radio colors
        if self.no_auto_button.isChecked():
            self.radio_checkable["auto_kick"] = True
            self.radio_checkable["auto_chip"] = True
            if self.radio_checkable["no_auto"]:
                print("No Auto clicked")
                print("Geneva:", self.geneva_value, "Power:", self.power_value)
            self.radio_checkable["no_auto"] = False
            self.change_button_state(self.charge_button, True)
            if self.charged:
                self.change_button_state(self.chip_button, True)
                self.change_button_state(self.kick_button, True)

        elif self.auto_kick_button.isChecked():
            self.radio_checkable["no_auto"] = True
            self.radio_checkable["auto_chip"] = True
            if self.radio_checkable["auto_kick"]:
                print("Auto Kick clicked")
                print("Geneva:", self.geneva_value, "Power:", self.power_value)
            self.radio_checkable["auto_kick"] = False
            self.change_button_state(self.chip_button, False)
            self.change_button_state(self.kick_button, False)
            self.change_button_state(self.charge_button, False)

        elif self.auto_chip_button.isChecked():
            self.radio_checkable["no_auto"] = True
            self.radio_checkable["auto_kick"] = True
            if self.radio_checkable["auto_chip"]:
                print("Auto Chip clicked")
                print("Geneva:", self.geneva_value, "Power:", self.power_value)
            self.radio_checkable["auto_chip"] = False
            self.change_button_state(self.chip_button, False)
            self.change_button_state(self.kick_button, False)
            self.change_button_state(self.charge_button, False)
