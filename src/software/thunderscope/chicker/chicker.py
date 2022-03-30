import pyqtgraph as pg
from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.Qt.QtWidgets import *
import software.thunderscope.constants as constants
import software.thunderscope.common_widgets as common_widgets


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
        self.radio_buttons_group = QButtonGroup()

        # push button group box
        self.push_button_box, self.push_buttons = common_widgets.create_button(
            ["Charge", "Kick", "Chip"]
        )
        self.charge_button = self.push_buttons[0]
        self.kick_button = self.push_buttons[1]
        self.chip_button = self.push_buttons[2]
        grid.addWidget(self.push_button_box, 0, 0)

        # radio button group box
        self.radio_button_box, self.radio_buttons = common_widgets.create_radio(
            ["No Auto", "Auto Kick", "Auto Chip"], self.radio_buttons_group
        )
        self.no_auto_button = self.radio_buttons[0]
        self.auto_kick_button = self.radio_buttons[1]
        self.auto_chip_button = self.radio_buttons[2]
        grid.addWidget(self.radio_button_box, 1, 0)
        self.no_auto_button.setChecked(True)

        # sliders
        (
            self.geneva_slider_box,
            self.geneva_slider,
            self.geneva_label,
        ) = common_widgets.create_slider(
            "Geneva Slider", 1, constants.GENEVA_NUM_POSITIONS, 1
        )
        grid.addWidget(self.geneva_slider_box, 2, 0)
        (
            self.power_slider_box,
            self.power_slider,
            self.power_label,
        ) = common_widgets.create_slider("Power Slider", 1, 100, 10)
        grid.addWidget(self.power_slider_box, 3, 0)

        self.setLayout(grid)
        self.grid = grid
        # to manage the state of radio buttons - to make sure message is only sent once
        self.radio_checkable = {"no_auto": True, "auto_kick": True, "auto_chip": True}
        self.charged = False
        # initial values
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
