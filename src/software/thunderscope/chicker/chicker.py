import pyqtgraph as pg
import software.thunderscope.constants as constants
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QGridLayout,
    QGroupBox,
    QButtonGroup,
    QMenu,
    QPushButton,
    QRadioButton,
    QVBoxLayout,
    QHBoxLayout,
    QWidget,
    QSlider,
)


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

        self.charge_button_list = self.createButton("Charge")
        self.charge_button = self.charge_button_list[1]
        grid.addWidget(self.charge_button_list[0], 0, 0)

        self.kick_button_list = self.createButton("Kick")
        self.kick_button = self.kick_button_list[1]
        grid.addWidget(self.kick_button_list[0], 0, 1)

        self.chip_button_list = self.createButton("Chip")
        self.chip_button = self.chip_button_list[1]
        grid.addWidget(self.chip_button_list[0], 0, 2)

        self.no_auto_list = self.createRadio("No Auto")
        self.no_auto_button = self.no_auto_list[1]
        grid.addWidget(self.no_auto_list[0], 1, 0)

        self.auto_kick_list = self.createRadio("Auto Kick")
        self.auto_kick_button = self.auto_kick_list[1]
        grid.addWidget(self.auto_kick_list[0], 1, 1)

        self.auto_chip_list = self.createRadio("Auto Chip")
        self.auto_chip_button = self.auto_chip_list[1]
        grid.addWidget(self.auto_chip_list[0], 1, 2)

        self.geneva_slider_list = self.createSlider("Geneva Slider", 1, 5, 1)
        self.geneva_slider = self.geneva_slider_list[1]
        grid.addWidget(self.geneva_slider_list[0], 2, 0, 1, 3)

        self.power_slider_list = self.createSlider("Power Slider", 1, 100, 10)
        self.power_slider = self.power_slider_list[1]
        grid.addWidget(self.power_slider_list[0], 3, 0, 1, 3)

        self.setLayout(grid)
        self.grid = grid
        # state of radio buttons in order NoAuto, AutoKick, AutoChip
        self.radioCheckable = [True, True, True]
        self.charged = False
        self.geneva_value = 1
        self.power_value = 1

    # all the buttons, radios, and sliders are in a group box --> vbox (vertical alignment)
    def createButton(self, text):
        groupBox = QGroupBox()
        button = QPushButton(text)
        groupBox.setStyleSheet("color: black")
        button.setCheckable(True)
        if text != "Charge":
            button.setCheckable(False)
            button.setStyleSheet("background-color: Grey")
        vbox = QVBoxLayout()
        vbox.addWidget(button)
        vbox.addStretch(1)
        groupBox.setLayout(vbox)
        return groupBox, button

    def createRadio(self, text):
        groupBox = QGroupBox()
        radio = QRadioButton(text)
        groupBox.setStyleSheet("color: black")
        # this is so that the button is properly visible (it looked very dim otherwise)
        radio.setStyleSheet("background-color: white")
        self.radio_buttons.addButton(radio)
        vbox = QVBoxLayout()
        vbox.addWidget(radio)
        vbox.addStretch(1)
        groupBox.setLayout(vbox)
        return groupBox, radio

    def createSlider(self, text, minVal, maxVal, tickSpacing):
        groupBox = QGroupBox(text)
        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(minVal)
        slider.setMaximum(maxVal)
        slider.setTickPosition(QSlider.TicksBothSides)
        slider.setTickInterval(tickSpacing)
        groupBox.setStyleSheet("color: white")
        vbox = QVBoxLayout()
        vbox.addWidget(slider)
        vbox.addStretch(1)
        groupBox.setLayout(vbox)
        return groupBox, slider

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
            self.charge_button.setStyleSheet("background-color: White")
            self.charge_button.setCheckable(True)
            self.chip_button.setStyleSheet("background-color: Grey")
            self.chip_button.setCheckable(False)
            self.kick_button.setStyleSheet("background-color: Grey")
            self.kick_button.setCheckable(False)
        else:
            self.charge_button.setStyleSheet("background-color: White")
            self.charge_button.setCheckable(True)
            self.chip_button.setStyleSheet("background-color: White")
            self.chip_button.setCheckable(True)
            self.kick_button.setStyleSheet("background-color: White")
            self.kick_button.setCheckable(True)

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
            self.radioCheckable[1] = True
            self.radioCheckable[2] = True
            if self.radioCheckable[0]:
                print("No Auto clicked")
                print("Geneva:", self.geneva_value, "Power:", self.power_value)
            self.radioCheckable[0] = False
            self.charge_button.setStyleSheet("background-color: White")
            self.charge_button.setCheckable(True)
            if self.charged:
                self.chip_button.setStyleSheet("background-color: White")
                self.chip_button.setCheckable(True)
                self.kick_button.setStyleSheet("background-color: White")
                self.kick_button.setCheckable(True)

        elif self.auto_kick_button.isChecked():
            self.radioCheckable[0] = True
            self.radioCheckable[2] = True
            if self.radioCheckable[1]:
                print("Auto Kick clicked")
                print("Geneva:", self.geneva_value, "Power:", self.power_value)
            self.radioCheckable[1] = False
            self.chip_button.setStyleSheet("background-color: Grey")
            self.chip_button.setCheckable(False)
            self.kick_button.setStyleSheet("background-color: Grey")
            self.kick_button.setCheckable(False)
            self.charge_button.setStyleSheet("background-color: Grey")
            self.charge_button.setCheckable(False)

        elif self.auto_chip_button.isChecked():
            self.radioCheckable[0] = True
            self.radioCheckable[1] = True
            if self.radioCheckable[2]:
                print("Auto Kick clicked")
                print("Geneva:", self.geneva_value, "Power:", self.power_value)
            self.radioCheckable[2] = False
            self.chip_button.setStyleSheet("background-color: Grey")
            self.chip_button.setCheckable(False)
            self.kick_button.setStyleSheet("background-color: Grey")
            self.kick_button.setCheckable(False)
            self.charge_button.setStyleSheet("background-color: Grey")
            self.charge_button.setCheckable(False)
