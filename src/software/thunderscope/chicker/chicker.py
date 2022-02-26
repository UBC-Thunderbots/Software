import pyqtgraph as pg
import software.thunderscope.constants as constants
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QApplication, QCheckBox, QGridLayout, QGroupBox, QButtonGroup,
                             QMenu, QPushButton, QRadioButton, QVBoxLayout, QHBoxLayout, QWidget, QSlider)

class ChickerWidget(QWidget):
    def __init__(self, parent=None):
        super(ChickerWidget, self).__init__(parent)

        grid = QGridLayout()
        self.radio_buttons = QButtonGroup()
        # grid --> groupBox --> button/slider
        grid.addWidget(self.createButton("Charge"), 0, 0)
        grid.addWidget(self.createButton("Kick"), 0, 1)
        grid.addWidget(self.createButton("Chip"), 0, 2)
        grid.addWidget(self.createRadio("No Auto"), 1, 0)
        grid.addWidget(self.createRadio("Auto Kick"), 1, 1)
        grid.addWidget(self.createRadio("Auto Chip"), 1, 2)
        grid.addWidget(self.createSlider("Geneva Slider", 1, 5, 1), 2, 0, 1, 3)
        grid.addWidget(self.createSlider("Power Slider", 1, 100, 10), 3, 0, 1, 3)
        self.setLayout(grid)
        self.grid = grid
        self.charged = False

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

        return groupBox

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

        return groupBox

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

        return groupBox

    def refresh(self):
        # set 'Charge'/'Discharge' text based on self.charged value
        for button in self.grid.parentWidget().findChildren(QPushButton):
            if self.charged:
                if button.text() == "Charge":
                    button.setText("Discharge")
            elif not self.charged:
                if button.text() == "Discharge":
                    button.setText("Charge")
                # set radio button to 'No Auto'
                for radio in self.grid.parentWidget().findChildren(QRadioButton):
                    if radio.text() == "No Auto":
                        radio.setChecked(True)

        # iterate over all boxes - possibly the only way to access the widgets
        # grid --> groupBox --> button/slider
        for item in self.grid.parentWidget().findChildren(QGroupBox):

            # check all push buttons
            for button in item.findChildren(QPushButton):
                if not self.charged:
                    # charge button white; text: "Charge"; setCheckable(True)
                    if button.text() == "Charge":
                        button.setStyleSheet("background-color: White")
                        button.setCheckable(True)
                    # kick/chip button grey; setCheckable(False)
                    else:
                        button.setStyleSheet("background-color: Grey")
                        button.setCheckable(False)
                    # behaviour upon clicking charge; change text to 'Discharge'; print; charged = True; toggle
                    if button.text() == "Charge" and button.isChecked():
                        button.toggle()
                        self.charged = True
                        print("Charge clicked") # ----- replace this line ----- (next PR)

                elif self.charged:
                    # discharge button white; text: "Discharge"; setCheckable(True)
                    if button.text() == "Discharge":
                        button.setStyleSheet("background-color: White")
                        button.setCheckable(True)
                    # kick/chip button white; setCheckable(True)
                    else:
                        button.setStyleSheet("background-color: White")
                        button.setCheckable(True)

                    if button.isChecked():
                        # discharge clicked; change text to 'Charge'; print; set 'No Auto' on; charged = False
                        if button.text() == "Discharge":
                            button.toggle()
                            self.charged = False
                            print("Discharge clicked") # ----- replace this line ----- (next PR)
                        # kick/chip button clicked; change text to 'Charge';
                        else:
                            button.toggle()
                            self.charged = False
                            if button.text() == "Kick":
                                print("Kick clicked") # ----- replace this line ----- (next PR)
                            elif button.text() == "Chip":
                                print("Chip clicked") # ----- replace this line ----- (next PR)

            # check all radio buttons, only if we are charged
            if self.charged:
                for radio in item.findChildren(QRadioButton):
                    if radio.isChecked():
                        # click no auto - change to no auto; Kick/chip white and enabled
                        if radio.text() == "No Auto":
                            # find kick/chip pushButtons
                            for button in self.grid.parentWidget().findChildren(QPushButton):
                                if button.text() == "Kick" or button.text() == "Chip":
                                    button.setStyleSheet("background-color: White")
                                    button.setCheckable(True)
                        # click auto chip/kick - charged = False; print; (may be)kick/chip grey, disabled
                        elif radio.text() == "Auto Kick" or "Auto Chip":
                            self.charged = False
                            if radio.text() == "Auto Chip":
                                print("Auto Chip clicked") # ----- replace this line ----- (next PR)
                            elif radio.text() == "Auto Kick":
                                print("Auto Kick clicked") # ----- replace this line ----- (next PR)
                            # find kick/chip pushButtons
                            for button in self.grid.parentWidget().findChildren(QPushButton):
                                if button.text() == "Kick" or button.text() == "Chip":
                                    button.setStyleSheet("background-color: Grey")
                                    button.setCheckable(False)