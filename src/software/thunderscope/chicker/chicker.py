import pyqtgraph as pg
import software.thunderscope.constants as constants
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QApplication, QCheckBox, QGridLayout, QGroupBox,
                             QMenu, QPushButton, QRadioButton, QVBoxLayout, QHBoxLayout, QWidget, QSlider)

class ChickerWidget(QWidget):
    def __init__(self, parent=None):
        super(ChickerWidget, self).__init__(parent)

        grid = QGridLayout()
        grid.addWidget(self.createButton("Charge"), 0, 0)
        grid.addWidget(self.createButton("Kick"), 0, 1)
        grid.addWidget(self.createButton("Auto Kick"), 0, 2)
        grid.addWidget(self.createButton("Discharge"), 1, 0)
        grid.addWidget(self.createButton("Chip"), 1, 1)
        grid.addWidget(self.createButton("Auto Chip"), 1, 2)
        grid.addWidget(self.createSlider("Geneva Slider", 0, 5, 1), 2, 0, 1, 3)
        grid.addWidget(self.createSlider("Power Slider", 0, 100, 10), 3, 0, 1, 3)
        self.setLayout(grid)
        self.grid = grid

    def createButton(self, text):
        groupBox = QGroupBox()
        button = QPushButton(text)
        groupBox.setStyleSheet("color: white")
        button.setStyleSheet("color: black")
        button.setCheckable(True)
        # button.toggle()
        if text != "Charge":
            button.setStyleSheet("background-color: grey")

        vbox = QVBoxLayout()
        vbox.addWidget(button)
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
        # iterate over all items in grid

        for button in self.grid.parentWidget().findChildren(QPushButton):
            if button.text() == "Charge" and button.isChecked():
                button.toggle()
                print("charge clicked")
                for otherButton in self.grid.parentWidget().findChildren(QPushButton):
                    otherButton.setStyleSheet("color: black")
                    if otherButton.text() != "Charge":
                        otherButton.setStyleSheet("background-color: White")
                    else:
                        otherButton.setStyleSheet("background-color: Grey")

            elif button.text() != "Charge" and button.isChecked():
                button.toggle()
                print("other button clicked")
                for otherButton in self.grid.parentWidget().findChildren(QPushButton):
                    otherButton.setStyleSheet("color: black")
                    if otherButton.text() == "Charge":
                        otherButton.setStyleSheet("background-color: White")
                    else:
                        otherButton.setStyleSheet("background-color: Grey")
            """

        for button in self.grid.parentWidget().findChildren(QPushButton):
            if button.text() == "Charge" and button.isChecked():
                print("can click")

            elif button.text() == "Charge" and not button.isChecked():
                print("disabled")
        
        """