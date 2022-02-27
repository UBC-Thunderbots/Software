import pyqtgraph as pg
from PyQt5.QtWidgets import *

class g3logCheckboxes(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        layout = QGridLayout()
        self.setLayout(layout)

        cbutton = QCheckBox("DEBUG")
        cbutton.setChecked(True)
        cbutton.log_level = "DEBUG"
        cbutton.toggled.connect(self.onClicked)
        layout.addWidget(cbutton, 0, 0)

        cbutton = QCheckBox("INFO")
        cbutton.setChecked(True)
        cbutton.log_level = "INFO"
        cbutton.toggled.connect(self.onClicked)
        layout.addWidget(cbutton, 1, 0)

        cbutton = QCheckBox("WARNING")
        cbutton.setChecked(True)
        cbutton.log_level = "WARNING"
        cbutton.toggled.connect(self.onClicked)
        layout.addWidget(cbutton, 2, 0)

        cbutton = QCheckBox("FATAL")
        cbutton.setChecked(True)
        cbutton.log_level = "FATAL"
        cbutton.toggled.connect(self.onClicked)
        layout.addWidget(cbutton, 3, 0)

        self.setStyleSheet(
            "color : white;"
            "background-color: black;"
            "selection-background-color: #606060;"
            "selection-color: #ffffff;"
        )

    def onClicked(self):
        cbutton = self.sender()
        print("Display " + (cbutton.log_level) + " logs is set to " + str(cbutton.isChecked()))
