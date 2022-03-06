import pyqtgraph as pg
from PyQt5.QtWidgets import *

class g3logCheckboxes(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        layout = QGridLayout()
        self.setLayout(layout)

        self.cbutton1 = QCheckBox("DEBUG")
        self.cbutton1.setChecked(True)
        self.cbutton1.log_level = "DEBUG"
        self.cbutton1.stateChanged.connect(self.onClicked)
        layout.addWidget(self.cbutton1, 0, 0)

        self.cbutton2 = QCheckBox("INFO")
        self.cbutton2.setChecked(True)
        self.cbutton2.log_level = "INFO"
        self.cbutton2.toggled.connect(self.onClicked)
        layout.addWidget(self.cbutton2, 1, 0)

        self.cbutton3 = QCheckBox("WARNING")
        self.cbutton3.setChecked(True)
        self.cbutton3.log_level = "WARNING"
        self.cbutton3.toggled.connect(self.onClicked)
        layout.addWidget(self.cbutton3, 2, 0)

        self.cbutton4 = QCheckBox("FATAL")
        self.cbutton4.setChecked(True)
        self.cbutton4.log_level = "FATAL"
        self.cbutton4.toggled.connect(self.onClicked)
        layout.addWidget(self.cbutton4, 3, 0)

        self.setStyleSheet(
            "color : white;"
            "background-color: black;"
            "selection-background-color: #606060;"
            "selection-color: #ffffff;"
        )

    def onClicked(self):
        cbutton = self.sender()
        print("Display " + (cbutton.log_level) + " logs is set to " + str(cbutton.isChecked()))

    def isChecked1(self):
        return self.cbutton1.isChecked()

    def isChecked2(self):
        return self.cbutton2.isChecked()

    def isChecked3(self):
        return self.cbutton3.isChecked()
    
    def isChecked4(self):
        return self.cbutton4.isChecked()
