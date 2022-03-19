import pyqtgraph as pg
from PyQt6.QtWidgets import *


class g3logCheckboxes(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        layout = QGridLayout()
        self.setLayout(layout)

        # Creates 4 checkboxes based on the 4 log types
        self.debug_checkbox = QCheckBox("DEBUG")
        self.debug_checkbox.setChecked(True)
        layout.addWidget(self.debug_checkbox, 0, 0)

        self.info_checkbox = QCheckBox("INFO")
        self.info_checkbox.setChecked(True)
        layout.addWidget(self.info_checkbox, 0, 1)

        self.warning_checkbox = QCheckBox("WARNING")
        self.warning_checkbox.setChecked(True)
        layout.addWidget(self.warning_checkbox, 0, 2)

        self.fatal_checkbox = QCheckBox("FATAL")
        self.fatal_checkbox.setChecked(True)
        layout.addWidget(self.fatal_checkbox, 0, 3)

        self.setStyleSheet(
            "color : white;"
            "background-color: black;"
            "selection-background-color: #606060;"
            "selection-color: #ffffff;"
        )
