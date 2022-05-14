from pyqtgraph.Qt.QtWidgets import QVBoxLayout, QWidget, QGridLayout, QCheckBox
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class g3logCheckboxes(QWidget):
    def __init__(self):
        """Check boxes to filter g3log levels
        """
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
