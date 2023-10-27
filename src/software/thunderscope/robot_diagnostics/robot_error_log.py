from pyqtgraph.Qt.QtWidgets import *


class RobotErrorLog(QWidget):
    def __init__(self):
        super(RobotErrorLog, self).__init__()

    def refresh(self):
        pass

    def add_error_log_message(self, message):
        print(message)
