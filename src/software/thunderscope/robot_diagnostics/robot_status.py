from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph import parametertree
from google.protobuf.json_format import MessageToDict
from thefuzz import fuzz
from proto.import_all_protos import *

class RobotStatus(QWidget):


    def __init__(self):
        layout = QVBoxLayout()
        self.setLayout(layout)

        self.param_group = parametertree.Parameter.create(
            name="RobotStatus",
            type="group",
            children=
        )
