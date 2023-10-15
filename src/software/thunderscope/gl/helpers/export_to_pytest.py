from typing import Dict, Optional
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *

class PytestBuilder():

    IMPORTS = [
        ""
    ]

    def __init__(self):


    def build(self, robots: Dict[int, Optional[QtGui.QVector3D]]):
        filename, _ = QFileDialog.getSaveFileName(
            caption="Save Pytest as",
            options=QFileDialog.Option.DontUseNativeDialog,
        )

        if not filename:
            return

        print(filename)

        with open(filename, "w") as pytest_file:
            pytest_file.write("import pytest\n")
            pytest_file.write("import software.python_bindings as tbots")