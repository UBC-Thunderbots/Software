from typing import  List
from PyQt6.QtGui import QKeySequence
from pyqtgraph.Qt import QtGui
from pyqtgraph.Qt.QtWidgets import *
from proto.import_all_protos import *
from software.thunderscope.gl.widgets.gl_toolbar import GLToolbar

class MultilayerToolbar(QWidget):
    def __init__(self, parent: QWidget, toolbars: List[GLToolbar]):
        super().__init__(parent)

        self.setLayout(QHBoxLayout())

        self.toolbars: List[GLToolbar] = toolbars

        for toolbar in toolbars:
            self.layout().addWidget(toolbar)
            toolbar.hide()

        # Create a shortcut for the F1 key
        self.shortcut_f1 = QtGui.QShortcut(QKeySequence("F1"), self)
        self.shortcut_f1.activated.connect(lambda: self.show_toolbar(0))

        self.shortcut_f1 = QtGui.QShortcut(QKeySequence("F2"), self)
        self.shortcut_f1.activated.connect(lambda: self.show_toolbar(1))

    def add_toolbar(self, toolbar: GLToolbar):
        self.toolbars.append(toolbar)
        self.layout().addWidget(toolbar)
        toolbar.hide()

        self.show_toolbar(0)

    def show_toolbar(self, num):
        # cannot show toolbar, since index is out of range
        if num >= len(self.toolbars):
            return

        for toolbar in self.toolbars:
            toolbar.hide()

        self.toolbars[num].show()
