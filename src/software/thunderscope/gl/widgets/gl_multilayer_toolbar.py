from typing import List
from PyQt6.QtGui import QKeySequence
from pyqtgraph.Qt import QtGui
from pyqtgraph.Qt.QtWidgets import *
from proto.import_all_protos import *
from software.thunderscope.gl.widgets.gl_toolbar import GLToolbar


class MultilayerToolbar(QWidget):
    """A widget where it allows user to easily switch between layers by pressing F keys."""

    def __init__(self, parent: QWidget, toolbars: List[GLToolbar]):
        """Initialize the MultilayerToolbar Widget

        :param parent: the parent widget of this toolbar
        :param toolbars: a list of toolbars to be swapped
        """
        super().__init__(parent)

        # TODO: as of current, this layer only supports two layers, but
        # it can be easily extended to support multiple layers in the future
        assert len(toolbars) <= 2

        self.setLayout(QHBoxLayout())

        self.toolbars: List[GLToolbar] = toolbars

        for toolbar in toolbars:
            self.layout().addWidget(toolbar)
            toolbar.hide()

        # Create a shortcut for the F1 key
        self.shortcut_f1 = QtGui.QShortcut(QKeySequence("F1"), self)
        self.shortcut_f1.activated.connect(lambda: self.show_toolbar(0))

        self.shortcut_f2 = QtGui.QShortcut(QKeySequence("F2"), self)
        self.shortcut_f2.activated.connect(lambda: self.show_toolbar(1))

    def add_toolbar(self, toolbar: GLToolbar):
        """Adding a toolbar to the widget. Appends the toolbar at the end of the array

        :param toolbar: the toolbar going to be added to this layer
        """
        # we only support 2 toolbars as of current
        assert len(self.toolbars) <= 1

        self.toolbars.append(toolbar)
        self.layout().addWidget(toolbar)
        toolbar.hide()

        self.show_toolbar(0)

    def show_toolbar(self, num):
        """Display a toolbar

        :param num: the index of the toolbar in the toolbars array going to be displayed
        """
        # cannot show toolbar, since index is out of range
        if num >= len(self.toolbars):
            return

        for toolbar in self.toolbars:
            toolbar.hide()

        self.toolbars[num].show()
