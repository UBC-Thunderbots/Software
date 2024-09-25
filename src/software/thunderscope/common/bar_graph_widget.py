import pyqtgraph as pg
from pyqtgraph.Qt import QtGui
from PyQt6.QtWidgets import *

from typing import Dict


class BarGraphWidget(QWidget):
    """Displays a bar graph in a pyqtgraph plot"""

    DEFAULT_BAR_COLOUR = QtGui.QColor(128, 128, 128)

    def __init__(self):
        """Creates a BarGraphWidget"""
        super().__init__()

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.plot_widget = pg.PlotWidget()
        self.layout.addWidget(self.plot_widget)

    def set_data(
        self, data: Dict[str, float], colours: Dict[str, QtGui.QColor] = {}
    ) -> None:
        """Sets the data displayed in the bar graph.

        The data should be a dictionary where each key-value pair is a bar
        in the graph. The key is the label for the bar, and the value is the
        height of the bar.

        :param data: the data to display
        :param colours: optional dict specifying bar colour (key is bar label)
        """
        x_labels = list(data.keys())
        y_values = list(data.values())
        x_values = list(range(1, len(x_labels) + 1))

        brushes = [
            pg.mkBrush(colours.get(label, BarGraphWidget.DEFAULT_BAR_COLOUR))
            for label in x_labels
        ]

        bar_graph = pg.BarGraphItem(
            x=x_values, height=y_values, width=0.6, brushes=brushes
        )

        self.plot_widget.clear()
        self.plot_widget.addItem(bar_graph)

        ax = self.plot_widget.getAxis("bottom")
        ax.setTicks([list(zip(x_values, x_labels))])

        self.plot_widget.plot()

    def set_vertical_axis_label(self, label: str) -> None:
        """Sets the the label to display on the vertical axis

        :param label: the label to display on the vertical axis
        """
        self.plot_widget.setLabel(axis="left", text=label)
