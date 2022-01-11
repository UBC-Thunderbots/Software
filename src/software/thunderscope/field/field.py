import pyqtgraph as pg
from software.thunderscope.field.field_layer import FieldLayer


class Field(pg.PlotWidget):

    """ TODO Docstring for Field. """

    def __init__(self):
        pg.PlotWidget.__init__(self)

        # Setup Field Plot
        self.setAspectLocked()
        self.showGrid(x=True, y=True, alpha=0.4)

        # Setup Field Plot Legend
        self.legend = pg.LegendItem((80, 60), offset=(70, 20))
        self.legend.setParentItem(self.graphicsItem())

        # Fields
        self.layers = []

    def add_layer(self, name: str, layer: FieldLayer):
        self.layers.append(layer)
        self.addItem(layer)
        self.legend.addItem(layer, name)

    def refresh(self):
        for layer in self.layers:
            layer.update()
