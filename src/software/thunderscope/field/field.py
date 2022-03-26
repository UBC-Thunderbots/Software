import pyqtgraph as pg
import time
from software.thunderscope.field.field_layer import FieldLayer


class Field(pg.PlotWidget):

    """Wrapper to handle Field Layers"""

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

    def keyPressEvent(self, event):
        """Propagate keypress event to all field layers
        
        :param event: The event
        
        """
        for layer in self.layers:
            layer.keyPressEvent(event)

    def keyReleaseEvent(self, event):
        """Propagate keyrelease event to all field layers
        
        :param event: The event
        
        """
        for layer in self.layers:
            layer.keyReleaseEvent(event)

    def add_layer(self, name: str, layer: FieldLayer):
        """Add a layer to this field and to the legend.
        
        :param name: The name of the layer
        :param layer: The FieldLayer graphics object

        """
        self.layers.append(layer)
        self.addItem(layer)
        self.legend.addItem(layer, name)

    def refresh(self):
        """Trigger an update on all the layers
        """
        for layer in self.layers:
            layer.update()
