import pyqtgraph as pg

from software.thunderscope.field.field_layer import FieldLayer


class Field(pg.PlotWidget):

    """Wrapper to handle Field Layers"""

    MAX_X_RANGE = 10000
    MAX_Y_RANGE = 6000

    def __init__(self):
        pg.PlotWidget.__init__(self)

        # Setup Field Plot
        self.setAspectLocked()
        self.showGrid(x=True, y=True, alpha=0.5)

        # NOTE: This line has caused a lot of grief. DO NOT remove this, or you
        # will have a _severe_ performance penalty.
        #
        # If auto range is enabled, pyqtgraph will recalculate the range of the
        # field plot every time something changes. Since we have a lot of things
        # going on the field, this really bogs down the system.
        #
        # At the time of writing, this capped us at 30 FPS (when we were
        # trying to run at 60fps). Removing this allowed us to run at 60 FPS.
        #
        # More info here: https://groups.google.com/g/pyqtgraph/c/7dR_-3EP29k
        self.disableAutoRange()

        # Only call auto range once
        self.range_set = False

        self.setMouseTracking(True)

        # Setup Field Plot Legend
        self.legend = pg.LegendItem((80, 60), offset=(70, 20))
        self.legend.setParentItem(self.graphicsItem())

        # Fields
        self.layers = []

    def keyPressEvent(self, event):
        """Propagate keypress event to all field layers
        
        :param event: The event
        
        """
        self.setMouseEnabled(x=False, y=False)
        for layer in self.layers:
            layer.keyPressEvent(event)

    def keyReleaseEvent(self, event):
        """Propagate keyrelease event to all field layers
        
        :param event: The event
        
        """
        self.setMouseEnabled(x=True, y=True)
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

        # Set the field range once
        if not self.range_set:
            self.setRange(
                xRange=(-int(Field.MAX_X_RANGE / 2), int(Field.MAX_X_RANGE / 2)),
                yRange=(-int(Field.MAX_Y_RANGE / 2), int(Field.MAX_Y_RANGE / 2)),
            )
            self.range_set = True
