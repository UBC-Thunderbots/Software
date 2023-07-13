import pyqtgraph as pg

from software.thunderscope.gl.helpers.graphics_list import GraphicsList


class GLLayer(pg.GraphicsObject):
    """Represents a layer in the 3D visualization. A layer manages and returns GLGraphicsItem 
    graphics to be displayed in the 3D scene.
    """

    def __init__(self):
        """Initialize the GLLayer"""
        pg.GraphicsObject.__init__(self)

        # Options for the layer, used to configure the legend
        self.opts = {
            "pxMode": True,
            "useCache": True,
            "antialias": True,
            "name": None,
            "symbol": "o",
            "size": 7,
            "pen": pg.mkPen("w"),
            "brush": pg.mkBrush("w"),
            "tip": "x: {x:.3g}\ny: {y:.3g}\ndata={data}".format,
        }

        self.graphics_list = GraphicsList()

    def updateGraphics(self):
        """Update the GLGraphicsItems in this layer

        :returns: tuple (added_graphics, removed_graphics)
            - added_graphics - List of the added GLGraphicsItems
            - removed_graphics - List of the removed GLGraphicsItems
        
        """
        raise NotImplementedError("Abstract class method called: updateGraphics")

    def pointInScenePressed(self, event):
        """Event handler for the pointInScenePressed event
        
        :param event: The event
        
        """

    def pointInSceneDragged(self, event):
        """Event handler for the pointInSceneDragged event
        
        :param event: The event
        
        """

    def pointInSceneReleased(self, event):
        """Event handler for the pointInSceneReleased event
        
        :param event: The event
        
        """
