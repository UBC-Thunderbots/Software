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

    def update_graphics(self):
        """Update the GLGraphicsItems in this layer

        :returns: tuple (added_graphics, removed_graphics)
            - added_graphics - List of the added GLGraphicsItems
            - removed_graphics - List of the removed GLGraphicsItems
        
        """
        raise NotImplementedError("Abstract class method called: update_graphics")

    def point_in_scene_pressed(self, event):
        """Event handler for the point_in_scene_pressed event
        
        :param event: The event
        
        """

    def point_in_scene_dragged(self, event):
        """Event handler for the point_in_scene_dragged event
        
        :param event: The event
        
        """

    def point_in_scene_released(self, event):
        """Event handler for the point_in_scene_released event
        
        :param event: The event
        
        """
