import pyqtgraph as pg

from software.thunderscope.gl.helpers.graphics_list import GraphicsList


class GLLayer(pg.GraphicsObject):
    """Represents a layer in the 3D visualization. A layer manages and returns GLGraphicsItem 
    graphics to be displayed in the 3D scene.
    """

    def __init__(self):
        """Initialize the GLLayer"""
        pg.GraphicsObject.__init__(self)

        self.name = ""
        self.graphics_list = GraphicsList()

    def set_name(self, name: str):
        """Sets the displayed name of the layer
        
        :param name: The name of the layer

        """
        self.name = name

    def update_graphics(self):
        """Update the GLGraphicsItems in this layer

        :returns: tuple (added_graphics, removed_graphics)
            - added_graphics - List of the added GLGraphicsItems
            - removed_graphics - List of the removed GLGraphicsItems
        
        """
        raise NotImplementedError("Abstract class method called: update_graphics")

    def mouse_in_scene_pressed(self, event):
        """Event handler for the mouse_in_scene_pressed event
        
        :param event: The event
        
        """

    def mouse_in_scene_dragged(self, event):
        """Event handler for the mouse_in_scene_dragged event
        
        :param event: The event
        
        """

    def mouse_in_scene_released(self, event):
        """Event handler for the mouse_in_scene_released event
        
        :param event: The event
        
        """

    def mouse_in_scene_moved(self, event):
        """Event handler for the mouse_in_scene_moved event
        
        :param event: The event
        
        """
