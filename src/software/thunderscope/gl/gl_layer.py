import pyqtgraph as pg
from pyqtgraph.opengl import *

from software.thunderscope.gl.helpers.graphics_list import GraphicsList
from software.thunderscope.gl.helpers.extended_gl_view_widget import PointInSceneEvent


class GLLayer(pg.GraphicsObject):
    """Represents a layer in the 3D visualization. A layer manages and returns GLGraphicsItem 
    graphics to be displayed in the 3D scene.
    """

    def __init__(self, name: str):
        """Initialize the GLLayer
        
        :param name: The displayed name of the layer

        """
        pg.GraphicsObject.__init__(self)

        self.name = name
        self.graphics_list = GraphicsList()

    def refresh_graphics(self):
        """Updates the GLGraphicsItems in this layer

        :returns: tuple (added_graphics, removed_graphics)
            - added_graphics - List of the added GLGraphicsItems
            - removed_graphics - List of the removed GLGraphicsItems
        
        """
        if self.isVisible():
            self._update_graphics()

        return self.graphics_list.get_changes()

    def mouse_in_scene_pressed(self, event: PointInSceneEvent):
        """Event handler for the mouse_in_scene_pressed event
        
        :param event: The event
        
        """

    def mouse_in_scene_dragged(self, event: PointInSceneEvent):
        """Event handler for the mouse_in_scene_dragged event
        
        :param event: The event
        
        """

    def mouse_in_scene_released(self, event: PointInSceneEvent):
        """Event handler for the mouse_in_scene_released event
        
        :param event: The event
        
        """

    def mouse_in_scene_moved(self, event: PointInSceneEvent):
        """Event handler for the mouse_in_scene_moved event
        
        :param event: The event
        
        """

    def _update_graphics(self):
        """Protected method that should overridden to fetch and update
        graphics for the layer. 
        
        Subclasses of GLLayer should fetch graphics from self.graphics_list
        and only update the returned graphics from GraphicsList collection.
        
        self.graphics_list.get_changes() should NOT be called in this function.

        """
        raise NotImplementedError("Abstract class method called: _update_graphics")
