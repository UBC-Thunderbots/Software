from pyqtgraph.Qt import QtGui
import pyqtgraph as pg
from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem

from typing import Callable, List

from software.thunderscope.gl.helpers.observable_list import Change, ChangeAction
from software.thunderscope.gl.helpers.extended_gl_view_widget import MouseInSceneEvent


class GLLayer(GLGraphicsItem):
    """Represents a layer in the 3D visualization. 
    
    A layer is added to the 3D scenegraph and represents a collection of
    GLGraphicsItems that we wish to display together. GLGraphicsItems should
    be added as children of a GLLayer (using GLGraphicsItem.setParentItem)
    
    """

    def __init__(self, name: str):
        """Initialize the GLLayer
        
        :param name: The displayed name of the layer

        """
        GLGraphicsItem.__init__(self)
        self.name = name

    def refresh_graphics(self):
        """Updates the GLGraphicsItems in this layer"""

    def keyPressEvent(self, event: QtGui.QKeyEvent):
        """Detect when a key has been pressed

        :param event: The event

        """

    def keyReleaseEvent(self, event: QtGui.QKeyEvent):
        """Detect when a key has been released

        :param event: The event

        """

    def mouse_in_scene_pressed(self, event: MouseInSceneEvent):
        """Detect that the mouse was pressed and picked a point in the 3D scene
        
        :param event: The event
        
        """

    def mouse_in_scene_dragged(self, event: MouseInSceneEvent):
        """Detect that the mouse was dragged within the 3D scene
        
        :param event: The event
        
        """

    def mouse_in_scene_released(self, event: MouseInSceneEvent):
        """Detect that the mouse was released after picking a point in the 3D scene
        
        :param event: The event
        
        """

    def mouse_in_scene_moved(self, event: MouseInSceneEvent):
        """Detect that the mouse was moved within the 3D scene
        
        :param event: The event
        
        """

    def _graphics_changed(self, change: Change):
        """Called by an ObservableList of GLGraphicsItems to notify 
        this observer that GLGraphicsItems were added or removed from
        the ObservableList.

        Subclasses of GLLayer should use ObservableLists to store collections of
        GLGraphicsItems. Each ObservableList should register this function as an observer 
        so that GLGraphicsItems are automatically added/removed to the 3D scenegraph
        when they are added/removed from the ObservableList.

        :param change: Change representing the GLGraphicsItems added or removed

        """
        if change.action == ChangeAction.ADD:
            for graphic in change.elements:
                graphic.setParentItem(self)
        elif change.action == ChangeAction.REMOVE:
            for graphic in change.elements:
                graphic.setParentItem(None)

    def _bring_list_to_length(
        self, list_: List, target_len: int, element_factory: Callable
    ):
        """Bring a list to a desired target length by either creating new elements
        using the provided `element_factory` and adding them to the list, or by 
        popping elements from the end of the list.

        Subclasses of GLLayer using ObservableLists for storing GLGraphicsItems may
        find this helper function useful.

        :param list_: The list
        :param target_len: The target length to bring the list to
        :param element_factory: Callable that returns an element to add to the list

        """
        while len(list_) > target_len:
            list_.pop()
        while len(list_) < target_len:
            list_.append(element_factory())
