from pyqtgraph.Qt import QtGui
import pyqtgraph as pg
from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem


from software.thunderscope.gl.helpers.observable_list import Change, ChangeAction
from software.thunderscope.gl.helpers.extended_gl_view_widget import MouseInSceneEvent


class GLLayer(GLGraphicsItem):
    """Represents a layer in the 3D visualization. 
    
    A layer is added to the 3D scenegraph and represents a collection of
    GLGraphicsItems to be displayed together. GLGraphicsItems should be 
    added as children of a GLLayer.
    
    """

    def __init__(self, name: str) -> None:
        """Initialize the GLLayer
        
        :param name: The displayed name of the layer

        """
        super().__init__()
        self.name = name

        # GLLayers can point to one another with this field, forming a
        # linked list of "related" GLLayers.
        #
        # Related layers are grouped together and treated as a single layer
        # in the layer menu. This lets us treat multiple layers rendered at
        # different depths as one unit and toggle their visibility together
        # as a whole.
        #
        # WARNING: Related GLLayers should not be parents/children of each other
        self.related_layer: GLLayer = None

    def refresh_graphics(self) -> None:
        """Updates the GLGraphicsItems in this layer"""

    def keyPressEvent(self, event: QtGui.QKeyEvent) -> None:
        """Detect when a key has been pressed

        :param event: The event

        """

    def keyReleaseEvent(self, event: QtGui.QKeyEvent) -> None:
        """Detect when a key has been released

        :param event: The event

        """

    def mouse_in_scene_pressed(self, event: MouseInSceneEvent) -> None:
        """Detect that the mouse was pressed and picked a point in the 3D scene
        
        :param event: The event
        
        """

    def mouse_in_scene_dragged(self, event: MouseInSceneEvent) -> None:
        """Detect that the mouse was dragged within the 3D scene
        
        :param event: The event
        
        """

    def mouse_in_scene_released(self, event: MouseInSceneEvent) -> None:
        """Detect that the mouse was released after picking a point in the 3D scene
        
        :param event: The event
        
        """

    def mouse_in_scene_moved(self, event: MouseInSceneEvent) -> None:
        """Detect that the mouse was moved within the 3D scene
        
        :param event: The event
        
        """

    def _graphics_changed(self, change: Change) -> None:
        """Called by an ObservableList of GLGraphicsItems to notify 
        this observer that GLGraphicsItems were added or removed from
        the ObservableList.

        Subclasses of GLLayer should use ObservableLists to store collections of
        GLGraphicsItems. Each ObservableList should register this function as an observer 
        so that GLGraphicsItems are automatically added/removed as children of this layer
        (and thus added/removed to the 3D scenegraph) when they are added/removed from 
        the ObservableList.

        :param change: Change representing the GLGraphicsItems added or removed

        """
        if change.action == ChangeAction.ADD:
            for graphic in change.elements:
                # Manually setting the parent of the GLGraphicsItem to this
                # layer since GLGraphicsItem.setParentItem does not work
                # correctly
                self._GLGraphicsItem__children.add(graphic)
                graphic._GLGraphicsItem__parent = self
                self.view().addItem(graphic)

        elif change.action == ChangeAction.REMOVE:
            for graphic in change.elements:
                # Manually removing the GLGraphicsItem as a child of this
                # layer since GLGraphicsItem.setParentItem does not work
                # correctly
                self._GLGraphicsItem__children.remove(graphic)
                graphic._GLGraphicsItem__parent = None
                self.view().removeItem(graphic)
