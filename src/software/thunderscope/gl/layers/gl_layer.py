from pyqtgraph.Qt import QtGui
from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem

from software.thunderscope.gl.helpers.observable_list import Change, ChangeAction
from software.thunderscope.gl.helpers.extended_gl_view_widget import MouseInSceneEvent
from software.thunderscope.gl.helpers.gl_patches import *


class GLLayer(GLGraphicsItem):
    """Represents a layer in the 3D visualization.

    A layer is added to the 3D scenegraph and represents a collection of
    GLGraphicsItems to be displayed together. GLGraphicsItems should be
    added as children of a GLLayer.

    GLLayers themselves can also be added as a children of a GLLayer,
    enabling us to group together related layers.
    """

    def __init__(self, name: str, parent_item: GLGraphicsItem | None = None) -> None:
        """Initialize the GLLayer

        :param name: The displayed name of the layer
        :param parent_item: The parent GLGraphicsItem of the GLLayer
        """
        super().__init__(parent_item)
        self.name = name

    def refresh_graphics(self) -> None:
        """Updates the GLGraphicsItems in this layer"""
        raise NotImplementedError("Subclasses must implement this method!")

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
                graphic.setParentItem(self)

        elif change.action == ChangeAction.REMOVE:
            for graphic in change.elements:
                graphic.setParentItem(None)
