from pyqtgraph.Qt import QtGui
from pyqtgraph.opengl import *
from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem

from software.thunderscope.constants import Colors, LINE_WIDTH
from typing import Optional

class GLGraphic(GLLinePlotItem):
    """Abstract base class for 1D and 2D graphics on the cartesian plane (i.e. x-y plane)"""

    def __init__(
            self,
            parent_item: Optional[GLGraphicsItem] = None,
            outline_color: QtGui.QColor = Colors.DEFAULT_GRAPHICS_COLOR,
            line_width: float = LINE_WIDTH,
    ) -> None:
        """Initialize the GLGraphic

        :param parent_item: The parent item of the graphic
        :param outline_color: The color of the graphic's outline
        :param line_width: The line width of the graphic's outline
        """
        super().__init__(parentItem=parent_item, width=line_width)

        self.x = 0
        self.y = 0
        self.orientation = 0

        self.points = []
        self.set_outline_color(outline_color)

    def set_position(self, x: float, y: float) -> None:
        """Set the position of the graphic in the scene

        :param x: The x coordinate to position the graphic at
        :param y: The y coordinate to position the graphic at
        """
        if self.x == x and self.y == y:
            return

        self.translate(x - self.x, y - self.y, 0)
        self.x = x
        self.y = y

    def set_orientation(self, degrees: float) -> None:
        """Set the orientation of the graphic in the scene

        :param degrees: The orientation of the graphic in degrees
        """
        if self.orientation == degrees:
            return

        # Rotate locally about the z axis (0, 0, 1)
        self.rotate(degrees - self.orientation, 0, 0, 1, local=True)
        self.orientation = degrees

    def set_outline_color(self, outline_color: QtGui.QColor) -> None:
        """Set the color of the graphic's outline

        :param outline_color: The color of the shape's outline
        """
        self.setData(color=outline_color)

    def _update_shape_data(self) -> None:
        """Update the underlying GLLinePlotItem and GLMeshItem representing
        the outline and fill of this shape
        """
        raise NotImplementedError(
            "Derived class of GLGraphic must implement _update_shape_data"
        )
