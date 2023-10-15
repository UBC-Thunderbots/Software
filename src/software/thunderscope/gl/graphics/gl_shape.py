from pyqtgraph.Qt import QtGui
from pyqtgraph.opengl import *
from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem

from software.thunderscope.constants import Colors, LINE_WIDTH

from typing import Optional

import numpy as np
import software.thunderscope.gl.helpers.triangulate as triangulate


class GLShape(GLLinePlotItem):
    """Base class for a graphic that displays a shape on the 
    cartesian plane (i.e. x-y plane)
    """

    def __init__(
        self,
        parent_item: Optional[GLGraphicsItem] = None,
        outline_color: QtGui.QColor = Colors.DEFAULT_GRAPHICS_COLOR,
        fill_color: Optional[QtGui.QColor] = None,
        line_width: float = LINE_WIDTH,
    ):
        """Base constructor for the GLShape
        
        :param parent_item: The parent item of the graphic
        :param outline_color: The color of the polygon's outline
        :param fill_color: The color used to fill the polygon, or None if no fill
        :param line_width: The line width of the polygon's outline

        """
        super().__init__(parentItem=parent_item, width=line_width)

        self.x = 0
        self.y = 0
        self.orientation = 0

        self.points = []
        self.fill_graphic = None

        self.set_outline_color(outline_color)
        self.set_fill_color(fill_color)

    def set_position(self, x: float, y: float):
        """Set the position of the graphic in the scene
        
        :param x: The x coordinate to position the graphic at
        :param y: The y coordinate to position the graphic at
        
        """
        if self.x == x and self.y == y:
            return

        self.translate(x - self.x, y - self.y, 0)
        self.x = x
        self.y = y

    def set_orientation(self, degrees: float):
        """Set the orientation of the graphic in the scene
        
        :param degrees: The orientation of the graphic in degrees

        """
        if self.orientation == degrees:
            return

        # Rotate locally about the z axis (0, 0, 1)
        self.rotate(degrees - self.orientation, 0, 0, 1, local=True)
        self.orientation = degrees

    def set_outline_color(self, outline_color: QtGui.QColor):
        """Set the color of the shape's outline
        
        :param outline_color: The color of the shape's outline 
        
        """
        self.setData(color=outline_color)

    def set_fill_color(self, fill_color: Optional[QtGui.QColor]):
        """Set the color used to fill the shape
        
        :param fill_color: The color used to fill the shape, or None if no fill 
        
        """
        if fill_color:
            if not self.fill_graphic:
                self.fill_graphic = GLMeshItem(parentItem=self)
                self._update_shape_data()
            self.fill_graphic.setColor(fill_color)
        else:
            if self.fill_graphic:
                self.fill_graphic.setParentItem(None)
                self.fill_graphic = None

    def _update_shape_data(self):
        """Update the underlying GLLinePlotItem and GLMeshItem representing
        the outline and fill of this shape
        """
        raise NotImplementedError(
            "Derived class of GLShape must implement _update_shape_data"
        )
