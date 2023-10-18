from pyqtgraph.Qt import QtGui
from pyqtgraph.opengl import *
from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem

from software.thunderscope.constants import Colors, LINE_WIDTH
from software.thunderscope.gl.graphics.gl_shape import GLShape

from typing import Optional

import numpy as np


class GLRect(GLShape):
    """Displays a rectangle on the cartesian plane (i.e. x-y plane)"""

    def __init__(
        self,
        parent_item: Optional[GLGraphicsItem] = None,
        x_length: float = 0,
        y_length: float = 0,
        outline_color: QtGui.QColor = Colors.DEFAULT_GRAPHICS_COLOR,
        fill_color: Optional[QtGui.QColor] = None,
        line_width: float = LINE_WIDTH,
    ) -> None:
        """Initialize the GLRect
        
        :param parent_item: The parent item of the graphic
        :param x_length: The length of the rectangle in the x direction
        :param y_length: The length of the rectangle in the y direction
        :param outline_color: The color of the rectangle's outline
        :param fill_color: The color used to fill the rectangle, or None if no fill
        :param line_width: The line width of the rectangle's outline

        """
        super().__init__(
            parent_item=parent_item,
            outline_color=outline_color,
            fill_color=fill_color,
            line_width=line_width,
        )

        self.x_length = 0
        self.y_length = 0
        self.set_dimensions(x_length, y_length)

    def set_dimensions(self, x_length: float = 0, y_length: float = 0) -> None:
        """Set the dimensions of the rectangle
        
        :param x_length: The length of the rectangle in the x direction
        :param y_length: The length of the rectangle in the y direction

        """
        if x_length == 0 or y_length == 0:
            return

        if self.x_length == x_length and self.y_length == y_length:
            return

        self.x_length = x_length
        self.y_length = y_length

        self._update_shape_data()

    def _update_shape_data(self) -> None:
        """Update the underlying GLLinePlotItem and GLMeshItem representing
        the outline and fill of this shape
        """
        self.points = [
            [-self.x_length / 2, self.y_length / 2, 0],
            [self.x_length / 2, self.y_length / 2, 0],
            [self.x_length / 2, -self.y_length / 2, 0],
            [-self.x_length / 2, -self.y_length / 2, 0],
            [-self.x_length / 2, self.y_length / 2, 0],
        ]

        self.setData(pos=self.points)

        if self.fill_graphic:
            faces = [[0, 1, 2], [2, 3, 4]]
            meshdata = MeshData(vertexes=self.points, faces=np.array(faces))
            self.fill_graphic.setMeshData(meshdata=meshdata)
