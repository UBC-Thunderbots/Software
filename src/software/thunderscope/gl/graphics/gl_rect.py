from pyqtgraph.Qt import QtGui
from pyqtgraph.opengl import *
from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem

from software.thunderscope.constants import Colors, LINE_WIDTH

from typing import Optional

import numpy as np


class GLRect(GLLinePlotItem):
    """Displays a rectangle parallel to the x-y plane"""

    def __init__(
        self,
        parentItem: Optional[GLGraphicsItem] = None,
        x_length: float = 0,
        y_length: float = 0,
        outline_color: QtGui.QColor = Colors.DEFAULT_GRAPHICS_COLOR,
        fill_color: Optional[QtGui.QColor] = None,
        line_width: float = LINE_WIDTH,
    ):
        """Initialize the GLRect
        
        :param parentItem: The parent item of the graphic
        :param x_length: The length of the rectangle in the x direction
        :param y_length: The length of the rectangle in the y direction
        :param outline_color: The color of the rectangle's outline
        :param fill_color: The color used to fill the rectangle, or None if no fill
        :param line_width: The line width of the rectangle's outline

        """
        super().__init__(parentItem=parentItem, width=line_width)

        self.x = 0
        self.y = 0
        self.x_length = 0
        self.y_length = 0

        self.fill_graphic = None

        self.set_dimensions(x_length, y_length)
        self.set_outline_color(outline_color)
        self.set_fill_color(fill_color)

    def set_dimensions(self, x_length: float = 0, y_length: float = 0):
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

        self.__update_data()

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

    def set_outline_color(self, outline_color: QtGui.QColor):
        """Set the color of the rectangle's outline
        
        :param outline_color: The color of the rectangle's outline 
        
        """
        self.setData(color=outline_color)

    def set_fill_color(self, fill_color: Optional[QtGui.QColor]):
        """Set the color used to fill the rectangle
        
        :param fill_color: The color used to fill the rectangle, or None if no fill 
        
        """
        if fill_color:
            if not self.fill_graphic:
                self.fill_graphic = GLMeshItem(parentItem=self)
                self.__update_data()
            self.fill_graphic.setColor(fill_color)
        else:
            if self.fill_graphic:
                self.fill_graphic.setParentItem(None)
                self.fill_graphic = None

    def __update_data(self):
        """Update the vertex and mesh data of the underlying GLLinePlotItem and 
        GLMeshData graphics that make up the circle graphic
        """
        points = [
            [-self.x_length / 2, self.y_length / 2, 0],
            [self.x_length / 2, self.y_length / 2, 0],
            [self.x_length / 2, -self.y_length / 2, 0],
            [-self.x_length / 2, -self.y_length / 2, 0],
            [-self.x_length / 2, self.y_length / 2, 0],
        ]

        self.setData(pos=points)

        if self.fill_graphic:
            faces = [[0, 1, 2], [2, 3, 4]]
            meshdata = MeshData(vertexes=points, faces=np.array(faces))
            self.fill_graphic.setMeshData(meshdata=meshdata)
