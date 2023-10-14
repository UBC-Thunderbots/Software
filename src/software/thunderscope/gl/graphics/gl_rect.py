from pyqtgraph.Qt import QtGui
from pyqtgraph.opengl import *
from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem

from software.thunderscope.constants import Colors, LINE_WIDTH

import numpy as np


class GLRect(GLLinePlotItem):
    """Displays a rectangle parallel to the x-y plane"""

    def __init__(
        self,
        parentItem: GLGraphicsItem = None,
        color: QtGui.QColor = Colors.DEFAULT_GRAPHICS_COLOR,
        line_width: float = LINE_WIDTH,
    ):
        """Initialize the GLRect
        
        :param parentItem: The parent item of the graphic
        :param color: The color of the graphic
        :param line_width: The line width of the graphic

        """
        super().__init__(parentItem=parentItem, color=color, width=line_width)

        self.x = 0
        self.y = 0
        self.x_length = 0
        self.y_length = 0

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

        self.setData(
            pos=np.array(
                [
                    [-x_length / 2, y_length / 2, 0],
                    [x_length / 2, y_length / 2, 0],
                    [x_length / 2, -y_length / 2, 0],
                    [-x_length / 2, -y_length / 2, 0],
                    [-x_length / 2, y_length / 2, 0],
                ]
            )
        )

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

    def set_color(self, color: QtGui.QColor):
        """Set the color of the graphic
        
        :param color: The color of the graphic
        
        """
        self.setData(color=color)
