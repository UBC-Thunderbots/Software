from pyqtgraph.Qt import QtGui
from pyqtgraph.opengl import *
from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem

from software.thunderscope.constants import Colors, LINE_WIDTH

import math
import numpy as np


class GLCircle(GLLinePlotItem):
    """Displays a circle parallel to the x-y plane"""

    def __init__(
        self,
        parentItem: GLGraphicsItem = None,
        radius: float = 1,
        num_points: int = 24,
        color: QtGui.QColor = Colors.DEFAULT_GRAPHICS_COLOR,
        line_width: float = LINE_WIDTH,
    ):
        """Initialize the GLCircle
        
        :param parentItem: The parent item of the graphic
        :param radius: The radius of the circle
        :param num_points: The number of points to generate when creating the circle
        :param color: The color of the graphic
        :param line_width: The line width of the graphic

        """
        super().__init__(parentItem=parentItem, color=color, width=line_width)

        self.x = 0
        self.y = 0
        self.radius = 0
        self.num_points = 0
        self.set_radius(radius, num_points)

    def set_radius(self, radius: float, num_points: int = 24):
        """Set the radius of the circle

        :param radius: The radius of the circle
        :param num_points: The number of points to generate when creating the circle
        
        """
        if self.radius == radius:
            return

        self.radius = radius
        self.num_points = num_points

        # Generate points on the circumference of a circle centered at (0,0)
        pi = math.pi
        points = np.array(
            [
                [
                    math.cos(2 * pi / self.num_points * x) * self.radius,
                    math.sin(2 * pi / self.num_points * x) * self.radius,
                    0,
                ]
                for x in range(0, self.num_points + 1)
            ]
        )
        self.setData(pos=points)

    def set_color(self, color: QtGui.QColor):
        """Set the color of the graphic
        
        :param color: The color of the graphic
        
        """
        self.setData(color=color)

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
