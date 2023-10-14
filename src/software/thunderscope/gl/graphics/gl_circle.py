from pyqtgraph.Qt import QtGui
from pyqtgraph.opengl import *
from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem

from software.thunderscope.constants import Colors, LINE_WIDTH

from typing import Optional

import math
import numpy as np


class GLCircle(GLLinePlotItem):
    """Displays a circle parallel to the x-y plane"""

    def __init__(
        self,
        parentItem: Optional[GLGraphicsItem] = None,
        radius: float = 1,
        num_points: int = 24,
        outline_color: QtGui.QColor = Colors.DEFAULT_GRAPHICS_COLOR,
        fill_color: Optional[QtGui.QColor] = None,
        line_width: float = LINE_WIDTH,
    ):
        """Initialize the GLCircle
        
        :param parentItem: The parent item of the graphic
        :param radius: The radius of the circle
        :param num_points: The number of points to generate when creating the circle
        :param outline_color: The color of the circle's outline
        :param fill_color: The color used to fill the circle, or None if no fill
        :param line_width: The line width of the circle's outline

        """
        super().__init__(parentItem=parentItem, width=line_width)

        self.x = 0
        self.y = 0
        self.radius = 0
        self.num_points = 0

        self.fill_graphic = None

        self.set_radius(radius, num_points)
        self.set_outline_color(outline_color)
        self.set_fill_color(fill_color)

    def set_radius(self, radius: float, num_points: int = 24):
        """Set the radius of the circle

        :param radius: The radius of the circle
        :param num_points: The number of points to generate when creating the circle
        
        """
        if self.radius == radius:
            return

        self.radius = radius
        self.num_points = num_points

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
        """Set the color of the circle's outline
        
        :param outline_color: The color of the circle's outline 
        
        """
        self.setData(color=outline_color)

    def set_fill_color(self, fill_color: Optional[QtGui.QColor]):
        """Set the color used to fill the circle
        
        :param fill_color: The color used to fill the circle, or None if no fill 
        
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
        # Generate points on the circumference of a circle centered at (0,0)
        points = [
            [
                math.cos(2 * math.pi / self.num_points * x) * self.radius,
                math.sin(2 * math.pi / self.num_points * x) * self.radius,
                0,
            ]
            for x in range(0, self.num_points + 1)
        ]

        self.setData(pos=points)

        if self.fill_graphic:
            # Add an extra point at (0, 0). This is so that we can construct
            # the triangular faces that make up the circle by connecting
            # two adjacent points along the circle to the center point
            vertexes = points + [(0, 0, 0)]

            faces = []
            for index in range(len(vertexes) - 2):
                faces.append([index, index + 1, len(vertexes) - 1])

            meshdata = MeshData(vertexes=vertexes, faces=np.array(faces))
            self.fill_graphic.setMeshData(meshdata=meshdata)
