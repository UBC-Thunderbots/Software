from pyqtgraph.Qt import QtGui
from pyqtgraph.opengl import *
from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem

from software.thunderscope.constants import Colors, LINE_WIDTH
from software.thunderscope.gl.graphics.gl_shape import GLShape

from typing import Optional

import math
import numpy as np


class GLCircle(GLShape):
    """Displays a circle on the cartesian plane (i.e. x-y plane)"""

    def __init__(
        self,
        parent_item: Optional[GLGraphicsItem] = None,
        radius: float = 1,
        num_points: int = 24,
        outline_color: QtGui.QColor = Colors.DEFAULT_GRAPHICS_COLOR,
        fill_color: Optional[QtGui.QColor] = None,
        line_width: float = LINE_WIDTH,
    ) -> None:
        """Initialize the GLCircle
        
        :param parent_item: The parent item of the graphic
        :param radius: The radius of the circle
        :param num_points: The number of points to generate when creating the circle
        :param outline_color: The color of the circle's outline
        :param fill_color: The color used to fill the circle, or None if no fill
        :param line_width: The line width of the circle's outline

        """
        super().__init__(
            parent_item=parent_item,
            outline_color=outline_color,
            fill_color=fill_color,
            line_width=line_width,
        )

        self.radius = 0
        self.num_points = 0

        self.set_radius(radius, num_points)

    def set_radius(self, radius: float, num_points: int = 24) -> None:
        """Set the radius of the circle

        :param radius: The radius of the circle
        :param num_points: The number of points to generate when creating the circle
        
        """
        if self.radius == radius:
            return

        self.radius = radius
        self.num_points = num_points

        self._update_shape_data()

    def _update_shape_data(self) -> None:
        """Update the underlying GLLinePlotItem and GLMeshItem representing
        the outline and fill of this shape
        """
        # Generate points on the circumference of a circle centered at (0,0)
        self.points = [
            [
                math.cos(2 * math.pi / self.num_points * x) * self.radius,
                math.sin(2 * math.pi / self.num_points * x) * self.radius,
                0,
            ]
            for x in range(0, self.num_points + 1)
        ]

        self.setData(pos=self.points)

        if self.fill_graphic:
            # Add an extra point at (0, 0). This is so that we can construct
            # the triangular faces that make up the circle by connecting
            # two adjacent points along the circle to the center point
            vertexes = self.points + [(0, 0, 0)]

            faces = []
            for index in range(len(vertexes) - 2):
                faces.append([index, index + 1, len(vertexes) - 1])

            meshdata = MeshData(vertexes=vertexes, faces=np.array(faces))
            self.fill_graphic.setMeshData(meshdata=meshdata)
