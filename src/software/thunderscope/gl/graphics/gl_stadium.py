from pyqtgraph.Qt import QtGui
from pyqtgraph.opengl import *
from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem

from software.thunderscope.constants import Colors, LINE_WIDTH
from software.thunderscope.gl.graphics.gl_shape import GLShape
from proto.geometry_pb2 import Stadium

from typing import Optional

import math
import numpy as np


class GLStadium(GLShape):
    """Displays a stadium on the cartesian plane"""

    def __init__(
        self,
        parent_item: Optional[GLGraphicsItem] = None,
        radius: float = 1,
        length: float = 1,
        num_points: int = 12,
        outline_color: QtGui.QColor = Colors.DEFAULT_GRAPHICS_COLOR,
        fill_color: Optional[QtGui.QColor] = None,
        line_width: float = LINE_WIDTH,
    ) -> None:
        """Initialize the GLStadium

        :param parent_item: The parent item of the graphic
        :param radius: The radius of the stadium
        :param length: The length of the top and bottom lines of the stadium
        :param num_points: The number of points per semicircle to generate when creating the stadium
        :param outline_color: The color of the stadium's outline
        :param fill_color: The color used to fill the stadium, or None if no fill
        :param line_width: The line width of the stadium's outline

        """
        super().__init__(
            parent_item=parent_item,
            outline_color=outline_color,
            fill_color=fill_color,
            line_width=line_width,
        )

        self.radius = 0
        self.length = 0
        self.num_points = 0

        self.set_parameters(radius, length, num_points)

    def set_parameters(self, radius: float, length: float, num_points: int = 12):
        """Sets the parameters of the stadium

        :param radius: The radius of the stadium
        :param length: The length of the top and bottom lines of the stadium
        :param num_points: The number of points per semicircle to generate when drawing the stadium

        """

        if (
            self.radius == radius
            and self.length == length
            and self.num_points == num_points
        ):
            return

        self.radius = radius
        self.length = length
        self.num_points = num_points

        self._update_shape_data()

    def set_radius(self, radius: float, num_points: int = 12):
        """Sets the parameters of the stadium

        :param radius: The radius of the stadium
        :param num_points: The number of points per semicircle to generate when drawing the stadium

        """

        if self.radius == radius and self.num_points == num_points:
            return

        self.radius = radius
        self.num_points = num_points

        self._update_shape_data()

    def set_length(self, length: float, num_points: int = 12):
        """Sets the parameters of the stadium

        :param length: The length of the top and bottom lines of the stadium
        :param num_points: The number of points per semicircle to generate when drawing the stadium

        """

        if self.length == length and self.num_points == num_points:
            return

        self.length = length
        self.num_points = num_points

        self._update_shape_data()

    def update_from_stadium(self, stadium: Stadium):
        """Updates the stadium to match the parameters of another stadium

        :param stadium: The stadium to copy the parameters from

        """
        x_start_to_end = stadium.segment.end.x_meters - stadium.segment.start.x_meters
        y_start_to_end = stadium.segment.end.y_meters - stadium.segment.start.y_meters
        length = math.sqrt(math.pow(x_start_to_end, 2) + math.pow(y_start_to_end, 2))

        self.set_parameters(stadium.radius, length)
        # set stadium position to average of its two points
        self.set_position(
            (stadium.segment.end.x_meters + stadium.segment.start.x_meters) / 2,
            (stadium.segment.end.y_meters + stadium.segment.start.y_meters) / 2,
        )

        # set stadium orientation to angle between positive x and vector from start to end
        self.set_orientation(math.atan2(y_start_to_end, x_start_to_end) * 180 / math.pi)

    def _update_shape_data(self) -> None:
        """Update the underlying GLLinePlotItem and GLMeshItem representing
        the outline and fill of this shape
        """
        # Generate points on the circumference of a semicircle, moving them to the negative x by half the length
        self.points = (
            [
                [
                    math.cos(math.pi / self.num_points * x + math.pi / 2) * self.radius
                    - self.length / 2,
                    math.sin(math.pi / self.num_points * x + math.pi / 2) * self.radius,
                    0,
                ]
                for x in range(0, self.num_points + 1)
            ]
            + [
                [
                    math.cos(math.pi / self.num_points * x + 3 * math.pi / 2)
                    * self.radius
                    + self.length / 2,
                    math.sin(math.pi / self.num_points * x + 3 * math.pi / 2)
                    * self.radius,
                    0,
                ]
                for x in range(0, self.num_points + 1)
            ]
            + [[-self.length / 2, self.radius, 0]]
        )

        self.setData(pos=self.points)

        if self.fill_graphic:
            # Add an extra point at (0, 0). This is so that we can construct
            # the triangular faces that make up the circle by connecting
            # two adjacent points along the circle to the center point
            vertexes = self.points + [(0, 0, 0)]

            faces = []
            for index in range(len(vertexes) - 2):
                faces.append([index, index + 1, len(vertexes) - 1])

            mesh_data = MeshData(vertexes=vertexes, faces=np.array(faces))
            self.fill_graphic.setMeshData(meshdata=mesh_data)
