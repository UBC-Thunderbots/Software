from pyqtgraph.Qt import QtGui
from pyqtgraph.opengl import *
from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem

from software.py_constants import ROBOT_MAX_RADIUS_METERS
from software.thunderscope.constants import Colors, LINE_WIDTH

from software.thunderscope.gl.graphics.gl_shape import GLShape
import software.thunderscope.gl.helpers.triangulate as triangulate

from typing import Optional

import math
import numpy as np


class GLRobotOutline(GLShape):
    """Displays an outline of a robot on the cartesian plane (i.e. x-y plane)"""

    def __init__(
        self,
        parent_item: Optional[GLGraphicsItem] = None,
        outline_color: QtGui.QColor = Colors.DEFAULT_GRAPHICS_COLOR,
        fill_color: Optional[QtGui.QColor] = None,
        line_width: float = LINE_WIDTH,
    ):
        """Initialize the GLRobotOutline
        
        :param parent_item: The parent item of the graphic
        :param color: The color of the graphic
        :param line_width: The line width of the graphic

        """
        super().__init__(
            parent_item=parent_item,
            outline_color=outline_color,
            fill_color=fill_color,
            line_width=line_width,
        )

    @staticmethod
    def get_robot_outline(z_coordinate: float = 0, num_points: int = 10):
        """Returns a list of points that represent the outline of a robot.
        The points will be on a plane parallel to the x-y plane.

        :param z_coordinate: The z coordinate of the plane to generate points on
        :param num_points: The number of points to generate
        :returns: A list of points representing the outline of a robot

        """
        # We compute points along 3/4 the circumference of a circle.
        # This is so that when we connect the points, there will be a chord
        # that slices the circle and produces a flat side (i.e the front of the bot)
        points = [
            [
                math.cos(1.5 * math.pi / num_points * x) * ROBOT_MAX_RADIUS_METERS,
                math.sin(1.5 * math.pi / num_points * x) * ROBOT_MAX_RADIUS_METERS,
                z_coordinate,
            ]
            for x in range(0, num_points + 1)
        ]

        # We need to repeat the first point at the end in order to close the polygon
        points = points + points[:1]

        return points

    def set_orientation(self, degrees: float):
        """Set the orientation of the graphic in the scene
        
        :param degrees: The orientation of the graphic in degrees

        """
        # We need to add 45 degrees to our desired orientation in order
        # to get the flat side of the robot (i.e. its front) to face
        # the right way
        super().set_orientation(degrees + 45)

    def _update_shape_data(self):
        """Update the underlying GLLinePlotItem and GLMeshItem representing
        the outline and fill of this shape
        """
        self.points = GLRobotOutline.get_robot_outline()
        self.setData(pos=self.points)

        if self.fill_graphic:
            faces = triangulate.earclip(self.points)
            meshdata = MeshData(vertexes=vertices, faces=np.array(faces))
            self.fill_graphic.setMeshData(meshdata=meshdata)
