from pyqtgraph.Qt import QtGui
from pyqtgraph.opengl import *
from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem

from software.py_constants import ROBOT_MAX_RADIUS_METERS
from software.thunderscope.constants import Colors, LINE_WIDTH

import math
import numpy as np


class GLRobotOutline(GLLinePlotItem):
    """Displays an outline of a robot parallel to the x-y plane"""

    def __init__(
        self,
        parentItem: GLGraphicsItem = None,
        color: QtGui.QColor = Colors.DEFAULT_GRAPHICS_COLOR,
        line_width: float = LINE_WIDTH,
    ):
        """Initialize the GLRobotOutline
        
        :param parentItem: The parent item of the graphic
        :param color: The color of the graphic
        :param line_width: The line width of the graphic

        """
        super().__init__(
            parentItem=parentItem,
            pos=np.array(GLRobotOutline.get_robot_outline()),
            color=color,
            width=line_width,
        )

        self.x = 0
        self.y = 0
        self.orientation = 0

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
        # We need to add 45 degrees to our desired orientation in order
        # to get the flat side of the robot (i.e. its front) to face
        # the right way
        degrees += 45

        if self.orientation == degrees:
            return

        # Rotate locally about the z axis (0, 0, 1)
        self.rotate(degrees - self.orientation, 0, 0, 1, local=True)
        self.orientation = degrees

    def set_color(self, color: QtGui.QColor):
        """Set the color of the graphic
        
        :param color: The color of the graphic
        
        """
        self.setData(color=color)
