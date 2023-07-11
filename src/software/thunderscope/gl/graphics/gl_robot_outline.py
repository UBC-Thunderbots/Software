from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.opengl import *

from software.py_constants import *

import math
import numpy as np


class GLRobotOutline(GLLinePlotItem):
    """Displays an outline of a robot on the x-y plane"""

    def __init__(self, color):

        self.x = 0
        self.y = 0
        self.orientation = 0
        self.color = color

        GLLinePlotItem.__init__(
            self, pos=np.array(GLRobotOutline.getRobotOutline()), color=color
        )

    @staticmethod
    def getRobotOutline(z_coordinate=0, num_points=10):
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

    def setPosition(self, x, y):

        if self.x == x and self.y == y:
            return

        self.translate(x - self.x, y - self.y, 0)
        self.x = x
        self.y = y

    def setOrientation(self, radians):

        # We need to add 45 degrees to our desired orientation in order
        # to get the flat side of the robot (i.e. its front) to face
        # the right way
        degrees = math.degrees(radians) + 45

        if self.orientation == degrees:
            return

        # Rotate locally about the z axis (0, 0, 1)
        self.rotate(degrees - self.orientation, 0, 0, 1, local=True)
        self.orientation = degrees
