from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.opengl import *

from software.py_constants import *

import math
import numpy as np

class GLRobot(GLMeshItem):

    def __init__(self, color):

        self.x = 0
        self.y = 0
        self.color = color
        self.num_points = 10

        GLMeshItem.__init__(
            self,
            meshdata=self.getMeshData(),
            color=self.color,
        )

    def getMeshData(self):
        """
        Return a MeshData instance with vertexes and faces computed
        for the surface of an extruded chord.

        Not implemented yet
        """

        pi = math.pi
        top_circle_points = [
            [
                math.cos(2 * pi / self.num_points * x) * ROBOT_MAX_RADIUS_METERS, 
                math.sin(2 * pi / self.num_points * x) * ROBOT_MAX_RADIUS_METERS,
                ROBOT_MAX_HEIGHT_METERS
            ] 
            for x in range(0, self.num_points + 1)
        ]
        bottom_circle_points = [
            [
                math.cos(2 * pi / self.num_points * x) * ROBOT_MAX_RADIUS_METERS, 
                math.sin(2 * pi / self.num_points * x) * ROBOT_MAX_RADIUS_METERS,
                0
            ] 
            for x in range(0, self.num_points + 1)
        ]
        circle_points = top_circle_points + bottom_circle_points
        points = circle_points + [[0, 0, ROBOT_MAX_HEIGHT_METERS]]

        side_faces = []
        for index in range(len(top_circle_points) - 1):
            side_faces.append([index, index + 1, index + len(top_circle_points)])
        for index in range(len(top_circle_points), len(circle_points) - 1):
            side_faces.append([index, index + 1, index + 1 - len(top_circle_points)])

        top_faces = []
        for index in range(len(top_circle_points) - 1):
            side_faces.append([index, index + 1, len(circle_points)])

        faces = side_faces

        return MeshData(
            vertexes=np.array(points),
            faces=np.array(faces),
        )

    def setPosition(self, x, y):

        if self.x == x and self.y == y:
            return

        self.translate(-self.x, -self.y, 0)

        self.x = x 
        self.y = y
        self.translate(self.x, self.y, 0)

        