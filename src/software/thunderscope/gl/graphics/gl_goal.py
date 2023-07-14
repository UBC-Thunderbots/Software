from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.opengl import *

from software.py_constants import *

import math
import numpy as np


class GLGoal(GLMeshItem):
    """Displays a 3D mesh representing the goal"""

    def __init__(self, color=(1.0, 1.0, 1.0, 0.5)):
        """Initialize the GLGoal
        
        :param color: The color of the graphic

        """
        self.x = 0
        self.y = 0
        self.x_length = 0
        self.y_length = 0
        self.orientation = 0
        self.color = color

        GLMeshItem.__init__(self, color=self.color)

    def set_dimensions(self, x_length, y_length):
        """Set the dimensions of the goal
        
        :param x_length: The length of the goal in the x direction
        :param y_length: The length of the goal in the y direction

        """
        if self.x_length == x_length and self.y_length == y_length:
            return

        self.x_length = x_length
        self.y_length = y_length

        self.setMeshData(meshdata=self.getMeshData(self.x_length, self.y_length))

    def getMeshData(self, x_length, y_length):
        """
        Return a MeshData instance with vertices and faces computed
        for a mesh representing the goal

        :param x_length: The length of the goal in the x direction
        :param y_length: The length of the goal in the y direction
        :returns: the computed MeshData instance 

        """
        # Construct points that make up the mesh.
        back_plate_points = [
            [-x_length, y_length / 2, 0],
            [-x_length, -y_length / 2, 0],
            [-x_length, y_length / 2, ROBOT_MAX_HEIGHT_METERS],
            [-x_length, -y_length / 2, ROBOT_MAX_HEIGHT_METERS],
        ]
        left_plate_points = [
            [-x_length, y_length / 2, 0],
            [0, y_length / 2, 0],
            [-x_length, y_length / 2, ROBOT_MAX_HEIGHT_METERS],
            [0, y_length / 2, ROBOT_MAX_HEIGHT_METERS],
        ]
        right_plate_points = [
            [-x_length, -y_length / 2, 0],
            [0, -y_length / 2, 0],
            [-x_length, -y_length / 2, ROBOT_MAX_HEIGHT_METERS],
            [0, -y_length / 2, ROBOT_MAX_HEIGHT_METERS],
        ]
        points = back_plate_points + left_plate_points + right_plate_points

        # Construct triangular faces that make up the mesh.
        # Each face is an array of 3 indices in the points array.
        faces = [
            # Back plate faces
            [0, 1, 2],
            [1, 2, 3],
            # Left plate faces
            [4, 5, 6],
            [5, 6, 7],
            # Right plate faces
            [8, 9, 10],
            [9, 10, 11]
        ]

        return MeshData(vertexes=np.array(points), faces=np.array(faces),)

    def set_position(self, x, y):
        """Set the position of the graphic in the scene
        
        :param x: The x coordinate to position the graphic at
        :param y: The y coordinate to position the graphic at
        
        """
        if self.x == x and self.y == y:
            return

        self.translate(x - self.x, y - self.y, 0)
        self.x = x
        self.y = y

    def set_orientation(self, degrees):
        """Set the orientation of the graphic in the scene
        
        :param degrees: The orientation of the graphic in degrees

        """
        if self.orientation == degrees:
            return

        # Rotate locally about the z axis (0, 0, 1)
        self.rotate(degrees - self.orientation, 0, 0, 1, local=True)
        self.orientation = degrees
