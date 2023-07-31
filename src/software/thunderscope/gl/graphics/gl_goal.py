from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.opengl import *

from software.py_constants import *

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

        # The 3D mesh isn't visible from the orthographic view, so
        # we need to draw an outline of the goal on the ground 
        self.goal_outline = GLLinePlotItem(color=self.color)
        self.goal_outline.setParentItem(self)

    def set_dimensions(self, x_length: float, y_length: float):
        """Set the dimensions of the goal
        
        :param x_length: The length of the goal in the x direction
        :param y_length: The length of the goal in the y direction

        """
        if self.x_length == x_length and self.y_length == y_length:
            return

        self.x_length = x_length
        self.y_length = y_length

        self.setMeshData(meshdata=self.__get_mesh_data(self.x_length, self.y_length))
        self.goal_outline.setData(
            pos=np.array(
                [
                    [0, self.y_length / 2, 0],
                    [-self.x_length, self.y_length / 2, 0],
                    [-self.x_length, -self.y_length / 2, 0],
                    [0, -self.y_length / 2, 0],
                ]
            ),
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

    def set_orientation(self, degrees: float):
        """Set the orientation of the graphic in the scene
        
        :param degrees: The orientation of the graphic in degrees

        """
        if self.orientation == degrees:
            return

        # Rotate locally about the z axis (0, 0, 1)
        self.rotate(degrees - self.orientation, 0, 0, 1, local=True)
        self.orientation = degrees
    
    def __get_mesh_data(self, x_length: float, y_length: float):
        """
        Return a MeshData instance with vertices and faces computed
        for a mesh representing the goal

        :param x_length: The length of the goal in the x direction
        :param y_length: The length of the goal in the y direction
        :returns: the computed MeshData instance 

        """
        # Construct vertices that make up the mesh.
        vertices = [
            [-x_length, y_length / 2, 0],
            [-x_length, -y_length / 2, 0],
            [-x_length, y_length / 2, ROBOT_MAX_HEIGHT_METERS],
            [-x_length, -y_length / 2, ROBOT_MAX_HEIGHT_METERS],
            [0, y_length / 2, 0],
            [0, y_length / 2, ROBOT_MAX_HEIGHT_METERS],
            [0, -y_length / 2, 0],
            [0, -y_length / 2, ROBOT_MAX_HEIGHT_METERS],
        ]

        # Construct triangular faces that make up the mesh.
        # Each face is an array of 3 indices in the points array.
        faces = [
            # Back plate faces
            [0, 1, 2],
            [3, 2, 1],
            # Left plate faces
            [0, 4, 5],
            [5, 2, 0],
            # Right plate faces
            [1, 6, 7],
            [7, 3, 1],
        ]

        return MeshData(vertexes=np.array(vertices), faces=np.array(faces),)