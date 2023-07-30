from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.opengl import *

from software.thunderscope.constants import Colors


class GLSphere(GLMeshItem):
    """Displays a 3D sphere"""

    def __init__(self, radius: float, color=Colors.BALL_COLOR, rows: int = 6, cols: int = 6):
        """Initialize the GLSphere
        
        :param radius: The radius of the sphere
        :param color: The color of the sphere
        :param rows: The number of rows in the mesh
        :param cols: The number of columns in the mesh

        """
        self.x = 0
        self.y = 0
        self.z = 0
        self.radius = radius

        GLMeshItem.__init__(
            self, meshdata=MeshData.sphere(rows, cols, self.radius), color=color,
        )

    def set_radius(self, radius: float, rows: int = 3, cols: int = 3):
        """Set the radius of the sphere

        :param radius: The radius of the sphere
        :param rows: The number of rows in the mesh
        :param cols: The number of columns in the mesh
        
        """
        if self.radius == radius:
            return

        self.radius = radius
        self.setMeshData(MeshData.sphere(rows, cols, self.radius))

    def set_position(self, x: float, y: float, z: float):
        """Set the position of the graphic in the scene
        
        :param x: The x coordinate to position the graphic at
        :param y: The y coordinate to position the graphic at
        :param z: The z coordinate to position the graphic at
        
        """
        if self.x == x and self.y == y and self.z == z:
            return

        self.translate(x - self.x, y - self.y, z - self.z)
        self.x = x
        self.y = y
        self.z = z
