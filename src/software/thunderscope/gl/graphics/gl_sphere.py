from pyqtgraph.Qt import QtGui
from pyqtgraph.opengl import *
from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem

from software.thunderscope.constants import Colors


class GLSphere(GLMeshItem):
    """Displays a 3D sphere"""

    def __init__(
        self,
        parentItem: GLGraphicsItem = None,
        radius: float = 1,
        rows: int = 10,
        cols: int = 10,
        color: QtGui.QColor = Colors.DEFAULT_GRAPHICS_COLOR,
    ):
        """Initialize the GLSphere
        
        :param parentItem: The parent item of the graphic
        :param radius: The radius of the sphere
        :param rows: The number of rows in the mesh
        :param cols: The number of columns in the mesh
        :param color: The color of the sphere

        """
        super().__init__(parentItem=parentItem, color=color)

        self.x = 0
        self.y = 0
        self.z = 0
        self.radius = 0
        self.set_radius(radius, rows, cols)

    def set_radius(self, radius: float, rows: int = 10, cols: int = 10):
        """Set the radius of the sphere

        :param radius: The radius of the sphere
        :param rows: The number of rows in the mesh
        :param cols: The number of columns in the mesh
        
        """
        if self.radius == radius:
            return

        self.radius = radius
        self.setMeshData(meshdata=MeshData.sphere(rows, cols, self.radius))

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
