from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.opengl import *

from software.py_constants import BALL_MAX_RADIUS_METERS
from software.thunderscope.constants import Colors


class GLBall(GLMeshItem):
    """Displays a 3D sphere representing the ball"""

    def __init__(self):

        self.x = 0
        self.y = 0
        self.z = 0

        GLMeshItem.__init__(
            self,
            meshdata=MeshData.sphere(3, 3, BALL_MAX_RADIUS_METERS),
            color=Colors.BALL_COLOR,
        )

    def setPosition(self, x, y, z):

        if self.x == x and self.y == y and self.z == z:
            return

        self.translate(x - self.x, y - self.y, z - self.z)
        self.x = x
        self.y = y
        self.z = z
