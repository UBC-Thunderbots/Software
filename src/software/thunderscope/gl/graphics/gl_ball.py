from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.opengl import *

from software.py_constants import BALL_MAX_RADIUS_METERS
from software.thunderscope.constants import Colors

class GLBall(GLMeshItem):

    def __init__(self):

        GLMeshItem.__init__(
            self,
            meshdata=MeshData.sphere(3, 3, BALL_MAX_RADIUS_METERS),
            color=Colors.BALL_COLOR
        )

        self.x = 0
        self.y = 0
        self.z = 0

    def setPosition(self, x, y, z):

        if self.x == x and self.y == y and self.z == z:
            return

        self.translate(-self.x, -self.y, -self.z)

        self.x = x 
        self.y = y
        self.z = z
        self.translate(self.x, self.y, self.z)

        