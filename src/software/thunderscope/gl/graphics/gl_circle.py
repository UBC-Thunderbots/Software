from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.opengl import *

import math
import numpy as np


class GLCircle(GLLinePlotItem):
    """Displays a circle on the x-y plane"""

    def __init__(self, radius=0, num_points=24, color=(1.0, 1.0, 1.0, 0.5)):

        self.x = 0
        self.y = 0
        self.radius = 0
        self.num_points = num_points

        GLLinePlotItem.__init__(self, color=color)
        self.setRadius(radius)

    def setRadius(self, radius):
        if self.radius == radius:
            return

        self.radius = radius

        # Generate points on the circumference of a circle centered at (0,0)
        pi = math.pi
        points = np.array(
            [
                [
                    math.cos(2 * pi / self.num_points * x) * self.radius,
                    math.sin(2 * pi / self.num_points * x) * self.radius,
                    0,
                ]
                for x in range(0, self.num_points + 1)
            ]
        )

        self.setData(pos=points)

    def setColor(self, color):
        self.setData(color=color)

    def setTranslation(self, x, y):

        if self.x == x and self.y == y:
            return

        self.translate(x - self.x, y - self.y, 0)
        self.x = x
        self.y = y
