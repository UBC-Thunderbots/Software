from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.opengl import *


class GLRect(GLGridItem):
    """Displays a rectangle parallel to the x-y plane"""

    def __init__(self, color=(255, 255, 255, 127.5)):
        """Initialize the GLRect
        
        :param color: The color of the graphic

        """
        self.x = 0
        self.y = 0
        self.x_length = 0
        self.y_length = 0

        GLGridItem.__init__(self, color=color)

    def set_dimensions(self, x_length: float = 0, y_length: float = 0):
        """Set the dimensions of the rectangle
        
        :param x_length: The length of the rectangle in the x direction
        :param y_length: The length of the rectangle in the y direction

        """
        if x_length == 0 or y_length == 0:
            return

        if self.x_length == x_length and self.y_length == y_length:
            return

        self.setSize(x_length, y_length, 0)
        self.setSpacing(x_length, y_length, 0)

        self.x_length = x_length
        self.y_length = y_length

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
