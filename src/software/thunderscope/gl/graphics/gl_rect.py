from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.opengl import *

class GLRect(GLGridItem):

    def __init__(self, color=(255, 255, 255, 127.5)):
        GLGridItem.__init__(self)

        self.x = 0
        self.y = 0
        self.x_length = 0
        self.y_length = 0

        self.setColor(color)

    def setDimensions(self, x_length=0, y_length=0):
        
        if x_length == 0 or y_length == 0:
            return

        if self.x_length == x_length and self.y_length == y_length:
            return
        
        self.setSize(x_length, y_length, 0)
        self.setSpacing(x_length, y_length, 0)

        self.x_length = x_length
        self.y_length = y_length

    def setTranslation(self, x, y):

        if self.x == x and self.y == y:
            return

        self.translate(-self.x, -self.y, 0)

        self.x = x
        self.y = y
        self.translate(self.x, self.y, 0)