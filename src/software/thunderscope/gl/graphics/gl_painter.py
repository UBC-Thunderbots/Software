from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem
from pyqtgraph.Qt import QtCore, QtGui

from typing import Optional, Callable

class GLPainter(GLGraphicsItem):
    """2D static overlay painted over top the viewport.
    This graphic enables the use of QPainter method calls to draw to the viewport.
    """
    
    def __init__(self, parent_item: Optional[GLGraphicsItem] = None) -> None:
        """Initialize the GLPainter
        
        :param parent_item: The parent item of the graphic
        
        """
        super().__init__(parentItem=parent_item)
        self.draw_functions = []

    def add_draw_function(self, draw_function: Callable[[QtGui.QPainter, QtCore.QRect], None]):
        """Register a draw function with this GLPainter.
        
        The draw function must accept a QPainter that it will use to perform
        drawing operations and a QRect that indicates the viewport dimensions.
        The draw function should not call end() on the QPainter.

        :param draw_function: The draw function to register

        """
        self.draw_functions.append(draw_function)

    def paint(self):
        """Called by the GLViewWidget to draw this graphic"""

        self.setupGLState()

        painter = QtGui.QPainter(self.view())
        viewport_rect = self.view().rect()

        for draw_function in self.draw_functions:
            draw_function(painter, viewport_rect)

        painter.end()
