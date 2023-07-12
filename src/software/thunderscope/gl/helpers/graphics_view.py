import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui

from software.thunderscope.gl.helpers.extended_gl_view_widget import (
    ExtendedGLViewWidget,
)


class GraphicsView(pg.GraphicsView):
    """GraphicsView subclass that uses GLViewWidget as its canvas. 
    This allows 2D graphics to be overlaid on a 3D background.
    """

    def __init__(self):
        """Initialize the GraphicsView"""
        self.gl_view_widget = ExtendedGLViewWidget()

        # Fixes strange bug where mousePos is not initialized
        self.gl_view_widget.mousePos = QtCore.QPointF(0, 0)

        pg.GraphicsView.__init__(self, background=None)
        self.setStyleSheet("background: transparent")
        self.setViewport(self.gl_view_widget)

    def paintEvent(self, event):
        """Propagate paint events to both widgets

        :param event: The event

        """
        self.gl_view_widget.paintEvent(event)
        return pg.GraphicsView.paintEvent(self, event)

    def mousePressEvent(self, event):
        """Propagate mouse events to both widgets
        
        :param event: The event
        
        """
        pg.GraphicsView.mousePressEvent(self, event)
        self.gl_view_widget.mousePressEvent(event)

    def mouseMoveEvent(self, event):
        """Propagate mouse events to both widgets
        
        :param event: The event
        
        """
        pg.GraphicsView.mouseMoveEvent(self, event)
        self.gl_view_widget.mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        """Propagate mouse events to both widgets
        
        :param event: The event
        
        """
        pg.GraphicsView.mouseReleaseEvent(self, event)
        self.gl_view_widget.mouseReleaseEvent(event)

    def wheelEvent(self, event):
        """Propagate mouse wheel events to both widgets
        
        :param event: The event
        
        """
        pg.GraphicsView.wheelEvent(self, event)
        self.gl_view_widget.wheelEvent(event)
