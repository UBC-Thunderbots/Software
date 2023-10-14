from pyqtgraph.Qt import QtGui
from pyqtgraph.opengl import *
from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem

from software.thunderscope.constants import Colors, LINE_WIDTH

from typing import Optional, List, Tuple

import numpy as np
import software.thunderscope.gl.helpers.triangulate as triangulate


class GLPolygon(GLLinePlotItem):
    """Displays a polygon on the cartesian plane (i.e. x-y plane)"""

    def __init__(
        self,
        parentItem: Optional[GLGraphicsItem] = None,
        points: List[Tuple[float, float]] = [],
        outline_color: QtGui.QColor = Colors.DEFAULT_GRAPHICS_COLOR,
        fill_color: Optional[QtGui.QColor] = None,
        line_width: float = LINE_WIDTH,
    ):
        """Initialize the GLRect
        
        :param parentItem: The parent item of the graphic
        :param outline_color: The color of the polygon's outline
        :param fill_color: The color used to fill the polygon, or None if no fill
        :param line_width: The line width of the polygon's outline

        """
        super().__init__(parentItem=parentItem, width=line_width)

        self.x = 0
        self.y = 0

        self.points = None
        self.fill_graphic = None

        self.set_points(points)
        self.set_outline_color(outline_color)
        self.set_fill_color(fill_color)

    def set_points(self, points: List[Tuple[float, float]]):
        """Update the point data representing the polygon to display.
        A polygon is an ordered sequence of points, where consecutive points
        in the sequence are connected an edge, and an edge connects the first
        and last points.

        :param points: A list of 2-tuples representing the polygon points
                       on the cartesian plane

        """
        self.points = points

        vertices = [(point[0], point[1], 0) for point in self.points]
        self.setData(pos=vertices)

        if self.fill_graphic:
            faces = triangulate.earclip(self.points)
            meshdata = MeshData(vertexes=vertices, faces=np.array(faces))
            self.fill_graphic.setMeshData(meshdata=meshdata)

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

    def set_outline_color(self, outline_color: QtGui.QColor):
        """Set the color of the polygon's outline
        
        :param outline_color: The color of the polygon's outline 
        
        """
        self.setData(color=outline_color)

    def set_fill_color(self, fill_color: Optional[QtGui.QColor]):
        """Set the color used to fill the polygon
        
        :param fill_color: The color used to fill the polygon, or None if no fill 
        
        """
        if fill_color:
            if not self.fill_graphic:
                self.fill_graphic = GLMeshItem(parentItem=self)
                self.set_points(self.points)
            self.fill_graphic.setColor(fill_color)
        else:
            if self.fill_graphic:
                self.fill_graphic.setParentItem(None)
                self.fill_graphic = None
