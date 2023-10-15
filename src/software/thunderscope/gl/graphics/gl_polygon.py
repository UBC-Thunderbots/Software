from pyqtgraph.Qt import QtGui
from pyqtgraph.opengl import *
from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem

from software.thunderscope.constants import Colors, LINE_WIDTH
from typing import Optional, List, Tuple

import software.thunderscope.gl.helpers.triangulate as triangulate
from software.thunderscope.gl.graphics.gl_shape import GLShape

import numpy as np


class GLPolygon(GLShape):
    """Displays a polygon on the cartesian plane (i.e. x-y plane)"""

    def __init__(
        self,
        parent_item: Optional[GLGraphicsItem] = None,
        points: List[Tuple[float, float]] = [],
        outline_color: QtGui.QColor = Colors.DEFAULT_GRAPHICS_COLOR,
        fill_color: Optional[QtGui.QColor] = None,
        line_width: float = LINE_WIDTH,
    ):
        """Initialize the GLPolygon
        
        :param parent_item: The parent item of the graphic
        :param points: A list of 2-tuples representing the polygon points 
                       on the cartesian plane. Must be non-empty
        :param outline_color: The color of the polygon's outline
        :param fill_color: The color used to fill the polygon, or None if no fill
        :param line_width: The line width of the polygon's outline

        """
        super().__init__(
            parent_item=parent_item, 
            outline_color=outline_color,
            fill_color=fill_color,
            line_width=line_width,
        )

        self.set_points(points)

    def set_points(self, points: List[Tuple[float, float]]):
        """Update the point data representing the polygon to display.
        A polygon is an ordered sequence of points, where consecutive points
        in the sequence are connected an edge, and an edge connects the first
        and last points.

        :param points: A list of 2-tuples representing the polygon points
                       on the cartesian plane. Must be non-empty

        """
        self.points = points
        self._update_shape_data()

    def _update_shape_data(self):
        """Update the underlying GLLinePlotItem and GLMeshItem representing
        the outline and fill of this shape
        """
        # We get OpenGL errors if we try passing in an empty list of
        # vertices, so return early
        if not self.points:
            return

        vertices = [(point[0], point[1], 0) for point in self.points]
        self.setData(pos=vertices)

        if self.fill_graphic:
            faces = triangulate.earclip(self.points)
            meshdata = MeshData(vertexes=vertices, faces=np.array(faces))
            self.fill_graphic.setMeshData(meshdata=meshdata)
