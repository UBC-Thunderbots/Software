from pyqtgraph.Qt import QtGui
from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem

from software.thunderscope.constants import Colors, LINE_WIDTH
from software.thunderscope.gl.graphics.gl_graphic import GLGraphic
from typing import Optional, override

class GLLineStrip(GLGraphic):
    """Displays a line strip on the cartesian plane (i.e. x-y plane)"""

    def __init__(
        self,
        parent_item: Optional[GLGraphicsItem] = None,
        points: list[tuple[float, float]] = [],
        outline_color: QtGui.QColor = Colors.DEFAULT_GRAPHICS_COLOR,
        line_width: float = LINE_WIDTH,
    ) -> None:
        """Initialize the GLLineStrip

        :param parent_item: The parent item of the graphic
        :param points: A list of 2-tuples representing the line strip points
                       on the cartesian plane. Must be non-empty
        :param outline_color: The color of the line strip's outline
        :param line_width: The line width of the line strip's outline
        """
        super().__init__(
            parent_item=parent_item,
            outline_color=outline_color,
            line_width=line_width,
        )

        self.set_points(points)

    def set_points(self, points: list[tuple[float, float]]) -> None:
        """Update the point data representing the line strip to display.
        A line strip is an ordered sequence of points, where consecutive points
        in the sequence are connected by an edge.

        :param points: A list of 2-tuples representing the line strip points
                       on the cartesian plane. Must be non-empty
        """
        self.points = points
        self._update_shape_data()

    @override
    def _update_shape_data(self) -> None:
        """Update the underlying GLLinePlotItem representing the outline of this line strip"""
        # We get OpenGL errors if we try passing in an empty list of
        # vertices, so return early
        if not self.points:
            return

        vertices = [(point[0], point[1], 0) for point in self.points]
        self.setData(pos=vertices)
