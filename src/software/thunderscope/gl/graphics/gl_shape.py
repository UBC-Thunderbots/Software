from pyqtgraph.Qt import QtGui
from pyqtgraph.opengl import *
from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem

from software.thunderscope.constants import Colors, LINE_WIDTH
from software.thunderscope.gl.graphics.gl_graphic import GLGraphic
from typing import Optional


class GLShape(GLGraphic):
    """Base class for a graphic that displays a 2D geometric shape on the
    cartesian plane (i.e. x-y plane)
    """

    def __init__(
        self,
        parent_item: Optional[GLGraphicsItem] = None,
        outline_color: QtGui.QColor = Colors.DEFAULT_GRAPHICS_COLOR,
        fill_color: Optional[QtGui.QColor] = None,
        line_width: float = LINE_WIDTH,
    ) -> None:
        """Base constructor for the GLShape

        :param parent_item: The parent item of the graphic
        :param outline_color: The color of the shape's outline
        :param fill_color: The color used to fill the shape, or None if no fill
        :param line_width: The line width of the shape's outline
        """
        super().__init__(
            parent_item=parent_item,
            outline_color=outline_color,
            line_width=line_width,
        )

        self.fill_graphic = None
        self.set_fill_color(fill_color)

    def set_fill_color(self, fill_color: Optional[QtGui.QColor]) -> None:
        """Set the color used to fill the shape

        :param fill_color: The color used to fill the shape, or None if no fill
        """
        if fill_color:
            if not self.fill_graphic:
                self.fill_graphic = GLMeshItem(parentItem=self)
                self._update_shape_data()
            self.fill_graphic.setColor(fill_color)
        else:
            if self.fill_graphic:
                self.fill_graphic.setParentItem(None)
                self.fill_graphic = None
