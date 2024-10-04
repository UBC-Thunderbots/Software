from PyQt6.QtGui import QFont, QColor
from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem
from pyqtgraph.Qt import QtCore, QtGui

from typing import Optional

from software.thunderscope.gl.graphics.gl_painter import GLPainter
from software.thunderscope.constants import Colors


class GLLabel(GLPainter):
    """Displays a 2D text label on the viewport"""

    def __init__(
        self,
        parent_item: Optional[GLGraphicsItem] = None,
        font: QFont = QFont("Roboto", 8),
        text_color: QColor = Colors.PRIMARY_TEXT_COLOR,
        offset: tuple[int, int] = (0, 0),
        text: str = "",
    ) -> None:
        """Initialize the GLGradientLegend

        :param parent_item: The parent item of the graphic
        :param font: The font using to render the text
        :param text_color: The color for rendering the text.
        :param offset: The offset (x, y) from the viewport left and top edge
                       to use when positioning the label.
                       If x is negative then the x offset is |x| pixels from
                       the viewport right edge.
                       If y is negative then the y offset is |y| pixels from
                       the viewport bottom edge.
        :param text: The optional title to display above the legend
        """
        super().__init__(parent_item=parent_item)

        self.text_pen = QtGui.QPen(text_color)
        self.font = font
        self.offset = offset
        self.text = text

        self.add_draw_function(self.draw_label)

    def draw_label(self, painter: QtGui.QPainter, viewport_rect: QtCore.QRect) -> None:
        """Draw the label

        :param painter: The QPainter to perform drawing operations with
        :param viewport_rect: The QRect indicating the viewport dimensions
        """
        # calculate width and height of the label
        painter.setFont(self.font)
        bounds = painter.boundingRect(
            QtCore.QRectF(0, 0, 0, 0),
            QtCore.Qt.AlignmentFlag.AlignLeft | QtCore.Qt.AlignmentFlag.AlignVCenter,
            str(self.text),
        )

        width = round(bounds.width())
        height = round(bounds.height())

        # Determine x and y coordinates of the label
        if self.offset[0] < 0:
            x = viewport_rect.right() + self.offset[0] - width
        else:
            x = viewport_rect.left() + self.offset[0]
        if self.offset[1] < 0:
            y = viewport_rect.bottom() + self.offset[1] - height
        else:
            y = viewport_rect.top() + self.offset[1]

        if self.text:
            painter.drawText(QtCore.QPoint(x, y), self.text)

    def set_text(self, new_text: str) -> None:
        """Update the text being displayed
        :param new_text: new text being displayed
        """
        self.text = new_text
