from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem
from pyqtgraph.Qt import QtCore, QtGui

from typing import Optional, Dict, Tuple

from software.thunderscope.gl.graphics.gl_painter import GLPainter
from software.thunderscope.constants import Colors


class GLGradientLegend(GLPainter):
    """Displays a color gradient rectangle along with text labels denoting 
    the value at specific points along the gradient.

    The graphic is a 2D static overlay painted over top the viewport.
    
    """

    def __init__(
        self,
        parent_item: Optional[GLGraphicsItem] = None,
        size: Tuple[int, int] = (10, 100),
        offset: Tuple[int, int] = (0, 0),
        gradient: QtGui.QLinearGradient = QtGui.QLinearGradient(),
        labels: Dict[str, float] = {"1.0": 1, "0.0": 0},
        title: Optional[str] = None,
    ) -> None:
        """Initialize the GLGradientLegend
        
        :param parent_item: The parent item of the graphic
        :param size: The size (width, height) of the color bar in pixels
        :param offset: The offset (x, y) from the viewport left and top edge 
                       to use when positioning the legend. 
                       If x is negative then the x offset is |x| pixels from 
                       the viewport right edge.
                       If y is negative then the y offset is |y| pixels from 
                       the viewport bottom edge.
        :param gradient: The gradient to use in the color bar
        :param labels: The labels to appear next to the color bar at specified stops
                       Accepts a dict of {"label": stop} pairs where stop is in [0, 1]
        :param title: The optional title to display above the legend
        
        """
        super().__init__(parent_item=parent_item)

        self.size = size
        self.offset = offset
        self.gradient = gradient
        self.labels = labels
        self.title = title

        self.text_pen = QtGui.QPen(Colors.PRIMARY_TEXT_COLOR)
        self.labels_font = QtGui.QFont("Roboto", 8)
        self.title_font = QtGui.QFont("Roboto", 9, QtGui.QFont.Weight.Bold)

        self.add_draw_function(self.draw_gradient_legend)

    def draw_gradient_legend(
        self, painter: QtGui.QPainter, viewport_rect: QtCore.QRect
    ) -> None:
        """Draw the gradient legend
        
        :param painter: The QPainter to perform drawing operations with
        :param viewport_rect: The QRect indicating the viewport dimensions 
        
        """
        # Determine max width of all labels
        label_width = 0
        label_height = 0
        for label in self.labels:
            bounds = painter.boundingRect(
                QtCore.QRectF(0, 0, 0, 0),
                QtCore.Qt.AlignmentFlag.AlignLeft
                | QtCore.Qt.AlignmentFlag.AlignVCenter,
                str(label),
            )
            label_width = max(label_width, bounds.width())
            label_height = max(label_height, bounds.height())

        # Determine x and y coordinates of color bar rectangle edges
        if self.offset[0] < 0:
            x_right = viewport_rect.right() + self.offset[0] - label_width
            x_left = x_right - self.size[0]
        else:
            x_left = viewport_rect.left() + self.offset[0]
            x_right = x_left + self.size[0]
        if self.offset[1] < 0:
            y_bottom = viewport_rect.bottom() + self.offset[1]
            y_top = y_bottom - self.size[1]
        else:
            y_top = viewport_rect.top() + self.offset[1]
            y_bottom = y_top + self.size[1]

        # Draw color bar
        self.gradient.setStart(0, y_bottom)
        self.gradient.setFinalStop(0, y_top)
        painter.setBrush(self.gradient)
        rect = QtCore.QRectF(
            QtCore.QPointF(x_left, y_top), QtCore.QPointF(x_right, y_bottom)
        )
        painter.drawRect(rect)

        # Draw labels
        if self.labels:
            painter.setPen(self.text_pen)
            painter.setFont(self.labels_font)
            for label in self.labels:
                y = y_bottom - self.labels[label] * (y_bottom - y_top)
                painter.drawText(
                    QtCore.QRectF(
                        x_right + 4, y - (label_height / 2), label_width, label_height
                    ),
                    QtCore.Qt.AlignmentFlag.AlignLeft
                    | QtCore.Qt.AlignmentFlag.AlignVCenter,
                    str(label),
                )

        # Draw legend title
        if self.title:
            painter.setFont(self.title_font)
            painter.drawText(QtCore.QPoint(x_left, y_top - label_height), self.title)

    def set_labels(self, labels: Dict[str, float]) -> None:
        """ Update the labels appearing next to the color bar

        :param labels: The labels to appear next to the color bar at specified stops
                       Accepts a dict of {"label": stop} pairs where stop is in [0, 1]
        """
        self.labels = labels
