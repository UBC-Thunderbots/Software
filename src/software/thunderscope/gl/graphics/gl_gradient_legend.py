import OpenGL.GL as GL

import pyqtgraph as pg
from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem
from pyqtgraph.Qt import QtCore, QtGui

from typing import Optional, Dict, Tuple

from software.thunderscope.constants import Colors


class GLGradientLegend(GLGraphicsItem):
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
        labels: Dict[str, float] = {},
        title: str = None,
    ) -> None:
        """Initialize the GLGradientLegend
        
        :param parent_item: The parent item of the graphic
        
        """
        self.size = size
        self.offset = offset
        self.gradient = gradient
        self.labels = labels
        self.title = title

        self.text_pen = QtGui.QPen(Colors.PRIMARY_TEXT_COLOR)
        self.labels_font = QtGui.QFont("Roboto", 8)
        self.title_font = QtGui.QFont("Roboto", 9, QtGui.QFont.Weight.Bold)

        super().__init__(parentItem=parent_item)

    def paint(self):
        self.setupGLState()

        painter = QtGui.QPainter(self.view())
        rect = self.view().rect()

        ## determine max width of all labels
        labelWidth = 0
        labelHeight = 0
        for k in self.labels:
            b = painter.boundingRect(
                QtCore.QRectF(0, 0, 0, 0),
                QtCore.Qt.AlignmentFlag.AlignLeft
                | QtCore.Qt.AlignmentFlag.AlignVCenter,
                str(k),
            )
            labelWidth = max(labelWidth, b.width())
            labelHeight = max(labelHeight, b.height())

        textPadding = 2  # in px

        xR = rect.right()
        xL = rect.left()
        yT = rect.top()
        yB = rect.bottom()

        # Coordinates describe edges of text and bar, additional margins will be added for background
        if self.offset[0] < 0:
            x3 = (
                xR + self.offset[0]
            )  # right edge from right edge of view, offset is negative!
            x2 = x3 - labelWidth - 2 * textPadding  # right side of color bar
            x1 = x2 - self.size[0]  # left side of color bar
        else:
            x1 = xL + self.offset[0]  # left edge from left edge of view
            x2 = x1 + self.size[0]
            x3 = (
                x2 + labelWidth + 2 * textPadding
            )  # leave room for 2x textpadding between bar and text
        if self.offset[1] < 0:
            y2 = (
                yB + self.offset[1]
            )  # bottom edge from bottom of view, offset is negative!
            y1 = y2 - self.size[1]
        else:
            y1 = yT + self.offset[1]  # top edge from top of view
            y2 = y1 + self.size[1]
        self.b = [x1, x2, x3, y1, y2, labelWidth]

        # Draw color bar
        self.gradient.setStart(0, y2)
        self.gradient.setFinalStop(0, y1)
        painter.setBrush(self.gradient)
        rect = QtCore.QRectF(QtCore.QPointF(x1, y1), QtCore.QPointF(x2, y2))
        painter.drawRect(rect)

        # Draw labels
        if self.labels:
            painter.setPen(self.text_pen)
            painter.setFont(self.labels_font)
            tx = x2 + 2 * textPadding  # margin between bar and text
            lh = labelHeight
            lw = labelWidth
            for label in self.labels:
                y = y2 - self.labels[label] * (y2 - y1)
                painter.drawText(
                    QtCore.QRectF(tx, y - lh / 2, lw, lh),
                    QtCore.Qt.AlignmentFlag.AlignLeft
                    | QtCore.Qt.AlignmentFlag.AlignVCenter,
                    str(label),
                )

        # Draw legend title
        if self.title:
            painter.setFont(self.title_font)
            painter.drawText(
                QtCore.QRectF(x1, y1 - lh - 12, 60, lh),
                QtCore.Qt.AlignmentFlag.AlignLeft
                | QtCore.Qt.AlignmentFlag.AlignVCenter,
                self.title,
            )

        painter.end()
