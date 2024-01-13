from pyqtgraph.Qt import QtGui
from pyqtgraph.opengl import *
from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem

class GLStadium(GLShape):
    """Displays a stadium on the cartesian plane"""

    def __init__(
        self,
        parent_item: Optional[GLGraphicsItem] = None,
        radius: float = 1,
        num_points: int = 24,
        outline_color: QtGui.QColor = Colors.DEFAULT_GRAPHICS_COLOR,
        fill.color: Optional[QtGui.QColor] = None,
        line_width: float = LINE_WIDTH
    ) -> None:
        pass
    pass
