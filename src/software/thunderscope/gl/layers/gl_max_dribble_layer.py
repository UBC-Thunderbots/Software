from pyqtgraph.opengl import *

from software.thunderscope.constants import Colors, DepthValues
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.gl.graphics.gl_circle import GLCircle
from software.thunderscope.gl.graphics.gl_polygon import GLPolygon
import software.python_bindings as tbots_cpp

from pyqtgraph.Qt import QtGui

from software.thunderscope.gl.helpers.observable_list import ObservableList

class GLMaxDribbleLayer(GLLayer):
    """GLLayer"""

    def __init__(self, name: str, buffer_size:int = 5) -> None:
        """Initialize the GLMaxDribbleLayer

        :param name: The displayed name of the layer
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary
                            """
        super().__init__(name)
        self.setDepthValue(DepthValues.BACKGROUND_DEPTH)

        self.world_buffer = ThreadSafeBuffer(buffer_size, tbots_cpp.World)

        self.color = QtGui.QColor(255,0,0,255)

        self.dribble_radius_graphic = GLPolygon(parent_item = self, outline_color = self.color)
        self.dribble_circle_graphic = GLCircle(parent_item = self, outline_color = self.color)

    def refresh_graphics(self) -> None:
        """Update graphics in this layer"""

        self.dribble_radius_graphic.set_outline_color(self.color)
        self.dribble_circle_graphic.set_outline_color(self.color)

        dribbleDispSegment = self.world_buffer.get(block=False).dribbleDisplacement

