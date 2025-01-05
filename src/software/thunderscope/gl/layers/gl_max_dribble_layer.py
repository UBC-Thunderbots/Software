from pyqtgraph.opengl import *

from software.thunderscope.constants import DepthValues
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.gl.graphics.gl_circle import GLCircle
from software.thunderscope.gl.graphics.gl_polygon import GLPolygon
import software.python_bindings as tbots_cpp
from proto.import_all_protos import *

from pyqtgraph.Qt import QtGui

import math



class GLMaxDribbleLayer(GLLayer):
    """Visualizes the maximumm dribbling distance when a robot controls the ball."""

    def __init__(self, name: str, buffer_size: int = 5) -> None:
        """Initialize the GLMaxDribbleLayer

        :param name: The displayed name of the layer
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary
        """
        super().__init__(name)
        self.setDepthValue(DepthValues.BACKGROUND_DEPTH)

        self.world_buffer = ThreadSafeBuffer(buffer_size, World)

        self.color = QtGui.QColor(255, 0, 0, 127)

        self.dribble_radius_graphic = GLPolygon(
            parent_item=self, outline_color=self.color
        )
        self.dribble_circle_graphic = GLCircle(
            parent_item=self, outline_color=self.color
        )
        self.dribble_circle_graphic.set_radius(1, 24)

    def refresh_graphics(self) -> None:
        """Update graphics in this layer"""
        world = self.world_buffer.get(block=False)
        if world.HasField("dribble_displacement"):
            dribble_disp = world.dribble_displacement

            dist = tbots_cpp.createSegment(dribble_disp).length()

            self.color = self.color_from_gradient(
                dist,
                [0, 0.5, 1, 1], # last bound needs to be repeated twice to achieve sharp colour snapping when dist > 1.
                [0, 255, 255, 255],
                [255, 160, 0, 0],
                [0, 0, 0, 255],
                [0, 200, 200, 255]
            )
            self.dribble_radius_graphic.set_outline_color(self.color)
            self.dribble_radius_graphic.set_points(
                [
                    (dribble_disp.start.x_meters, dribble_disp.start.y_meters),
                    (dribble_disp.end.x_meters, dribble_disp.end.y_meters),
                ]
            )
            self.dribble_circle_graphic.set_outline_color(self.color)
            self.dribble_circle_graphic.set_position(
                dribble_disp.start.x_meters, dribble_disp.start.y_meters
            )
            self.dribble_circle_graphic.show()
            self.dribble_radius_graphic.show()
        else:
            self.dribble_radius_graphic.hide()
            self.dribble_circle_graphic.hide()
    def color_from_gradient(
            self,
            x: float,
            t_range: list[float],
            r_range: list[int],
            g_range: list[int],
            b_range: list[int],
            a_range: list[int]
    ):
        """Returns a color interpolated from a gradient defined by the point along a sigmoid curve.
        :param x: value to interpolate
        :param t_range: the thresholds for each section in the gradient
        :param r_range: the r values at each threshold
        :param g_range: the g values at each threshold
        :param b_range: the b values at each threshold
        :param a_range: the a values at each threshold
        :return: a color according to the gradient"""

        if not(len(t_range) == len(r_range) == len(g_range) == len(b_range) == len(a_range)):
            return QtGui.QColor(255, 255, 255, 255)
        else:
            if x < t_range[0]:
                return QtGui.QColor(r_range[0],
                                    g_range[0],
                                    b_range[0],
                                    a_range[0])
            elif x > t_range[-1]:
                return QtGui.QColor(r_range[-1],
                                    g_range[-1],
                                    b_range[-1],
                                    a_range[-1])
            else:
                for i in range(len(t_range)-1):
                    if t_range[i] <= x < t_range[i+1]:
                        sig_val = tbots_cpp.sigmoid(x, (t_range[i] + t_range[i+1]) / 2, t_range[i+1] - t_range[i])
                        return QtGui.QColor(
                            int(r_range[i] + (r_range[i+1] - r_range[i]) * sig_val),
                            int(g_range[i] + (g_range[i+1] - g_range[i]) * sig_val),
                            int(b_range[i] + (b_range[i+1] - b_range[i]) * sig_val),
                            int(a_range[i] + (a_range[i+1] - a_range[i]) * sig_val)
                        )
