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
            if dist >= 1:
                self.color = QtGui.QColor(
                    255, 0, 255, 255
                )  # Purple for high contrast to show violation.
            elif 0.75 < dist < 1:
                self.color = QtGui.QColor(
                    255, int(self.sigmoid_interpolate(dist, 0.5, 160, 1, 0)), 0, 200
                )
            else:
                self.color = QtGui.QColor(
                    int(self.sigmoid_interpolate(dist, 0, 0, 0.5, 255)),
                    255,
                    0,
                    int(self.sigmoid_interpolate(dist, 0, 0, 0.5, 200)),
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

    def sigmoid_interpolate(self, val, x1, y1, x2, y2):
        """Interpolates a value along a sigmoid curve.
        :param val: Value to interpolate
        :param x1: Lower bound
        :param y1: Value at lower bound
        :param x2: Upper bound
        :param y2: Value at upper bound
        :return: The sigmoid-interpolated value
        """
        xc = (x1 + x2) / 2
        k = 10 / (x2 - x1)
        return y1 + (y2 - y1) / (1 + math.exp(-k * (val - xc)))
