from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.opengl import *

import numpy as np

from proto.geometry_pb2 import Circle, Polygon
from proto.tbots_software_msgs_pb2 import PrimitiveSet

import software.thunderscope.constants as constants
from software.py_constants import *
from software.thunderscope.constants import Colors
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.gl.gl_layer import GLLayer
from software.thunderscope.gl.graphics.gl_circle import GLCircle


class GLObstacleLayer(GLLayer):
    """GLLayer that visualizes obstacles"""

    def __init__(self, buffer_size=5):
        """Initialize the GLObstacleLayer

        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        GLLayer.__init__(self)

        self.primitive_set_buffer = ThreadSafeBuffer(buffer_size, PrimitiveSet)

        self.line_graphics = []
        self.circle_graphics = []

    def updateGraphics(self):
        """Update the GLGraphicsItems in this layer

        :returns: tuple (added_graphics, removed_graphics)
            - added_graphics - List of the added GLGraphicsItems
            - removed_graphics - List of the removed GLGraphicsItems
        
        """
        if not self.isVisible():
            return (
                [], 
                self.clearGraphicsList(self.line_graphics) 
                + self.clearGraphicsList(self.circle_graphics)
            )

        primitive_set = self.primitive_set_buffer.get(
            block=False
        ).robot_primitives.values()

        obstacles_ptrs = [
            primitive.move.motion_control.static_obstacles
            for primitive in primitive_set
            if primitive.HasField("move")
        ]
        poly_obstacles = [
            poly_obstacle
            for obstacles in obstacles_ptrs
            for obstacle in obstacles
            for poly_obstacle in obstacle.polygon
        ]
        circle_obstacles = [
            circle_obstacle
            for obstacles in obstacles_ptrs
            for obstacle in obstacles
            for circle_obstacle in obstacle.circle
        ]

        added_line_graphics, removed_line_graphics = self.setupGraphicsList(
            graphics_list=self.line_graphics,
            num_graphics=len(poly_obstacles),
            graphic_init_func=lambda: GLLinePlotItem(color=Colors.NAVIGATOR_OBSTACLE_COLOR),
        )

        added_circle_graphics, removed_circle_graphics = self.setupGraphicsList(
            graphics_list=self.circle_graphics,
            num_graphics=len(circle_obstacles),
            graphic_init_func=lambda: GLCircle(color=Colors.NAVIGATOR_OBSTACLE_COLOR),
        )

        added_graphics = added_line_graphics + added_circle_graphics
        removed_graphics = removed_line_graphics + removed_circle_graphics

        for line_graphic, poly_obstacle in zip(self.line_graphics, poly_obstacles):

            # In order to close the polygon, we need to include the first point at the end of
            # the list of points in the polygon
            polygon_points = list(poly_obstacle.points) + poly_obstacle.points[:1]

            line_graphic.setData(
                pos=np.array(
                    [[point.x_meters, point.y_meters, 0] for point in polygon_points]
                ),
            )

        for circle_graphic, circle_obstacle in zip(self.circle_graphics, circle_obstacles):

            circle_graphic.setRadius(circle_obstacle.radius)
            circle_graphic.setPosition(
                circle_obstacle.origin.x_meters,
                circle_obstacle.origin.y_meters,
            )

        return added_graphics, removed_graphics
