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


class GLObstacleLayer(GLLayer):
    def __init__(self, buffer_size=5):
        GLLayer.__init__(self)

        self.primitive_set_buffer = ThreadSafeBuffer(buffer_size, PrimitiveSet)

        self.obstacle_lines = []

    def updateGraphics(self):

        if not self.isVisible():
            return [], self.clearGraphicsList(self.obstacle_lines)

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

        added_graphics, removed_graphics = self.setupGraphicsList(
            graphics_list=self.obstacle_lines,
            num_graphics=len(poly_obstacles),
            graphic_init_func=lambda: GLLinePlotItem(),
        )

        for index, poly_obstacle in enumerate(poly_obstacles):

            # In order to close the polygon, we need to include the first point at the end of
            # the list of points in the polygon
            polygon_points = list(poly_obstacle.points) + poly_obstacle.points[:1]

            obstacle_line = self.obstacle_lines[index]
            obstacle_line.setData(
                pos=np.array(
                    [[point.x_meters, point.y_meters, 0] for point in polygon_points]
                ),
                color=Colors.NAVIGATOR_OBSTACLE_COLOR,
            )

        return added_graphics, removed_graphics
