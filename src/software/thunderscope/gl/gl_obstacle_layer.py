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

        added_graphics = []
        removed_graphics = []

        if not self.isVisible():
            for index in range(len(self.obstacle_lines)):
                removed_graphics.append(self.obstacle_lines.pop())
            return added_graphics, removed_graphics

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

        # Add or remove GLLinePlotItems from self.obstacle_lines so that we
        # have the same number of GLLinePlotItems as obstacles
        if len(self.obstacle_lines) < len(poly_obstacles):
            num_lines_to_add = len(poly_obstacles) - len(self.obstacle_lines)

            for i in range(num_lines_to_add):
                gl_line_plot_item = GLLinePlotItem()
                self.obstacle_lines.append(gl_line_plot_item)
                added_graphics.append(gl_line_plot_item)

        elif len(self.obstacle_lines) > len(poly_obstacles):
            num_lines_to_remove = len(self.obstacle_lines) - len(poly_obstacles)

            for i in range(num_lines_to_remove):
                removed_graphics.append(self.obstacle_lines.pop())

        for index, poly_obstacle in enumerate(poly_obstacles):
            
            # In order to close the polygon, we need to include the first point at the end of
            # the list of points in the polygon
            polygon_points = list(poly_obstacle.points) + poly_obstacle.points[:1]

            obstacle_line = self.obstacle_lines[index]
            obstacle_line.setData(
                pos=np.array([[point.x_meters, point.y_meters, 0] for point in polygon_points]),
                color=Colors.NAVIGATOR_OBSTACLE_COLOR
            )

        return added_graphics, removed_graphics

        