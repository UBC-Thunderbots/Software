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

    def __init__(self, name: str, buffer_size: int = 5):
        """Initialize the GLObstacleLayer

        :param name: The displayed name of the layer
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        GLLayer.__init__(self, name)

        self.primitive_set_buffer = ThreadSafeBuffer(buffer_size, PrimitiveSet)

        self.graphics_list.register_graphics_group(
            "poly_obstacles",
            lambda: GLLinePlotItem(color=Colors.NAVIGATOR_OBSTACLE_COLOR),
        )
        self.graphics_list.register_graphics_group(
            "circle_obstacles", lambda: GLCircle(color=Colors.NAVIGATOR_OBSTACLE_COLOR)
        )

    def _update_graphics(self):
        """Fetch and update graphics for the layer"""

        primitive_set = self.primitive_set_buffer.get(
            block=False
        ).robot_primitives.values()

        obstacles_ptrs = [
            primitive.move.motion_control.static_obstacles
            for primitive in primitive_set
            if primitive.HasField("move")
        ]

        for obstacles in obstacles_ptrs:
            for obstacle in obstacles:

                for poly_obstacle_graphic, poly_obstacle in zip(
                    self.graphics_list.get_graphics(
                        "poly_obstacles", len(obstacle.polygon)
                    ),
                    obstacle.polygon,
                ):
                    # In order to close the polygon, we need to include the first point at the end of
                    # the list of points in the polygon
                    polygon_points = (
                        list(poly_obstacle.points) + poly_obstacle.points[:1]
                    )

                    poly_obstacle_graphic.setData(
                        pos=np.array(
                            [
                                [point.x_meters, point.y_meters, 0]
                                for point in polygon_points
                            ]
                        ),
                    )

                for circle_obstacle_graphic, circle_obstacle in zip(
                    self.graphics_list.get_graphics(
                        "circle_obstacles", len(obstacle.circle)
                    ),
                    obstacle.circle,
                ):
                    circle_obstacle_graphic.set_radius(circle_obstacle.radius)
                    circle_obstacle_graphic.set_position(
                        circle_obstacle.origin.x_meters,
                        circle_obstacle.origin.y_meters,
                    )
