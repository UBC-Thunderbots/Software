from pyqtgraph.opengl import *

import numpy as np

from proto.tbots_software_msgs_pb2 import PrimitiveSet

from software.thunderscope.constants import Colors
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.gl.graphics.gl_circle import GLCircle
from software.thunderscope.gl.graphics.gl_polygon import GLPolygon

from software.thunderscope.gl.helpers.observable_list import ObservableList


class GLObstacleLayer(GLLayer):
    """GLLayer that visualizes obstacles"""

    def __init__(self, name: str, buffer_size: int = 5) -> None:
        """Initialize the GLObstacleLayer

        :param name: The displayed name of the layer
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        super().__init__(name)

        self.primitive_set_buffer = ThreadSafeBuffer(buffer_size, PrimitiveSet)

        self.poly_obstacle_graphics = ObservableList(self._graphics_changed)
        self.circle_obstacle_graphics = ObservableList(self._graphics_changed)

    def refresh_graphics(self) -> None:
        """Update graphics in this layer"""

        primitive_set = self.primitive_set_buffer.get(
            block=False
        ).robot_primitives.values()

        obstacles = [
            obstacle
            for primitive in primitive_set
            if primitive.HasField("move")
            for obstacle in primitive.move.motion_control.static_obstacles
        ]
        poly_obstacles = [
            poly_obstacle
            for obstacle in obstacles
            for poly_obstacle in obstacle.polygon
        ]
        circle_obstacles = [
            circle_obstacle
            for obstacle in obstacles
            for circle_obstacle in obstacle.circle
        ]

        # Ensure we have the same number of graphics as obstacles
        self.poly_obstacle_graphics.resize(
            len(poly_obstacles),
            lambda: GLPolygon(
                outline_color=Colors.NAVIGATOR_OBSTACLE_COLOR
            ),
        )
        self.circle_obstacle_graphics.resize(
            len(circle_obstacles),
            lambda: GLCircle(outline_color=Colors.NAVIGATOR_OBSTACLE_COLOR),
        )

        for poly_obstacle_graphic, poly_obstacle in zip(
            self.poly_obstacle_graphics, poly_obstacles
        ):
            # In order to close the polygon, we need to include the first point at the end of
            # the list of points in the polygon
            polygon_points = list(poly_obstacle.points) + poly_obstacle.points[:1]

            poly_obstacle_graphic.set_points(
                [[point.x_meters, point.y_meters] for point in polygon_points]
            )

        for circle_obstacle_graphic, circle_obstacle in zip(
            self.circle_obstacle_graphics, circle_obstacles
        ):
            circle_obstacle_graphic.set_radius(circle_obstacle.radius)
            circle_obstacle_graphic.set_position(
                circle_obstacle.origin.x_meters, circle_obstacle.origin.y_meters,
            )
