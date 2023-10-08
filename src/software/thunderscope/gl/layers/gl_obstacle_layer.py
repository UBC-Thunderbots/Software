from pyqtgraph.opengl import *

import numpy as np

from proto.tbots_software_msgs_pb2 import PrimitiveSet

from software.thunderscope.constants import Colors
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.gl.graphics.gl_circle import GLCircle

from software.thunderscope.gl.helpers.observable_list import ObservableList


class GLObstacleLayer(GLLayer):
    """GLLayer that visualizes obstacles"""

    def __init__(self, name: str, buffer_size: int = 5):
        """Initialize the GLObstacleLayer

        :param name: The displayed name of the layer
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        super().__init__(name)

        self.primitive_set_buffer = ThreadSafeBuffer(buffer_size, PrimitiveSet)

        self.poly_obstacle_graphics = ObservableList(self._graphics_changed)
        self.circle_obstacle_graphics = ObservableList(self._graphics_changed)

    def refresh_graphics(self):
        """Update graphics in this layer"""

        primitive_set = self.primitive_set_buffer.get(
            block=False
        ).robot_primitives.values()

        obstacles_ptrs = [
            primitive.move.motion_control.static_obstacles
            for primitive in primitive_set
            if primitive.HasField("move")
        ]

        poly_obstacle_graphics_index = 0
        circle_obstacle_graphics_index = 0

        for obstacles in obstacles_ptrs:
            for obstacle in obstacles:

                for poly_obstacle in obstacle.polygon:

                    # Get a previously cached graphic or create a new one
                    if poly_obstacle_graphics_index >= len(self.poly_obstacle_graphics):
                        poly_obstacle_graphic = GLLinePlotItem(
                            color=Colors.NAVIGATOR_OBSTACLE_COLOR, width=3.0,
                        )
                        self.poly_obstacle_graphics.append(poly_obstacle_graphic)
                    else:
                        poly_obstacle_graphic = self.poly_obstacle_graphics[
                            poly_obstacle_graphics_index
                        ]
                    poly_obstacle_graphics_index += 1

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

                for circle_obstacle in obstacle.circle:

                    # Get a previously cached graphic or create a new one
                    if circle_obstacle_graphics_index >= len(
                        self.circle_obstacle_graphics
                    ):
                        circle_obstacle_graphic = GLCircle(
                            color=Colors.NAVIGATOR_OBSTACLE_COLOR
                        )
                        self.circle_obstacle_graphics.append(circle_obstacle_graphic)
                    else:
                        circle_obstacle_graphic = self.circle_obstacle_graphics[
                            circle_obstacle_graphics_index
                        ]
                    circle_obstacle_graphics_index += 1

                    circle_obstacle_graphic.set_radius(circle_obstacle.radius)
                    circle_obstacle_graphic.set_position(
                        circle_obstacle.origin.x_meters,
                        circle_obstacle.origin.y_meters,
                    )

        # Remove graphics we don't need anymore
        while poly_obstacle_graphics_index < len(self.poly_obstacle_graphics):
            self.poly_obstacle_graphics.pop()
        while circle_obstacle_graphics_index < len(self.circle_obstacle_graphics):
            self.circle_obstacle_graphics.pop()
