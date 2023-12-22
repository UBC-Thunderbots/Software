from pyqtgraph.opengl import *

from proto.visualization_pb2 import ObstacleList

from software.thunderscope.constants import Colors, DepthValues
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.gl.graphics.gl_circle import GLCircle
from software.thunderscope.gl.graphics.gl_polygon import GLPolygon

from software.thunderscope.gl.helpers.observable_list import ObservableList


class GLObstacleLayer(GLLayer):
    """GLLayer that visualizes obstacles"""

    def __init__(self, name: str, buffer_size: int = 1) -> None:
        """Initialize the GLObstacleLayer

        :param name: The displayed name of the layer
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        super().__init__(name)
        self.setDepthValue(DepthValues.BACKGROUND_DEPTH)

        self.obstacles_list_buffer = ThreadSafeBuffer(buffer_size, ObstacleList)

        self.poly_obstacle_graphics = ObservableList(self._graphics_changed)
        self.circle_obstacle_graphics = ObservableList(self._graphics_changed)

    def refresh_graphics(self) -> None:
        """Update graphics in this layer"""
        # TODO (NIMA): Should we just return if there's no new val? Why redraw if nothing changed?
        obstacles = self.obstacles_list_buffer.get(block=False).obstacles

        poly_obstacles = []
        circle_obstacles = []
        for obstacle in obstacles:
            if obstacle.HasField("polygon"):
                poly_obstacles.append(obstacle.polygon)
            else:
                circle_obstacles.append(obstacle.circle)

        # Ensure we have the same number of graphics as obstacles
        self.poly_obstacle_graphics.resize(
            len(poly_obstacles),
            lambda: GLPolygon(outline_color=Colors.NAVIGATOR_OBSTACLE_COLOR),
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
