from pyqtgraph.opengl import *

import numpy as np

from proto.visualization_pb2 import HRVOVisualization

from software.thunderscope.constants import Colors
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.gl.graphics.gl_circle import GLCircle
from software.thunderscope.gl.graphics.gl_polygon import GLPolygon
from software.thunderscope.gl.helpers.observable_list import Change, ChangeAction
from software.thunderscope.gl.helpers.observable_list import ObservableList


class GLHrvoLayer(GLLayer):
    """GLHrvoLayer that visualizes the state of the HRVO Simulator"""

    def __init__(self, name: str, robot_id: int, buffer_size: int = 5) -> None:
        """Initialize the GLHrvoLayer

        :param name: The displayed name of the layer
        :param robot_id: The id of the robot which this layer will visualize
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        super().__init__(name)

        self.robot_id = robot_id
        self.hrvo_buffer = ThreadSafeBuffer(buffer_size, HRVOVisualization)
        self.prev_message = HRVOVisualization(robot_id=self.robot_id)

        self.velocity_obstacle_graphics = ObservableList(self._graphics_changed)
        self.robot_circle_graphics = ObservableList(self._graphics_changed)
        self.trajectory_graphics = ObservableList(self._graphics_changed)

    def refresh_graphics(self) -> None:
        """Update graphics in this layer"""

        velocity_obstacle_msg = self.prev_message
        while not self.hrvo_buffer.queue.empty():
            msg = self.hrvo_buffer.get(block=False)
            if msg.robot_id == self.robot_id:
                velocity_obstacle_msg = msg
                break
        self.prev_message = velocity_obstacle_msg

        # Ensure we have the same number of graphics as protos
        self.velocity_obstacle_graphics.resize(
            len(velocity_obstacle_msg.velocity_obstacles),
            lambda: GLPolygon(outline_color=Colors.NAVIGATOR_OBSTACLE_COLOR),
        )
        self.robot_circle_graphics.resize(
            len(velocity_obstacle_msg.robots),
            lambda: GLCircle(
                outline_color=Colors.NAVIGATOR_OBSTACLE_COLOR, num_points=10
            ),
        )

        for velocity_obstacle_graphic, velocity_obstacle in zip(
            self.velocity_obstacle_graphics, velocity_obstacle_msg.velocity_obstacles,
        ):
            polygon_points = [
                [
                    velocity_obstacle.apex.x_component_meters,
                    velocity_obstacle.apex.y_component_meters,
                ],
                [
                    velocity_obstacle.apex.x_component_meters
                    + velocity_obstacle.left_side.x_component_meters,
                    velocity_obstacle.apex.y_component_meters
                    + velocity_obstacle.left_side.y_component_meters,
                ],
                [
                    velocity_obstacle.apex.x_component_meters
                    + velocity_obstacle.right_side.x_component_meters,
                    velocity_obstacle.apex.y_component_meters
                    + velocity_obstacle.right_side.y_component_meters,
                ],
            ]

            # In order to close the polygon, we need to include the first point at the end of
            # the list of points in the polygon
            polygon_points = polygon_points + polygon_points[:1]

            velocity_obstacle_graphic.set_points(polygon_points)

        for robot_circle_graphic, robot_circle in zip(
            self.robot_circle_graphics, velocity_obstacle_msg.robots,
        ):
            robot_circle_graphic.set_radius(robot_circle.radius)
            robot_circle_graphic.set_position(
                robot_circle.origin.x_meters, robot_circle.origin.y_meters
            )

        # Ensure we have the same number of graphics as protos
        self.trajectory_graphics.resize(
            1, lambda: GLPolygon(outline_color=Colors.NAVIGATOR_PATH_COLOR),
        )

        self.trajectory_graphics[0].set_points(
            [
                [point.x_meters, point.y_meters]
                for point in velocity_obstacle_msg.trajectory.points
            ]
        )
