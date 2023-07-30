from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.opengl import *

import numpy as np

from proto.geometry_pb2 import Polygon
from proto.visualization_pb2 import HRVOVisualization

import software.thunderscope.constants as constants
from software.py_constants import *
from software.thunderscope.constants import Colors
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.gl.gl_layer import GLLayer
from software.thunderscope.gl.graphics.gl_circle import GLCircle


class GLHrvoLayer(GLLayer):
    """GLHrvoLayer that visualizes the state of the HRVO Simulator"""

    def __init__(self, name: str, robot_id: int, buffer_size: int = 5):
        """Initialize the GLHrvoLayer

        :param name: The displayed name of the layer
        :param robot_id: The id of the robot which this layer will visualize
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        GLLayer.__init__(self, name)

        self.robot_id = robot_id
        self.hrvo_buffer = ThreadSafeBuffer(buffer_size, HRVOVisualization)
        self.prev_message = HRVOVisualization(robot_id=self.robot_id)

        self.graphics_list.register_graphics_group(
            "velocity_obstacles",
            lambda: GLLinePlotItem(color=Colors.NAVIGATOR_OBSTACLE_COLOR),
        )

    def _update_graphics(self):
        """Fetch and update graphics for the layer"""

        velocity_obstacle_msg = self.prev_message
        while not self.hrvo_buffer.queue.empty():
            msg = self.hrvo_buffer.get(block=False)
            if msg.robot_id == self.robot_id:
                velocity_obstacle_msg = msg

        self.prev_message = velocity_obstacle_msg

        for velocity_obstacle_graphic, velocity_obstacle in zip(
            self.graphics_list.get_graphics(
                "velocity_obstacles", len(velocity_obstacle_msg.velocity_obstacles)
            ),
            velocity_obstacle_msg.velocity_obstacles,
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

            velocity_obstacle_graphic.setData(
                pos=np.array([[point[0], point[1], 0] for point in polygon_points]),
            )
