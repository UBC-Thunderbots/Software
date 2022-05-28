import pyqtgraph as pg
from proto.visualization_pb2 import HRVOVisualization
from pyqtgraph.Qt import QtCore, QtGui

from software.thunderscope.colors import Colors
from software.py_constants import *
from software.thunderscope.field.field_layer import FieldLayer
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class HRVOLayer(FieldLayer):
    def __init__(self, robot_id, buffer_size=5):
        """Visualize the state of the HRVO Simulator

        :param robot_id: The id of the robot which this layer will visualize
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        FieldLayer.__init__(self)
        self.robot_id = robot_id
        self.hrvo_buffer = ThreadSafeBuffer(buffer_size, HRVOVisualization)

    def paint(self, painter, option, widget):
        """Paint this layer

        :param painter: The painter object to draw with
        :param option: Style information (unused)
        :param widget: The widget that we are painting on

        """

        # Draw the HRVO velocity obstacles and agents
        velocity_obstacle_msg = self.hrvo_buffer.get(block=False)

        if velocity_obstacle_msg.robot_id != self.robot_id:
            return

        painter.setPen(pg.mkPen(Colors.NAVIGATOR_OBSTACLE_COLOR))

        for velocity_obstacle in velocity_obstacle_msg.velocity_obstacles:
            polygon_points = [
                QtCore.QPoint(
                    int(
                        MILLIMETERS_PER_METER
                        * velocity_obstacle.apex.x_component_meters
                    ),
                    int(
                        MILLIMETERS_PER_METER
                        * velocity_obstacle.apex.y_component_meters
                    ),
                ),
                QtCore.QPoint(
                    int(
                        MILLIMETERS_PER_METER
                        * (
                            velocity_obstacle.apex.x_component_meters
                            + velocity_obstacle.left_side.x_component_meters
                        )
                    ),
                    int(
                        MILLIMETERS_PER_METER
                        * (
                            velocity_obstacle.apex.y_component_meters
                            + velocity_obstacle.left_side.y_component_meters
                        )
                    ),
                ),
                QtCore.QPoint(
                    int(
                        MILLIMETERS_PER_METER
                        * (
                            velocity_obstacle.apex.x_component_meters
                            + velocity_obstacle.right_side.x_component_meters
                        )
                    ),
                    int(
                        MILLIMETERS_PER_METER
                        * (
                            velocity_obstacle.apex.y_component_meters
                            + velocity_obstacle.right_side.y_component_meters
                        )
                    ),
                ),
            ]

            velocity_obstacle_triangle = QtGui.QPolygon(polygon_points)
            painter.drawPolygon(velocity_obstacle_triangle)

        for robot_circle in velocity_obstacle_msg.robots:
            painter.drawEllipse(
                self.createCircle(
                    int(MILLIMETERS_PER_METER * robot_circle.origin.x_meters),
                    int(MILLIMETERS_PER_METER * robot_circle.origin.y_meters),
                    int(MILLIMETERS_PER_METER * robot_circle.radius),
                )
            )
