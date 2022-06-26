import pyqtgraph as pg
from proto.geometry_pb2 import Circle, Polygon
from proto.tbots_software_msgs_pb2 import PrimitiveSet
from pyqtgraph.Qt import QtCore, QtGui

from software.thunderscope.colors import Colors
import software.thunderscope.constants as constants
from software.py_constants import *
from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.thunderscope.field.field_layer import FieldLayer
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class ObstacleLayer(FieldLayer):
    def __init__(self, buffer_size=5):
        """Visualize the obstacles

        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        FieldLayer.__init__(self)
        self.primitive_set_buffer = ThreadSafeBuffer(buffer_size, PrimitiveSet)

    def paint(self, painter, option, widget):
        """Paint this layer

        :param painter: The painter object to draw with
        :param option: Style information (unused)
        :param widget: The widget that we are painting on

        """

        # Draw the obstacles
        primitive_set = self.primitive_set_buffer.get(
            block=False
        ).robot_primitives.values()
        obstacles_ptrs = [
            primitive.move.motion_control.static_obstacles
            for primitive in primitive_set
            if primitive.HasField("move")
        ]

        painter.setPen(pg.mkPen(Colors.NAVIGATOR_OBSTACLE_COLOR))

        for obstacles in obstacles_ptrs:
            for obstacle in obstacles:
                for polyobstacle in obstacle.polygon:
                    polygon_points = [
                        QtCore.QPoint(
                            int(MILLIMETERS_PER_METER * point.x_meters),
                            int(MILLIMETERS_PER_METER * point.y_meters),
                        )
                        for point in polyobstacle.points
                    ]

                    poly = QtGui.QPolygon(polygon_points)
                    painter.drawPolygon(poly)

                for circleobstacle in obstacle.circle:
                    painter.drawEllipse(
                        self.createCircle(
                            int(MILLIMETERS_PER_METER * circleobstacle.origin.x_meters),
                            int(MILLIMETERS_PER_METER * circleobstacle.origin.y_meters),
                            int(MILLIMETERS_PER_METER * circleobstacle.radius),
                        )
                    )
