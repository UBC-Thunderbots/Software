import pyqtgraph as pg
from proto.geometry_pb2 import Circle, Polygon
from proto.visualization_pb2 import Obstacles
from pyqtgraph.Qt import QtCore, QtGui

from software.thunderscope.colors import Colors
import software.thunderscope.constants as constants
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
        self.cached_obstacles = Obstacles()

        self.obstacle_buffer = ThreadSafeBuffer(buffer_size, Obstacles)

    def paint(self, painter, option, widget):
        """Paint this layer

        :param painter: The painter object to draw with
        :param option: Style information (unused)
        :param widget: The widget that we are painting on

        """

        # Draw the obstacles
        obstacles = self.obstacle_buffer.get(block=False)

        painter.setPen(pg.mkPen(Colors.NAVIGATOR_OBSTACLE_COLOR))

        for polyobstacle in obstacles.polygon:
            polygon_points = [
                QtCore.QPoint(
                    int(constants.MM_PER_M * point.x_meters),
                    int(constants.MM_PER_M * point.y_meters),
                )
                for point in polyobstacle.points
            ]

            poly = QtGui.QPolygon(polygon_points)
            painter.drawPolygon(poly)

        for circleobstacle in obstacles.circle:
            painter.drawEllipse(
                self.createCircle(
                    constants.MM_PER_M * circleobstacle.origin.x_meters,
                    constants.MM_PER_M * circleobstacle.origin.y_meters,
                    constants.MM_PER_M * circleobstacle.radius,
                )
            )
