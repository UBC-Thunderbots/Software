import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from software.thunderscope.field.field_layer import FieldLayer
import software.thunderscope.constants as constants
import software.thunderscope.colors as colors
from proto.geometry_pb2 import Polygon, Circle
from proto.visualization_pb2 import Obstacles
from software.networking.threaded_unix_listener import ThreadedUnixListener


class ObstacleLayer(FieldLayer):
    def __init__(self):
        FieldLayer.__init__(self)
        self.obstacle_receiver = ThreadedUnixListener(
            constants.UNIX_SOCKET_BASE_PATH + Obstacles.DESCRIPTOR.full_name,
            Obstacles,
            max_buffer_size=1,
        )
        self.cached_obstacles = Obstacles()

    def paint(self, painter, option, widget):
        """Paint this layer

        :param painter: The painter object to draw with
        :param option: Style information (unused)
        :param widget: The widget that we are painting on

        """

        obstacles = self.obstacle_receiver.maybe_pop()

        if not obstacles:
            obstacles = self.cached_obstacles

        self.cached_obstacles = obstacles

        painter.setPen(pg.mkPen(colors.NAVIGATOR_OBSTACLE_COLOR))

        for polyobstacle in obstacles.polygon:
            polygon_points = [
                QtCore.QPoint(
                    constants.MM_PER_M * point.x_meters,
                    constants.MM_PER_M * point.y_meters,
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
