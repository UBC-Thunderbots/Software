import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from software.thunderscope.field.field_layer import FieldLayer
import software.thunderscope.constants as constants
import software.thunderscope.colors as colors
import queue
from proto.geometry_pb2 import Polygon, Circle
from proto.visualization_pb2 import Obstacles
from software.networking.threaded_unix_listener import ThreadedUnixListener


class ObstacleLayer(FieldLayer):
    def __init__(self):
        FieldLayer.__init__(self)
        self.cached_obstacles = Obstacles()

        self.obstacle_buffer = queue.Queue(10)  # TODO pass in buffer size everywhere

    def paint(self, painter, option, widget):
        """Paint this layer

        :param painter: The painter object to draw with
        :param option: Style information (unused)
        :param widget: The widget that we are painting on

        """

        try:
            obstacles = self.obstacle_buffer.get_nowait()
        except queue.Empty as empty:
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
