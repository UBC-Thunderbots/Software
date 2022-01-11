import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from software.thunderscope.field.field_layer import FieldLayer
import software.thunderscope.field.constants as constants
import software.thunderscope.colors as colors
from proto.geometry_pb2 import Polygon, Circle
from proto.visualization_pb2 import Obstacle
from software.networking.threaded_unix_listener import ThreadedUnixListener


class ObstacleLayer(FieldLayer):
    def __init__(self):
        FieldLayer.__init__(self)
        self.obstacle_receiver = ThreadedUnixListener(
            "/tmp/tbots/TbotsProto.Obstacle", Obstacle, max_buffer_size=1,
        )
        self.cached_obstacles = Obstacle()

    def paint(self, painter, option, widget):

        obstacles = self.obstacle_receiver.maybe_pop()

        if not obstacles:
            obstacles = self.cached_obstacles

        self.cached_obstacles = obstacles

        painter.setPen(pg.mkPen(colors.NAVIGATOR_OBSTACLE_COLOR))

        for polyobstacle in obstacles.polygon:
            polygon_points = [
                QtCore.QPoint(
                    constants.MM_TO_M * point.x_meters,
                    constants.MM_TO_M * point.y_meters,
                )
                for point in polyobstacle.points
            ]

            poly = QtGui.QPolygon(polygon_points)
            painter.drawPolygon(poly)

        for circleobstacle in obstacles.circle:
            painter.drawEllipse(
                self.createCircle(
                    constants.MM_TO_M * circleobstacle.origin.x_meters,
                    constants.MM_TO_M * circleobstacle.origin.y_meters,
                    constants.MM_TO_M * circleobstacle.radius,
                )
            )
