import pyqtgraph as pg
from proto.geometry_pb2 import Circle, Polygon
from proto.visualization_pb2 import Obstacles
from pyqtgraph.Qt import QtCore, QtGui

import software.thunderscope.colors as colors
import software.thunderscope.constants as constants
from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.thunderscope.field.field_layer import FieldLayer

from proto.world_pb2 import (
    SimulatorTick,
    World,
    WorldState,
    Validation,
    ValidationStatus,
    ValidationGeometry,
)


class ValidationLayer(FieldLayer):
    def __init__(self):
        FieldLayer.__init__(self)
        self.validation_receiver = ThreadedUnixListener(
            constants.UNIX_SOCKET_BASE_PATH + "validation",
            Validation,
            max_buffer_size=1,
        )
        self.cached_validations = Validation()

    def paint(self, painter, option, widget):
        """Paint this layer

        :param painter: The painter object to draw with
        :param option: Style information (unused)
        :param widget: The widget that we are painting on

        """

        validations = self.obstacle_receiver.maybe_pop()

        if not validations:
            validations = self.cached_obstacles

        self.cached_validations = obstacles

        painter.setPen(pg.mkPen(colors.NAVIGATOR_OBSTACLE_COLOR))

        for polyvalidation in obstacles.polygon:
            polygon_points = [
                QtCore.QPoint(
                    constants.MM_PER_M * point.x_meters,
                    constants.MM_PER_M * point.y_meters,
                )
                for point in polyvalidation.points
            ]

            poly = QtGui.QPolygon(polygon_points)
            painter.drawPolygon(poly)

        for circlevalidation in obstacles.circle:
            painter.drawEllipse(
                self.createCircle(
                    constants.MM_PER_M * circlevalidation.origin.x_meters,
                    constants.MM_PER_M * circlevalidation.origin.y_meters,
                    constants.MM_PER_M * circlevalidation.radius,
                )
            )
