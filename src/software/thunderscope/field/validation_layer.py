import pyqtgraph as pg
from proto.geometry_pb2 import Circle, Polygon
from proto.visualization_pb2 import Obstacles
from proto.world_pb2 import (SimulatorTick, ValidationGeometry,
                             ValidationProto, ValidationStatus, World,
                             WorldState)
from pyqtgraph.Qt import QtCore, QtGui

import software.thunderscope.colors as colors
import software.thunderscope.constants as constants
from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.thunderscope.field.field_layer import FieldLayer


class ValidationLayer(FieldLayer):
    def __init__(self):
        FieldLayer.__init__(self)
        self.validation_receiver = ThreadedUnixListener(
            constants.UNIX_SOCKET_BASE_PATH + "validation",
            ValidationProto,
            max_buffer_size=1,
            convert_from_any=False,
        )

        self.cached_validation = ValidationProto()

    def paint(self, painter, option, widget):
        """Paint this layer

        :param painter: The painter object to draw with
        :param option: Style information (unused)
        :param widget: The widget that we are painting on

        """

        validation = self.validation_receiver.maybe_pop()

        if not validation:
            validation = self.cached_validation

        self.cached_validation = validation

        for geometry, status in zip(validation.geometry, validation.status):

            if status == ValidationStatus.PASS:
                painter.setPen(pg.mkPen(colors.VALIDATION_PASSED_COLOR, width=3))
            if status == ValidationStatus.FAIL:
                painter.setPen(pg.mkPen(colors.VALIDATION_FAILED_COLOR, width=3))
            if status == ValidationStatus.PENDING:
                painter.setPen(pg.mkPen(colors.VALIDATION_PENDING_COLOR, width=3))

            for circle in geometry.circles:
                painter.drawEllipse(
                    self.createCircle(
                        constants.MM_PER_M * circle.origin.x_meters,
                        constants.MM_PER_M * circle.origin.y_meters,
                        constants.MM_PER_M * circle.radius,
                    )
                )

            for polygon in geometry.polygons:
                polygon_points = [
                    QtCore.QPoint(
                        constants.MM_PER_M * point.x_meters,
                        constants.MM_PER_M * point.y_meters,
                    )
                    for point in polygon.points
                ]

                poly = QtGui.QPolygon(polygon_points)
                painter.drawPolygon(poly)
