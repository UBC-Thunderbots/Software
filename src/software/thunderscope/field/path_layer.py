import pyqtgraph as pg
from proto.tbots_software_msgs_pb2 import PrimitiveSet
from pyqtgraph.Qt import QtCore, QtGui

import software.thunderscope.constants as constants
from software.py_constants import *
from software.thunderscope.colors import Colors
from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.thunderscope.field.field_layer import FieldLayer
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class PathLayer(FieldLayer):
    def __init__(self, buffer_size=5):
        """Visualizes paths from the navigator

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
        primitive_set = self.primitive_set_buffer.get(
            block=False
        ).robot_primitives.values()
        paths = [
            primitive.move.motion_control.path
            for primitive in primitive_set
            if primitive.HasField("move")
        ]

        requested_destinations = [
            primitive.move.motion_control.requested_destination
            for primitive in primitive_set
            if primitive.HasField("move")
        ]

        painter.setPen(
            pg.mkPen(Colors.NAVIGATOR_PATH_COLOR, width=constants.LINE_WIDTH)
        )

        for path in paths:
            polygon_points = [
                QtCore.QPoint(
                    int(MILLIMETERS_PER_METER * point.x_meters),
                    int(MILLIMETERS_PER_METER * point.y_meters),
                )
                for point in path.points
            ]
            poly = QtGui.QPolygon(polygon_points)
            painter.drawPolyline(poly)

        offset_1 = 30
        offset_2 = 60
        for dest in requested_destinations:
            x_mm = int(MILLIMETERS_PER_METER * dest.x_meters)
            y_mm = int(MILLIMETERS_PER_METER * dest.y_meters)
            # polygon represents an X to mark the requested destination
            polygon_points = [
                QtCore.QPoint(x_mm, y_mm + offset_1),
                QtCore.QPoint(x_mm + offset_1, y_mm + offset_2),
                QtCore.QPoint(x_mm + offset_2, y_mm + offset_1),
                QtCore.QPoint(x_mm + offset_1, y_mm),
                QtCore.QPoint(x_mm - offset_1, y_mm + offset_2),
                QtCore.QPoint(x_mm - offset_2, y_mm + offset_1),
                QtCore.QPoint(x_mm, y_mm - offset_1),
                QtCore.QPoint(x_mm - offset_1, y_mm - offset_2),
                QtCore.QPoint(x_mm - offset_2, y_mm - offset_1),
                QtCore.QPoint(x_mm - offset_1, y_mm),
                QtCore.QPoint(x_mm + offset_1, y_mm - offset_2),
                QtCore.QPoint(x_mm + offset_2, y_mm - offset_1),
            ]

            poly = QtGui.QPolygon(polygon_points)
            painter.setBrush(pg.mkBrush(Colors.NAVIGATOR_PATH_COLOR))
            painter.drawPolygon(poly)
