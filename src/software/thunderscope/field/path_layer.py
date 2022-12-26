import pyqtgraph as pg
from proto.tbots_software_msgs_pb2 import PrimitiveSet
from pyqtgraph.Qt import QtCore, QtGui

import software.thunderscope.constants as constants
from software.py_constants import *
from software.thunderscope.colors import Colors
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

        # Cached painter pens and brush
        self.path_pen = pg.mkPen(
            Colors.NAVIGATOR_PATH_COLOR,
            width=constants.LINE_WIDTH,
            style=QtCore.Qt.PenStyle.CustomDashLine,
            dash=[1, 2],
        )
        self.desired_position_pen = pg.mkPen(
            Colors.DESIRED_ROBOT_LOCATION_OUTLINE,
            width=constants.LINE_WIDTH,
            style=QtCore.Qt.PenStyle.CustomDashLine,
            dash=[1, 2],
        )
        self.transparent_brush = pg.mkBrush(Colors.TRANSPARENT)

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
            (
                primitive.move.motion_control.requested_destination,
                primitive.move.final_angle,
            )
            for primitive in primitive_set
            if primitive.HasField("move")
        ]

        # Draw lines representing all paths
        painter.setPen(self.path_pen)
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

        # Draw outlines of robots representing the desired final position and orientation
        painter.setBrush(self.transparent_brush)
        painter.setPen(self.desired_position_pen)
        for dest, final_angle in requested_destinations:
            self.drawRobot(dest, final_angle, painter)
